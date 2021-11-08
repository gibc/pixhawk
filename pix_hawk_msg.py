#"""
#Example of how to filter for specific mavlink messages coming from the
#autopilot using pymavlink.

#Can also filter within recv_match command - see "Read all parameters" example
#"""
# Import mavutil
from re import X
import re
from pymavlink import mavutil
import math
from threading import Thread, Lock
import time
import os
import traceback

#from pynput import keyboard

class aharsData:
    def __init__(self, roll=-1, pitch=-1, heading=-1, altitude=-1, climb=-1, groundspeed=-1, airspeed=-1, 
                fix_type=-1, gnd_track=-1, wind_speed=-1, wind_dir=-1, xmag=-1, ymag=-1, zmag=-1):
        #print('aharsData init')
        self.roll = roll
        self.pitch = pitch
        self.heading = heading
        self.altitude = altitude
        self.climb = climb
        self.groundspeed = groundspeed
        self.airspeed = airspeed
        self.fix_type = fix_type
        self.gnd_track = gnd_track
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag

class mavlinkmsg (Thread):
    _instance = None

    def __init__(self):
        Thread.__init__(self)
        self.roll = 0
        self.roll_rad = 0
        self.pitch = 0
        self.pitch_rad = 0
        self.heading = 0
        self.altitude = 0
        self.climb = 0
        self.groundspeed = 0
        self.airspeed = 0
        self.fix_type = 0
        self.wind_speed = 0
        self.wind_dir = 0
        self.gnd_track = 0
        self.buf_len = 10
        self.alt_buf = list(range(0,self.buf_len))
        self.alt_buf_idx = 0
        self.climb_buf_time = 0
        self.cur_alt = 0
        self.msglock = Lock()
        self.run_thread = True
        self.old_ave = 0
        self.old_xmag_average = 0
        self.old_ymag_average = 0
        self.ymag_max = -1000
        self.ymag_min = 1000
        self.xmag_max = -1000
        self.xmag_min = 1000
        self.xmag = -1
        self.ymag = -1
        self.zmag = -1

        self.smothed_heading = 0

        #self.cur_key = 0
        #self.listener = keyboard.Listener(on_press = self.on_press, suppress=True)
        #self.listener.start()
        #self.listener.join()

        """for mag cal"""
        self.xmag_list = []
        self.ymag_list = []  
        self.mag_list_count = 0
        self.xmag_ave = 0
        self.ymag_ave = 0
        
        self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00', baud=19200)
        #self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00')
        self.master.param_fetch_one('COMPASS_OFS_X')
        self.master.param_fetch_one("COMPASS_OFS_Y")
        self.master.param_fetch_one("COMPASS_OFS_Z")

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_WIND, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS3, -1)
        #self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        #self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 2)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ADSB_VEHICLE, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 5)
        
    """ 
    def on_press(self, key):
        self.cur_key = key.char
        print('got key: ', key.char)
    """ 

    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            _instance = mavlinkmsg()
            _instance.start()
        return _instance

    
    def sm_climb(self, new_val, n):
        new_av = self.old_ave * (n-1)/n + new_val/n
        self.old_av = new_av
        return new_av

    def xmag_average(self, new_val, n):
        xmag_average = self.old_xmag_average * (n-1)/n + new_val/n
        self.old_xmag_average = xmag_average
        return xmag_average

    def ymag_average(self, new_val, n):
        ymag_average = self.old_ymag_average * (n-1)/n + new_val/n
        self.old_ymag_average = ymag_average
        return ymag_average

    def heading_average(self, new_val, n):
        diff = self.smothed_heading - new_val
        heading_average = new_val
        if abs(diff) < 10:
            heading_average = self.smothed_heading * (n-1)/n + new_val/n
            self.smothed_heading = heading_average
        else:
            self.smothed_heading = new_val

        return heading_average

    def un_tilt_mag(self, magx, magy, magz):

        with self.msglock:
        
            #print('roll ', self.roll)
            #print('pitch ', self.pitch)
            magx_corrected = magx * math.cos(self.pitch_rad) + magz * math.sin(self.pitch_rad)

            """NOTE: had to invert sign of roll to make this work"""
            magy_corrected = magx * math.sin(-self.roll_rad) * math.sin(self.pitch_rad) +  \
                         magy * math.cos(-self.roll_rad) - \
                         magz * math.sin(-self.roll_rad) * math.cos(self.pitch_rad)
            
            """magy_corrected = magx * math.sin(self.pitch_rad)  +  \
                         magy * math.sin(self.roll_rad)*math.sin(self.pitch_rad) - \
                         magz * math.cos(self.roll_rad) * math.sin(self.pitch_rad)"""

        return (magx_corrected, magy_corrected)
        
    def get_climb_rate(self, alt):
        #print('alt' , alt)
        try:
            alt_delta = alt - self.cur_alt
            self.cur_alt = alt
            if abs(alt_delta) <= 5:
                return 0
            
            #print('alt_delta',alt_delta)
            
            #self.cur_alt = alt
            
            ms_time = time.time() * 1000
            #print('alt_buf', self.alt_buf)
            ms_time = time.time() * 1000
            ms_interval = ms_time - self.climb_buf_time
            
            self.climb_buf_time = ms_time
            if ms_interval != 0:
                rate = alt_delta / ms_interval # feet per millisecond          
                rate = rate * 1000 # feet per second
                rate = rate * 60 # feet per minute
            #print('rate', rate)
            else:
                rate = 0        
        
            """
            cur_alt = sum(self.alt_buf)/self.buf_len
            #print('cur_alt' , cur_alt)
            self.alt_buf[self.alt_buf_idx] = alt
            self.alt_buf_idx += 1
            if self.alt_buf_idx >= self.buf_len:
                self.alt_buf_idx = 0
            #print('self.alt_buf_idx', self.alt_buf_idx)
            new_alt = sum(self.alt_buf)/self.buf_len
            #print('new_alt' , new_alt)
        
            alt_delta = new_alt - cur_alt
            #return alt_delta
            #print('alt_delta' , alt_delta)
                               
            if ms_interval != 0:
                rate = alt_delta / ms_interval # feet per millisecond          
                rate = rate * 1000 # feet per second
                rate = rate * 60 # feet per minute
            #print('rate', rate)
            else:
                rate = 0
           
        #alt_delta_ave = sum(self.climb_buf)/10
            #print('rate', rate)
            """
            #print('rate', rate)
            
        except Exception as e:
            print(str(e))
        return rate
        
    
    def run(self):
        #return
        print("started mavlinkmsg thread")
        while self.run_thread:
            #time.sleep(.1)
            try:
                msg = self.master.recv_match(blocking=True)
                if not msg:
                    continue
                if msg.get_type() == 'BAD_DATA':
                    continue
                
                #print(msg.get_type)
                
                if msg.get_type() == 'PARAM_VALUE':
                    
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    dic = msg.to_dict()
                    param_id = dic['param_id']
                    #print('param_id ', param_id)
                    
                    
                    param_value = dic['param_value']
                    #print('param_value ', param_value)
                    
                    if param_id == 'COMPASS_OFS_X':
                        self.COMPASS_OFS_X = param_value = dic['param_value']
                        print('self.COMPASS_OFS_X ', self.COMPASS_OFS_X)
                        
                    if param_id == 'COMPASS_OFS_Y':
                        self.COMPASS_OFS_Y = param_value = dic['param_value']
                        print('self.COMPASS_OFS_Y ', self.COMPASS_OFS_Y)
                        
                    if param_id == 'COMPASS_OFS_Z':
                        self.COMPASS_OFS_Z = param_value = dic['param_value']
                        print('self.COMPASS_OFS_Z ', self.COMPASS_OFS_Z)
                    
                if msg.get_type() == 'SCALED_IMU':   
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    """
                     this gives reasonable results for vertically oriented compass
                     where back of unit is indicated direction
                    """
                    declination = 4.75
                    xmag_av = -0.609
                    ymag_av = -37.887

                    dic = msg.to_dict()
                    xmag = dic['xmag']
                    self.xmag  = xmag
                    xmag -= xmag_av

                    """
                    if xmag > self.xmag_max:
                        self.xmag_max = xmag
                    if xmag < self.xmag_min:
                        self.xmag_min = xmag
                    """    

                    #xmag_ave = self.xmag_average(xmag, 500)
                    #print('xmag_ave ', xmag_ave)
                    
                    ymag = dic['ymag']
                    self.ymag = ymag
                    ymag -= ymag_av
                    ymag = -ymag #invert for compass orientation

                    self.zmag = dic['zmag']

                    """
                    if ymag > self.ymag_max:
                        self.ymag_max = ymag
                    if ymag < self.ymag_min:
                        self.ymag_min = ymag
                    """
                    if False:
                        if self.mag_list_count < 1000:
                            self.xmag_list.append(xmag)
                            self.ymag_list.append(ymag)
                            self.mag_list_count += 1
                            print('mag_list_count ', self.mag_list_count)
                        elif self.mag_list_count == 1000:
                            self.xmag_ave = sum(self.xmag_list) / len(self.xmag_list)
                            self.ymag_ave = sum(self.ymag_list) / len(self.ymag_list)
                            print('xmag_av ', self.xmag_ave)
                            print('ymag_av ', self.ymag_ave)
                        else:
                            print('xmag_av ', self.xmag_ave)
                            print('ymag_av ', self.ymag_ave)

                    #print('xmag {} ymag {}'.format(xmag,ymag))

                    #print('minx {} maxx {} miny {} maxy {}'.format(self.xmag_min, self.xmag_max, self.ymag_min, self.ymag_max))
                    #print('x_offset {} y_offset {}'.format((self.xmag_max+self.xmag_min)/2, (self.ymag_max+self.ymag_min/2)))

                    #ymag_ave = self.ymag_average(ymag, 500)
                    #print('ymag_ave ', ymag_ave)

                    zmag = dic['zmag']

                    corr_mags = self.un_tilt_mag(xmag, ymag, zmag)
                    xmag = corr_mags[0]
                    ymag = corr_mags[1]
                    
                    heading = math.degrees(math.atan(xmag/ymag))
                    if ymag > 0:
                        heading = 90 - heading
                    else:
                        heading = 270 - heading
                    if ymag == 0:
                        if xmag < 0:
                            heading = 180
                        else:
                            heading = 0
                    heading = heading + declination
                    if heading > 360:
                        heading = heading - 360
                    #print('roll ', self.roll)

                    """handle jump from 360 to 0"""
                    diff = heading - self.heading
                    #if diff < 10 and diff > -10:
                        #heading = self.heading_average(heading, 5)
                    #else:
                        #self.heading_average = heading

                    heading = self.heading_average(heading, 5)
                        
                    #print('heading ', heading)

                    """switched from vfr_hud"""
                    self.heading = heading 
                    
                
                if msg.get_type() == 'RAW_IMU':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    dic = msg.to_dict()
                    
                    xmag = dic['xmag']
                    #xmag = xmag - self.COMPASS_OFS_X
                    #print('xmag ', xmag)
                    
                    ymag = dic['ymag']
                    #ymag = ymag - self.COMPASS_OFS_Y
                    #print('ymag ', ymag)
                    
                    zmag = dic['zmag']
                    #zmag = zmag - self.COMPASS_OFS_Z
                    #print('zmag ', zmag)

                    if self.mag_list_count < 1000:
                        self.xmag_list.append(xmag)
                        self.ymag_list.append(ymag)
                        self.mag_list_count += 1
                        print('mag_list_count ', self.mag_list_count)
                    else:
                        xmag_av = self.xmag_list(sum) / len(self.xmag_list)
                        ymag_av = self.ymag_list(sum) / len(self.ymag_list)
                        print('xmag_av ', xmag_av)
                        print('ymag_av ', ymag_av)

                    
                    #heading = math.degrees(math.atan(ymag/xmag))
                    # from ardupilot example
                    heading = math.degrees(math.atan(-ymag/xmag))
                    if heading < 0:
                        heading += 360
                    """
                    heading = heading - 90
                    if heading < 0:
                        print('heading < 0')
                        heading = 360 - heading
                    if heading > 360:
                        print('heading > 360')
                        heading = 360 - heading
                    """
                    
                    if ymag > 0:
                        heading = 90 - math.degrees(math.atan(ymag/xmag))
                    if ymag < 0:
                        heading = 270 - math.degrees(math.atan(ymag/xmag))
                    if ymag == 0:
                        if xmag < 0:
                            heading = 180
                        if xmag > 0:
                            heading = 0
                    
                    #if heading > 360:
                    #    heading = heading - 360
                    #if heading < 0:
                    #    heading = 360 + heading
                    print('\nheading ', heading)
                    
                # NOTE: I get this message if requestd!!
                #print(msg.get_type())
                #print("Message: %s" % msg)
                """
                NOTE!! this message is just a constant if 'wind sensor' not installed
                """
                if msg.get_type() == 'WIND':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    dic = msg.to_dict()
                    self.wind_dir = dic['direction']
                    self.wind_speed = dic['speed']
                    #pass
                    
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    dic = msg.to_dict()
                    vel_z = dic['vz'] #gnd speed Z cm/s (altitude, positive down)
                    vel_z = vel_z * 1.9685  # to feet/min
                    vel_z = -vel_z # to positive u
                    #print('vel_z ', vel_z)
                    with self.msglock:                   
                        self.climb = vel_z
                        
                    
                
                if msg.get_type() == 'ADSB_VEHICLE':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    dic = msg.to_dict()
                    callsign = dic['callsign']
                    print("callsign: ", callsign)
                    adsb_altitude = .00328 * dic['altitude']
                    print("adsb_altitude: ", adsb_altitude)
                    ICAO_address = dic['ICAO_address']
                    print("ICAO_address: ", ICAO_address)
                
                if msg.get_type() == 'AHRS3':
                    with self.msglock:
                        altitude = dic['altitude']
                        self.altitude = 3.2808 * altitude
                        print("altitude: ", self.altitude)
            
                if msg.get_type() == 'AHRS2':
                    #self.msglock.acquire()
                    with self.msglock:
                        #print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)
                        """
                        dic = msg.to_dict()
                        roll = dic['roll']
                        self.roll = math.degrees(roll)
                        #print("roll: ", roll)
            
                        pitch = dic['pitch']
                        self.pitch = math.degrees(pitch)
                        #print("pitch: ", pitch)
            
                        yaw = dic['yaw']
                        yaw = math.degrees(yaw)
                        #print("yaw: ", yaw)
                        """
                        altitude = dic['altitude']
                        self.altitude = 3.2808 * altitude
                        print("altitude: ", self.altitude)
                        #print("")
                    #self.msglock.release()
            
                #if msg.get_type() == 'GPS_RAW_INT':
                if msg.get_type() == 'GPS2_RAW':
                    with self.msglock:
                        #print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)
                        dic = msg.to_dict()
                    #lat = dic['lat']
                    ##print("lat: ", lat)
                    # lon = dic['lon']
                    #print("lon: ", lon)
                    #alt = dic['alt']
                    #print("alt: ", alt*.00328084)
                        satellites_visible = dic['satellites_visible']
                        #print("satellites_visible: ", satellites_visible)
                        fix_type = dic['fix_type']
                        self.fix_type = fix_type
                        
                        self.gnd_track = dic['cog'] / 100 #convert from 100th of degresss to degrees
                        #print("gnd_track: ", self.gnd_track)
                        #print("")
            
                if msg.get_type() == 'VFR_HUD':
                    with self.msglock:
                        #print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)
                        dic = msg.to_dict()
                        """self.heading = dic['heading'] switched to DYI compass"""
                        #print("heading: ", self.heading)
                        #alt = dic['alt']
                        #print("alt: ", alt*3.28084)
                        self.airspeed = dic['airspeed']*2.237 
                        #print("airspeed: ", airspeed)#m/s
                        #self.climb = dic['climb']*2.237 
                        #print('climb: ', climb) #climb rate m/s
                        self.groundspeed = dic['groundspeed']*2.237 
                        #print('groundspeed: ', groundspeed)
                        altitude = dic['alt']
                        self.altitude = 3.2808 * altitude
                        #print("altitude: ", self.altitude)
                        
                        """ switch to gps_int msg value
                        climb = dic['climb']
                        self.climb = round(climb * 196.85)  # meters/sec to feet/mi
                        #self.climb = round(self.climb, -1)
                        self.climb = self.sm_climb(self.climb, 5)
                        self.climb = round(self.climb)
                        #self.climb = self.get_climb_rate(self.altitude)
                        #print('climb: ', self.climb)
                        """
                        
                        #print("climb: ", self.climb)
                        groundspeed = dic['groundspeed']
                        self.groundspeed = groundspeed * 2.23694  # meters/sec to mph
                        #print("groundspeed: ", self.groundspeed)
                        #print("")
                    #self.msglock.release()
                """
                if msg.get_type() == 'EKF_STATUS_REPORT':
                    print("\n\n*****Got message: %s*****" % msg.get_type())
                    print("Message: %s" % msg)
                    print("")
                    
                if msg.get_type() == 'NAV_CONTROLLER_OUTPUT':
                    print("\n\n*****Got message: %s*****" % msg.get_type())
                    print("Message: %s" % msg)
                    print("")
                 """
                
                if msg.get_type() == 'ATTITUDE':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    with self.msglock:
                        #print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)
                        dic = msg.to_dict()
                        self.roll_rad = dic['roll']
                        self.roll = math.degrees(self.roll_rad)
                        #print("roll: ", self.roll)
            
                        self.pitch_rad = dic['pitch']
                        self.pitch = math.degrees(self.pitch_rad)
                        #print("pitch: ", self.pitch)
            
                        yaw = dic['yaw']
                        yaw = math.degrees(yaw)
                        #print("yaw: ", yaw)
                
                    
                #time.sleep(.1)
            
        #AOA if msg.get_type() == 11020:
            #print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
            #print("")
            
            except Exception:
                traceback.print_exc()
    
        print("stopped mavlinkmsg thread")
            
    def getAharsData(self, inData):
        #print("getAharsData()")
        #return inData
        if(self.msglock.acquire(blocking=False)):
        #if(False):
            newData = aharsData(self.roll, self.pitch, self.heading, self.altitude, 
                self.climb, self.groundspeed, self.airspeed, self.fix_type, self.gnd_track, self.wind_speed, 
                self.wind_dir, self.xmag, self.ymag, self.zmag)
            self.msglock.release()
            return newData
        else:
            return inData
        
        
    def request_message_interval(self, message_id: int, frequency_hz: float):

            #Request MAVLink message in a desired frequency,
            #documentation for SET_MESSAGE_INTERVAL:
            #https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

            #Args:
            #message_id (int): MAVLink message ID
            #frequency_hz (float): Desired frequency in Hz
        interval = -1
        if frequency_hz > 0:
            interval = 1e6 / frequency_hz

        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id, # The MAVLink message ID
            interval, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
            0, 0, 0, 0)

# Create the connection
# From topside computer
#master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00'
#                                    , baud=115200)

#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_WIND, 1)
#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 1)
#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 1)
#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1)
#request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1)
#AOArequest_message_interval(11020, 1)

if __name__ == '__main__':
    msgthd = mavlinkmsg()
    msgthd.start()

    ahdata = (-1,-1,-1,-1)

    while True:
        ahdata = msgthd.getAharsData(ahdata)
    
        #print("roll: ", ahdata.roll)
        #print("pitch: ", ahdata.pitch)
        #print("heading: ", ahdata.heading)
        #print("")
        
    time.sleep(.1)

"""
while True:
    try:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'BAD_DATA':
            continue
        #print(msg.get_type())
        #print("Message: %s" % msg)
        if msg.get_type() == 'WIND':
            print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)
            
        if msg.get_type() == 'AHRS2':
            print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)
            dic = msg.to_dict()
            roll = dic['roll']
            roll = math.degrees(roll)
            print("roll: ", roll)
            
            pitch = dic['pitch']
            pitch = math.degrees(pitch)
            print("pitch: ", pitch)
            
            yaw = dic['yaw']
            yaw = math.degrees(yaw)
            print("yaw: ", yaw)
            print("")
            
        if msg.get_type() == 'GPS2_RAW':
            print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)
            dic = msg.to_dict()
            lat = dic['lat']
            print("lat: ", lat)
            lon = dic['lon']
            print("lon: ", lon)
            alt = dic['alt']
            print("alt: ", alt*.00328084)
            satellites_visible = dic['satellites_visible']
            print("satellites_visible: ", satellites_visible)
            print("")
            
        if msg.get_type() == 'VFR_HUD':
            print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)
            dic = msg.to_dict()
            heading = dic['heading']
            print("heading: ", heading)
            alt = dic['alt']
            print("alt: ", alt*3.28084)
            airspeed = dic['airspeed']*2.237 
            print("airspeed: ", airspeed)#m/s
            climb = dic['climb']*2.237 
            print('climb: ', climb) #climb rate m/s
            groundspeed = dic['groundspeed']*2.237 
            print('groundspeed: ', groundspeed)
            print("")
        
        if msg.get_type() == 'EKF_STATUS_REPORT':
            print("\n\n*****Got message: %s*****" % msg.get_type())
            print("Message: %s" % msg)
            print("")
            
        #AOA if msg.get_type() == 11020:
            #print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
            #print("")
            
            
            
        
    except:
        pass
"""