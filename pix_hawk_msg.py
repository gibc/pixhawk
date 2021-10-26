#"""
#Example of how to filter for specific mavlink messages coming from the
#autopilot using pymavlink.

#Can also filter within recv_match command - see "Read all parameters" example
#"""
# Import mavutil
from pymavlink import mavutil
import math
from threading import Thread, Lock
import time
import os

#print(os.environ)


class aharsData:
    def __init__(self, roll, pitch, heading, altitude, climb, groundspeed, airspeed, fix_type, gnd_track, wind_speed, wind_dir):
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

class mavlinkmsg (Thread):
    def __init__(self):
        Thread.__init__(self)
        self.roll = 0
        self.pitch = 0
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
        
        self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00', baud=19200)
        #self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00')

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_WIND, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS3, -1)
        #self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        #self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 2)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ADSB_VEHICLE, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        
    def sm_climb(self, new_val, n):
        new_av = self.old_ave * (n-1)/n + new_val/n
        self.old_av = new_av
        return new_av
        
        
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
                    print('vel_z ', vel_z)
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
                        self.heading = dic['heading']
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
                        roll = dic['roll']
                        self.roll = math.degrees(roll)
                        #print("roll: ", roll)
            
                        pitch = dic['pitch']
                        self.pitch = math.degrees(pitch)
                        #print("pitch: ", pitch)
            
                        yaw = dic['yaw']
                        yaw = math.degrees(yaw)
                        #print("yaw: ", yaw)
                
                    
                #time.sleep(.1)
            
        #AOA if msg.get_type() == 11020:
            #print("\n\n*****Got message: %s*****" % msg.get_type())
            #print("Message: %s" % msg)
            #print("")
            
            except:
                pass
        print("stopped mavlinkmsg thread")
            
    def getAharsData(self, inData):
        #print("getAharsData()")
        #return inData
        if(self.msglock.acquire(blocking=False)):
        #if(False):
            newData = aharsData(self.roll, self.pitch, self.heading, self.altitude, self.climb, self.groundspeed, self.airspeed, self.fix_type, self.gnd_track, self.wind_speed, self.wind_dir)
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