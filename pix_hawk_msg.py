#"""
#Example of how to filter for specific mavlink messages coming from the
#autopilot using pymavlink.

#Can also filter within recv_match command - see "Read all parameters" example
#"""
# Import mavutil
#from posixpath import join
#from re import S, X
#import re
from distutils.command.config import config
from numpy import float32, true_divide
from pymavlink import mavutil
#from pymavlink import mavextra

import math
from threading import Thread, Lock
import time
import os
import traceback
from pix_hawk_compass_ops import CompassOps
import mavextra
from PhidgetThread import PhidgetThread
from PhidgetThread import PhidgetMag
from pix_hawk_adsb import AdsbDict, AdsbVehicle
import pix_hawk_config
from pix_hawk_util import DebugPrint, Math

class aharsData:
    def __init__(self, roll=-1, pitch=-1, heading=-1, altitude=-1, climb=-1, groundspeed=-1, airspeed=-1, 
                fix_type=-1, gnd_track=-1, wind_speed=-1, wind_dir=-1, xmag=-1, ymag=-1, zmag=-1,
                xacc=-1, yacc=-1, zacc=-1, gps_alt=-1, lat=-1, lon=-1, baro_press=-1):
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
        self.lat = lat
        self.lon = lon
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc
        self.gps_alt = gps_alt
        self.baro_press = baro_press
        

class mavlinkmsg (Thread):
    _instance = None
    _instance_count = 0
    _run_thread = True


    def __init__(self, gps_manager=None):
        Thread.__init__(self)
        self.gps_manager = gps_manager

        #self.compass_ops = CompassOps('mag_params/home_params.txt')
        #if start_compass_thread:
        self.phigetThread = PhidgetThread.get_instance()
        self.adsb_dic = AdsbDict.get_instance()
        self.vh = None
        self.compass_ops = CompassOps('mag_params/new_keik_params.txt')
        self.ATTITUDE = None
        self.RAW_IMU = None
        self.declination = None
        self.SENSOR_OFFSETS = None
        self.roll = 0
        self.roll_rad = 0
        self.pitch = 0
        self.pitch_rad = 0
        self.heading = 0
        self.baro_press = -1
        self.altitude = 0
        self.climb = 0
        self.groundspeed = 0
        self.airspeed = 0
        self.fix_type = 0
        self.lat = 0
        self.lon = 0
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
        self.old_zmag_average = 0
        self.ymag_max = -1000
        self.ymag_min = 1000
        self.xmag_max = -1000
        self.xmag_min = 1000
        self.xmag = -1
        self.ymag = -1
        self.zmag = -1
        self.xacc = -1
        self.yacc = -1
        self.zacc = -1
        self.gps_alt = -1
        self.tail_count = 0

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
        #self.master.close()
        self.master.param_fetch_one('COMPASS_OFS_X')
        self.master.param_fetch_one("COMPASS_OFS_Y")
        self.master.param_fetch_one("COMPASS_OFS_Z")

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_WIND, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS3, -1)
        #self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ADSB_VEHICLE, 10)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SENSOR_OFFSETS, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 5)
        
        
    """ 
    def on_press(self, key):
        self.cur_key = key.char
        print('got key: ', key.char)
    """ 

    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            cls._instance = mavlinkmsg()
            cls._run_thread = True
            cls._instance.start()
        cls._instance_count += 1
        return cls._instance
    
    @classmethod
    def put_instance(cls):
        cls._instance_count -= 1
        if cls._instance_count <= 0:
            cls._run_thread = False
            cls._instance = None
        
    
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

    def zmag_average(self, new_val, n):
        zmag_average = self.old_zmag_average * (n-1)/n + new_val/n
        self.old_zmag_average = zmag_average
        return zmag_average

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
        while mavlinkmsg._run_thread:
            #time.sleep(.1)
            try:
                msg = self.master.recv_match(blocking=True)
                if not msg:
                    continue
                if msg.get_type() == 'BAD_DATA':
                    continue
                
                #print(msg.get_type)

                if msg.get_type() == 'SCALED_PRESSURE':
                    dic = msg.to_dict()
                    self.baro_press = dic['press_abs']

                    # alt meters = 44330 * (1 - (measured pres/baro pre) ^ (1/5.255))
                    #in_mecury = 30.28
                    #hpa = 33.86389 * in_mecury
                    #baro_pressure = hpa # set this from atis, convert from in to pa
                    #lt = 44330 * (1 - (pres/baro_pressure ) ** (1/5.255))
                    #self.baro_alt = alt * 3.28084 # convert meters to ftSS

                if msg.get_type() == 'SENSOR_OFFSETS':
                    self.SENSOR_OFFSETS = msg
                    DebugPrint.print('self.SENSOR_OFFSET X', self.SENSOR_OFFSETS.mag_ofs_x)
                    DebugPrint.print('self.SENSOR_OFFSET Y', self.SENSOR_OFFSETS.mag_ofs_y)
                    DebugPrint.print('self.SENSOR_OFFSET Z', self.SENSOR_OFFSETS.mag_ofs_z)
                
                """if msg.get_type() == 'PARAM_VALUE':
                    
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
                        print('self.COMPASS_OFS_Z ', self.COMPASS_OFS_Z)"""
                
                # gib - form comments and Fixes #1179 say mag values a mgause (milli gause)
                if msg.get_type() == 'SCALED_IMU':   
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)

                    
                    
                    dic = msg.to_dict()

                    self.xacc = dic['xacc']
                    self.yacc = dic['yacc']
                    self.zacc = dic['zacc']
                    
                    declination = 4.75
                    

                   
                    xmag = dic['xmag']
                    self.xmag  = xmag
                    #xmag += 19
                    xmag_ave = int(self.xmag_average(xmag, 1000))
                    
                    
                    ymag = dic['ymag']
                    self.ymag = ymag
                    #ymag -= 30
                    ymag_ave = int(self.ymag_average(ymag, 1000))
                    
                    zmag = dic['zmag']
                    self.zmag = zmag
                    #zmag -=120
                    zmag_ave = int(self.zmag_average(zmag, 1000))
                    
                    self.mag_list_count = self.mag_list_count + 1

                    #print('xmag {0:03d} ymag {1:03d} zmag {2:03d}'.format(self.xmag,self.ymag,self.zmag))
                    #print('xmag_av {0:03d} ymag_av {1:03d} zmag_av {2:03d} count {3:03d}'.format(xmag_ave,ymag_ave,zmag_ave, 
                    #    int(self.mag_list_count)))

            
                    heading = self.compass_ops._get_heading(self.xmag,self.ymag,self.zmag, self.pitch, self.roll)
                    #heading = self.compass_ops._get_heading(self.ymag,self.xmag,self.zmag, self.pitch, self.roll)
                    #self.heading = heading 
                    
 
                if msg.get_type() == 'RAW_IMU':

                    

                    

                    #phidgetMags = self.phigetThread.getMagFeild()
                    #msg.xmag = phidgetMags.xmag * 100
                    #msg.ymag = phidgetMags.ymag * 100
                    #msg.zmag = phidgetMags.zmag * 100
        
        

                    self.RAW_IMU = msg

                    
                    

                    if self.ATTITUDE != None:
                        #heading = mavextra.mag_heading(self.RAW_IMU,self.ATTITUDE, self.declination, self.SENSOR_OFFSETS, (0,0,0), s_factor=.97)
                        #heading = mavextra.mag_heading(self.RAW_IMU,self.ATTITUDE, self.declination, s_factor=.97)
                        if self.phigetThread != None:
                            yaw = self.phigetThread.get_yaw()
                        else:
                            yaw = 0

                        
                        #self.heading = heading
                        self.heading = yaw

                        DebugPrint.print('heading ',self.heading)

                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)

                    """dic = msg.to_dict()
                    
                    xmag = dic['xmag']
                    
                    
                    ymag = dic['ymag']
                    
                    
                    zmag = dic['zmag']"""
                    

                    

                    
                    #heading = math.degrees(math.atan(ymag/xmag))
                    # from ardupilot example
                    """heading = math.degrees(math.atan(-ymag/xmag))
                    if heading < 0:
                        heading += 360"""

                    """
                    heading = heading - 90
                    if heading < 0:
                        print('heading < 0')
                        heading = 360 - heading
                    if heading > 360:
                        print('heading > 360')
                        heading = 360 - heading
                    """
                    
                    """if ymag > 0:
                        heading = 90 - math.degrees(math.atan(ymag/xmag))
                    if ymag < 0:
                        heading = 270 - math.degrees(math.atan(ymag/xmag))
                    if ymag == 0:
                        if xmag < 0:
                            heading = 180
                        if xmag > 0:
                            heading = 0"""
                    
                    #if heading > 360:
                    #    heading = heading - 360
                    #if heading < 0:
                    #    heading = 360 + heading
                    #print('\nheading ', heading)
                    
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

                if msg.get_type() == 'GPS_RAW_INT':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    with self.msglock:
                        dic = msg.to_dict()
                        self.lat = dic['lat']/10000000
                    
                        self.lon = dic['lon']/10000000
                        
                        satellites_visible = dic['satellites_visible']
                        #print("satellites_visible: ", satellites_visible)
                        fix_type = dic['fix_type']
                        self.fix_type = fix_type

                        if self.fix_type < 3:
                            self.lat = 39.932138
                            self.lon = -105.065293
                        
                        self.gnd_track = dic['cog'] / 100 #convert from 100th of degresss to degrees
                        if pix_hawk_config.MockHeading > 0:
                            self.gnd_track = pix_hawk_config.MockHeading

                        self.gps_alt = dic['alt'] * 0.00328084

                        #if self.fix_type < 4:
                        #    self.gps_alt = self.baro_alt

                        
                    
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    with self.msglock:
                        dic = msg.to_dict()
                                      
                        vel_z = dic['vz'] #gnd speed Z cm/s (altitude, positive down)
                        vel_z = vel_z * 1.9685  # to feet/min
                        vel_z = -vel_z # to positive u
                    
                                       
                        self.climb = vel_z
                        
                    
                
                if msg.get_type() == 'ADSB_VEHICLE':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    """
                    1	ADSB_FLAGS_VALID_COORDS	
                    2	ADSB_FLAGS_VALID_ALTITUDE	
                    4	ADSB_FLAGS_VALID_HEADING	
                    8	ADSB_FLAGS_VALID_VELOCITY	
                    16	ADSB_FLAGS_VALID_CALLSIGN	
                    32	ADSB_FLAGS_VALID_SQUAWK	
                    64	ADSB_FLAGS_SIMULATED	
                    128	ADSB_FLAGS_VERTICAL_VELOCITY_VALID	
                    256	ADSB_FLAGS_BARO_VALID	
                    32768	ADSB_FLAGS_SOURCE_UAT

                    Mode S code A50720  in hex for N423DS
                    """	

                    dic = msg.to_dict()

                    flags = dic['flags']

                    fstr = "{0:b}".format(flags)

                    cord_valid = flags & 1 == 1
                    call_sign_valid = flags & 10 == 10
                    heading_valid = flags & 4 == 4

                    callsign = str(dic['callsign'])
                    print("callsign: ", callsign)
                    adsb_heading = dic['heading'] / 100 # to degrees from hundreth?
                    #adsb_heading -= 90
                    #if adsb_heading < 0:
                    #    adsb_heading = 360 + adsb_heading
                    adsb_heading = int(adsb_heading)
                    
                    hor_velocity = dic['hor_velocity'] * 0.0223694 # cm/s to mph
                    ver_velocity = dic['ver_velocity'] * 1.9685  # cm/s to ft/min 
                    lat = dic['lat']/10000000
                    lon = dic['lon']/10000000
                    adsb_altitude = .00328 * dic['altitude']
                    print("adsb_altitude: ", adsb_altitude)
                    ICAO_address = str(dic['ICAO_address'])
                    print("                               ICAO_address: ", ICAO_address)
                    #Don't create new adbs vehical if recieved from adasb out transmitter
                    #    instead, create synthetic vehical from gps msg data

                    if ICAO_address != pix_hawk_config.icao and callsign != pix_hawk_config.callsign:
                        if cord_valid and heading_valid and call_sign_valid:

                            dist = self.adsb_dic.vehicleInLimits(self.lat, self.lon, self.gps_alt, lat, lon, adsb_altitude)
                            if dist > 0:
                                #if pix_hawk_config.Use1090Radio:
                                self.adsb_dic.updateVehicle(ICAO_address, callsign, lat, lon, 
                                    adsb_altitude, hor_velocity, ver_velocity, adsb_heading, True, dist)
                    else:
                        if self.gps_manager != None:
                            self.gps_manager.update_gps_listener('sb', 3, self.lat, self.lon, 
                                adsb_altitude, hor_velocity, ver_velocity, adsb_heading)

                        #Global.update_origin_ap(ICAO_address, callsign, lat, lon, 
                                    #adsb_altitude, hor_velocity, ver_velocity, adsb_heading)

                if msg.get_type() == 'AHRS3':
                    with self.msglock:
                        dic = msg.to_dict()
                        altitude = dic['altitude']
                        self.altitude = 3.2808 * altitude
                        print("altitude: ", self.altitude)
            
                if msg.get_type() == 'AHRS2':
                    #self.msglock.acquire()
                    with self.msglock:
                        dic = msg.to_dict()
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
            
                        yaw
                        #print("yaw: ", yaw)
                        """
                        altitude = dic['altitude']
                        self.altitude = 3.2808 * altitude
                        #print("altitude: ", self.altitude)
                        #print("")
                    #self.msglock.release()
            
                #if msg.get_type() == 'GPS_RAW_INT':
                if msg.get_type() == 'GPS2_RAW':
                    ### This now all handled by GPS_RAW_INT msg and Here 3 gps sensor
                    with self.msglock:
                        #print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)

                        # print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)
                        dic = msg.to_dict()
                        self.lat = dic['lat']/10000000
                    
                        self.lon = dic['lon']/10000000
                        mag_data = mavextra.get_mag_field_ef(self.lat, self.lon)
                    
                        self.declination = mag_data[0]
                    
                    
                        satellites_visible = dic['satellites_visible']
                        #print("satellites_visible: ", satellites_visible)
                        fix_type = dic['fix_type']
                        self.fix_type = fix_type

                        if self.fix_type < 3:
                            self.lat = 39.932138
                            self.lon = -105.065293
                        
                        self.gnd_track = dic['cog'] / 100 #convert from 100th of degresss to degrees
                        if pix_hawk_config.MockHeading > 0:
                            self.gnd_track = pix_hawk_config.MockHeading
                        
                        self.gps_alt = dic['alt'] * 0.00328084
                        if self.fix_type < 3:
                            self.gps_alt = 5400 # at eagle rd per topo map

                        vel = int(dic['vel'])
                        vel = vel * 0.0223694  # cm/s to mph

                        my_icao = pix_hawk_config.icao
                        
                        #self.adsb_dic.updateVehicle(my_icao, "N423DS", self.lat, self.lon, self.gps_alt, 0, 0, self.gnd_track, True)

                        if self.gps_manager != None:
                            climb = 0 # not availabe in this msg
                            self.gps_manager.update_gps_listener('px', self.fix_type, self.lat, self.lon, 
                                self.gps_alt, vel, climb, self.gnd_track)
                               
                        if pix_hawk_config.MockAirPlane:
                        #if False:
                            if self.tail_count < 80:
                                off =self.tail_count*.001
                                self.tail_count += 1
                                dist = Math.latlon_distance(self.lat, self.lon, self.lat-.04+off, self.lon-.005)
                                self.adsb_dic.updateVehicle('myicao1234a', "N423DS", self.lat-.04+off, 
                                    self.lon-.005, self.gps_alt+100, 120, 10, self.gnd_track, True, dist) #self.gnd_track)
                            elif self.tail_count < 100:
                                self.tail_count += 1
                            else:
                                self.tail_count = 0

                            #Global.update_origin_ap(pix_hawk_config.icao, 'N423DS', self.lat, self.lon, self.gps_alt, 120, 0, 0)

                        """
                        self.adsb_dic.updateVehicle('myicao1234a', "LEFT", self.lat-.05, self.lon, self.gps_alt+500, 0, 0, 90, True) #self.gnd_track)
                        self.adsb_dic.updateVehicle('myicao1234b', "RIGHT", self.lat+.05, self.lon, self.gps_alt-500, 0, 0, 180, True) #self.gnd_track)
                        self.adsb_dic.updateVehicle('myicao1234c', "TOP", self.lat, self.lon+.05, self.gps_alt+1600, 0, 0, 270, True) #self.gnd_track)
                        self.adsb_dic.updateVehicle('myicao1234d', "BOT", self.lat, self.lon-.05, self.gps_alt-1600, 0, 0, 360, True) #self.gnd_track)
                        """
            
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

                    self.ATTITUDE = msg
                    if self.RAW_IMU != None:
                        #heading = mavextra.mag_heading(self.RAW_IMU,self.ATTITUDE, self.declination, self.SENSOR_OFFSETS, (0,0,0), s_factor=.97)
                        
                        #heading = mavextra.mag_heading(self.RAW_IMU,self.ATTITUDE, self.declination, s_factor=.97)

                        #self.heading = heading
                        pass

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
                self.master.close()
                self.phigetThread.put_instance()
                if self.adsb_dic != None:
                    self.adsb_dic.put_instance()
                self.put_instance()
                
                traceback.print_exc()
                
        self.master.close()
        if self.phigetThread != None:
            self.phigetThread.put_instance()
        if self.adsb_dic != None:
            self.adsb_dic.put_instance()
        print("stopped mavlinkmsg thread")
            
    def getAharsData(self, inData):
        #print("getAharsData()")
        #return inData
        if(self.msglock.acquire(blocking=False)):
        #if(False):
            newData = aharsData(self.roll, self.pitch, self.heading, self.altitude, 
                self.climb, self.groundspeed, self.airspeed, self.fix_type, self.gnd_track, self.wind_speed, 
                self.wind_dir, self.xmag, self.ymag, self.zmag, self.xacc, self.yacc, self.zacc, self.gps_alt, self.lat, self.lon,
                self.baro_press)
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

    from pix_hawk_gps_reader import GpsManager
    gmng = GpsManager()

    msgthd = mavlinkmsg(gmng)
    msgthd.start()

    try:
        
        ahdata = (-1,-1,-1,-1)

        while msgthd._run_thread:
            ahdata = msgthd.getAharsData(ahdata)
        #print('msgthd.heading ', msgthd.heading)
    
        #print("roll: ", ahdata.roll)
        #print("pitch: ", ahdata.pitch)
        #print("heading: ", ahdata.heading)
        #print("")
        
            time.sleep(.1)

        mavlinkmsg.phigetThread.put_instance()
        mavlinkmsg.put_instance()
        msgthd.join()

    except:
        mavlinkmsg.phigetThread.put_instance()
        mavlinkmsg.put_instance()
        msgthd.join()


