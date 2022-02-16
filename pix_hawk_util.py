
from itertools import count
import re
from threading import Thread, Lock, Timer
import threading
import time
import termios
import sys
import select
import tty
from math import sin, cos, radians, fmod, floor
import numpy as np
import math
import pix_hawk_config

class DebugPrint():

    lock = Lock()
    
    @staticmethod
    def print(*args):
        if not pix_hawk_config.DEBUG:
            return
        with DebugPrint.lock:
            arg_str = ','.join(map(str,args))
            print(arg_str)


class FunTimer():

    def __init__(self, enable=True):
       self.timer_list = []
       self.dict  = {}
       self.fun_count = 0
       self.enable = enable

    def start(self, name):
        if not self.enable:
            return
        if not name in self.dict:
                cur_time = time.time()
                cum_time = 0
                self.timer_list.append([cur_time,cum_time])
                self.dict[name] = self.fun_count
                self.fun_count += 1
        idx = self.dict[name]
        self.timer_list[idx][0] = time.time()

    def stop(self, name):
        if not self.enable:
            return
        idx = self.dict[name]
        cur_time = self.timer_list[idx][0]
        time_dif = time.time() - cur_time
        self.timer_list[idx][1] += time_dif

    def close(self):
        if not self.enable:
            return
        cum_time = 0
        for itm in self.timer_list:
            cum_time += itm[1]

        for key in self.dict:
            idx = self.dict[key]
            tot_time = self.timer_list[idx][1]
            percent_time = tot_time/cum_time
            print('fun name {0} percent time {1} tot time {2}'.format(key, percent_time, tot_time))

class OriginAircraft():
    def __init__(self, ICAO_address, callsign, lat, lon, alt, speed, climb, heading):
       
        self.ICAO_address = ICAO_address 
        self.callsign = callsign
        self.lat = lat
        self.lon = lon
        self.altitude = alt
        self.speed = speed  
        self.climb = climb
        self.heading = heading

        self.time_out_interval = 5
        self.timeout_time = time.time() + self.time_out_interval
        self.timeout_thread = Thread(target = self.timeout_target)
        self.timeout_thread.start()
        
        self.is_timed_out = False

    def timeout_target(self):
        print('timeout_target thread started')
        while True:
            if time.time() < self.timeout_time:
                time.sleep(self.timeout_time - time.time())
            else:
                self.is_timed_out = True
                self._origin_ap = None
                print('timeout_target thread stopped')  
            return

    def set_timeout(self):
        self.timeout_time = time.time() + self.time_out_interval


class Global():
    _baro_climb = 0
    _lock = Lock()
    _alt_mode_gps = True
    _origin_ap = None

    @classmethod
    def set_baro_climb(cls, climb):
        with cls._lock:
            cls._baro_climb = climb

    @classmethod
    def get_baro_climb(cls):
        with cls._lock:
            return cls._baro_climb

    @classmethod
    def set_alt_mode_gps(cls, alt_mode):
        with cls._lock:
            cls._alt_mode_gps = alt_mode

    @classmethod
    def get_alt_mode_gps(cls):
        with cls._lock:
            return cls._alt_mode_gps

    @classmethod
    
    def update_origin_ap(cls, icao, callsign, lat, lon, adsb_altitude, hor_velocity, ver_velocity, adsb_heading):
        print('++++++++++++ Origin Aircraft Updated +++++++++++++++++++++++')
        with cls._lock:
            if cls._origin_ap == None:
                cls._origin_ap = OriginAircraft(icao, callsign, lat, lon, adsb_altitude, 
                        hor_velocity, ver_velocity, adsb_heading)
            else:
                cls._origin_ap.icao = icao
                cls._origin_ap.callsign = callsign
                cls._origin_ap.lat = lat
                cls._origin_ap.lon = lon
                cls._origin_ap.altitude = adsb_altitude
                cls._origin_ap.h_speed = hor_velocity
                cls._origin_ap.v_speed = ver_velocity
                cls._origin_ap.heading = adsb_heading
                cls._origin_ap.set_timeout()
                
    @classmethod
    def get_origin_ap(cls):
        with cls._lock:
            return cls._origin_ap




class Math():

    @classmethod
    def get_bearing(cls, lat1, long1, lat2, long2, track=0):
        #if(abs(lat1 - lat2) < .00001 or abs(long1 - long2) < .00001):
            #return -1
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = np.arctan2(x,y)
        brng = np.degrees(brng)

        brng = (brng + 360) % 360
        
        brng -= track
        if brng < 0:
            brng = 360+brng

        return brng


    @classmethod
    def rotate_point(cls, point, angle, center_point=(0, 0)):
        """Rotates a point around center_point(origin by default)
        Angle is in degrees.
        Rotation is counter-clockwise
        """
        #print('angle ', angle)
        #angle_rad = radians(angle % 360)
        angle_rad = radians(angle)
        #print('angle_rad ', angle_rad)
        # Shift the point so that center_point becomes the origin
        new_point = (point[0] - center_point[0], point[1] - center_point[1])
        new_point = (new_point[0] * cos(angle_rad) - new_point[1] * sin(angle_rad),
                    new_point[0] * sin(angle_rad) + new_point[1] * cos(angle_rad))
        # Reverse the shifting we have done
        new_point = (new_point[0] + center_point[0], new_point[1] + center_point[1])
        return new_point
    
    @classmethod
    def rotate_line(cls, point1, point2, angle, center_point=(0, 0)):
        """Rotates a line around center_point(origin by default)
        Angle is in degrees.
        Rotation is counter-clockwise
        """
        r_point1 = cls.rotate_point(point1, angle, center_point)
        r_point2 = cls.rotate_point(point2, angle, center_point)
        return (r_point1, r_point2)

    @classmethod
    def cart2pol(cls, x, y):
        
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(x, y)
        ang = np.degrees(phi)
        
        return(rho, ang)

    @classmethod
    def pol2cart(cla, rho, phi, invert=False):
        
        #phi = phi - rotate
        #if phi < 0:
        #    phi = 360 + phi
        phi = phi % 360
        
        phi = np.radians(phi)
        if invert:
            x = rho * np.sin(phi)
            y = rho * np.cos(phi)
        else:
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
        
        return(x, y)

    @classmethod
    def angle_sum_right(a1, a2):
        #print('get_sum_right', a1, a2)
        sum = a1+a2
        if sum < 360:
            return sum
        else:
            return sum - 360

    @classmethod
    def angle_dif_right(a1, a2):
        if a2 > a1:
            return a2 - a1
        else:
            to360 = 360 - a1
            return to360 + a2

    @classmethod
    def latlon_distance(self, lat1, lon1, lat2, lon2):
        origin = (lat1,lon1)
        destination = (lat2,lon2)
        radius = 6371
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
            math.sin(dlon / 2) * math.sin(dlon / 2))

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        d = (radius * c) / 1.609344 # convet to miles

        return d

    @classmethod
    def br2clock(self, bear):
        clk_degrees = 360 / 12
        bear = bear/clk_degrees
        if bear < .5:
            return 12
        clk = Math.round_half_up(bear)
        return clk

    @classmethod
    def round_half_up(self, n, decimals=0):
        multiplier = 10 ** decimals
        if decimals == 0:
            return int(floor(n*multiplier + 0.5) / multiplier)
        else:
            return (floor(n*multiplier + 0.5) / multiplier)


class KeyBoard(Thread):
    _instance = None
    _instance_count = 0
    _run_thread = True
    def __init__(self):
        Thread.__init__(self)
    
        self.key_buf = None
        self.keylock = Lock()
        self.time_out = False
    
    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            cls._instance = KeyBoard()
            cls._instance.start()

        cls._instance_count += 1    
        return cls._instance

    def run(self):

        print("started KeyBoard thread")

        while KeyBoard._run_thread:
        
            #key = input()
            #key = getch.getche()
            key = self.getch()
            #print('keyboard got key ', key)
            self.set_key(key)
        print('end kbd thread')

    @classmethod
    def stop(cls):
        cls._instance_count -= 1
        if cls._instance_count == 0:
            cls._run_thread = False
            cls._instance = None
        
    def get_key(self):
        with self.keylock:
            key = self.key_buf
            self.key_buf = None
            return key
    
    def wait_key(self, timeout = 10):
        self.time_out = False
        timer = None
        if timeout > 0:
            timer = Timer(timeout, self.end_fun)
            timer.start()
        
        #key = None
        while self.time_out == False:
            print('.', end='', flush=True)
            key = self.get_key()
            if key != None:
                if timer!= None:
                    timer.cancel()
                return key
            time.sleep(.05)
        return None
    
    # call by timer
    def end_fun(self):
        self.time_out = True

    def set_key(self, key):
        with self.keylock:
            self.key_buf = key

    def getch(self):
        
        def _getch():
            ch = None
            fd = sys.stdin.fileno()    
            old_settings = termios.tcgetattr(fd)
            try:
                #tty.setraw(fd)
                tty.setcbreak(fd)
                if select.select([sys.stdin,],[],[], 0.5)[0]:
                    #print('sys.stdin.read(1)')
                    ch = sys.stdin.read(1)
                else:
                    ch = None
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                #termios.tcsetattr(fd, termios.TCSANOW, old_settings)
            #sys.stdout.write(ch)   
            #print(ch)
            return ch
            
        return _getch()




if __name__ == '__main__':

    clk = Math.br2clock(13)
    print(clk)

    xy = Math.pol2cart(10, 0)
    print (xy)
    """for i in range(20):
        br = Math.get_bearing(40, -105, 39+i/10, -104)
        print(br)
        dist = Math.latlon_distance(40, -105, 39+i/10, -104)
        xy = Math.pol2cart(dist, br, rotate=90)
        #print(xy)"""

    """br = Math.get_bearing(40, -105, 40, -104)
    print(br)
    dist = Math.latlon_distance(40, -105, 40, -104)
    xy = Math.pol2cart(dist, br, rotate=90)
    print(xy)

    br = Math.get_bearing(40, -105, 40, -106)
    print(br)
    dist = Math.latlon_distance(40, -105, 40, -106)
    xy = Math.pol2cart(dist, br, rotate=90)
    print(xy)

    br = Math.get_bearing(40, -105, 39, -105)
    print(br)
    dist = Math.latlon_distance(40, -105, 39, -105)
    xy = Math.pol2cart(dist, br, rotate=90)
    print(xy)

    br = Math.get_bearing(40, -105, 41, -105)
    print(br)
    dist = Math.latlon_distance(40, -105, 41, -105)
    xy = Math.pol2cart(dist, br, rotate=90)
    print(xy)"""


    #ft = FunTimer(enable=False)
    """ft = FunTimer()

    for i in range(2):

        ft.start('fun_one')
        time.sleep(1.5)
        ft.stop ('fun_one')

        ft.start('fun_two')
        time.sleep(.75)
        ft.stop ('fun_two')

    ft.close()"""
    """kbd = KeyBoard.get_instance()
    
    key = kbd.wait_key(timeout=30)
    print('wait kye ', key)
    i= 0
    while i in range(0, 20):
        key = kbd.get_key()
        if key != None:
            print(key)
        time.sleep(.1)
        i += 1
    KeyBoard.stop()"""

    
    """while True:
        key = kbd.get_key()
        if key != None:
            print('got key: ', key)
        key = kbd.wait_key(timeout=3)
        if key == None:
            print('\nwait key time out')
        else:
            print('\ngot wait key: ', key)
        time.sleep(.1)"""
    