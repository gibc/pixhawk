
import re
from threading import Thread, Lock, Timer
import time
import termios
import sys
import select
import tty
from math import sin, cos, radians, fmod
import numpy as np
import math

class Math():

    @classmethod
    def get_bearing(cls, lat1, long1, lat2, long2):
        #if(abs(lat1 - lat2) < .00001 or abs(long1 - long2) < .00001):
            #return -1
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = np.arctan2(x,y)
        brng = np.degrees(brng)

        brng = (brng + 360) % 360
        #brng -=90
        #if brng < 0:
        #    brng = 360 - brng

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
    def pol2cart(cla, rho, phi):
        
        phi = np.radians(phi)
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
    kbd = KeyBoard.get_instance()
    
    key = kbd.wait_key(timeout=30)
    print('wait kye ', key)
    i= 0
    while i in range(0, 20):
        key = kbd.get_key()
        if key != None:
            print(key)
        time.sleep(.1)
        i += 1
    KeyBoard.stop()

    
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
    