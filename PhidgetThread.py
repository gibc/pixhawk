#from _typeshed import Self
from math import trunc
from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Magnetometer import *
from Phidget22.Devices.Spatial import *

import time
import math

from threading import Thread, Lock

def onMagneticFieldChange(self, magneticField, timestamp):
    if PhidgetThread._instance != None:
        PhidgetThread._instance.setMagFeild(magneticField)
    #print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
    #print("Timestamp: " + str(timestamp))
    #print("----------")

def onAccelerationChange(self, acceleration, timestamp):
    pass
	#print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	#print("Timestamp: " + str(timestamp))
	#print("----------")

def onAngularRateUpdate(self, angularRate, timestamp):
    pass
	#print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	#print("Timestamp: " + str(timestamp))
	#print("----------")

def onSpatialData(self, acceleration, angularRate, magneticField, timestamp):
    pass
	#print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	#print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	#print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
	#print("Timestamp: " + str(timestamp))
	#print("----------")

def onAlgorithmData(self, quaternion, timestamp):
    #print("Quaternion: " + str(quaternion))
    #print("Timestamp " + str(timestamp))
    ea = euler_from_quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    #print("roll " + str(ea[0]))
    #print("pith " + str(ea[1]))
    #print("yaw " + str(ea[2]))
    #print('roll {0} pitch {1} yaw {2}'.format(ea[0], ea[1], ea[2]))

def euler_from_quaternion(x, y, z, w):
    #print('call euler_from_quaternion')
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    global roll_x
    global pitch_y
    global yaw_z
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    #setyaw(yaw)
    if PhidgetThread._instance != None:
        PhidgetThread._instance.set_yaw(math.degrees(yaw))
        PhidgetThread._instance.set_pitch(math.degrees(pitch_y))
        PhidgetThread._instance.set_roll(math.degrees(roll_x))
    #print('yaw_z1', 
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw) # in radians
 

class PhidgetMag:

    def __init__(self, xmag, ymag, zmag):
        self.xmag = xmag
        self.ymag = ymag
        self.zmag = zmag




class PhidgetThread (Thread):
    _instance = None
    _instance_count = 0
    _run_thread = True
    

    def __init__(self):
        Thread.__init__(self)
        PhidgetThread._instance = self
        """self.magnetometer0 = Magnetometer()
        self.accelerometer0 = Accelerometer()
        self.gyroscope0 = Gyroscope()"""
        self.run_thread = True
        self.spatial0 = Spatial()

        """self.accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
        self.gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)
        self.magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)
        self.spatial0.setOnSpatialDataHandler(onSpatialData)"""
        self.spatial0.setOnAlgorithmDataHandler(onAlgorithmData)

        """self.accelerometer0.openWaitForAttachment(1000)
        self.gyroscope0.openWaitForAttachment(1000)
        self.magnetometer0.openWaitForAttachment(1000)"""
        self.spatial0.openWaitForAttachment(1500)

        self.phidgetMag = PhidgetMag(-1,-1,-1)
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.msglock = Lock()
    
    def run(self):
        #return
        print("started PhidgetThread thread")
        try:
            #self.accelerometer0.openWaitForAttachment(5000)
            #self.gyroscope0.openWaitForAttachment(5000)
            #self.magnetometer0.openWaitForAttachment(5000)
            #self.spatial0.openWaitForAttachment(5000)

            while self.run_thread:
                time.sleep(.5)

        except:
            PhidgetThread.put_instance()
            

        """self.accelerometer0.close()
        self.gyroscope0.close()
        self.magnetometer0.close()"""
        self.spatial0.close()
        print("ended PhidgetThread thread")

    def close(self):
        self.run_thread = False
        self.join()

    def set_yaw(self, yaw):
        #print('yaw ', yaw)
        if yaw < 0:
            yaw = 360 + yaw
        yaw += 7
        if yaw > 360:
            yaw = yaw - 360
        with self.msglock:
            self.yaw = yaw

    def get_yaw(self):
        with self.msglock: 
            yaw = self.yaw
        return yaw

    def set_pitch(self, pitch):
        with self.msglock:
            self.pitch = pitch

    def get_pitch(self):
        with self.msglock: 
            pitch = self.pitch
        return pitch

    def set_roll(self, roll):
        with self.msglock:
            self.roll = roll

    def get_roll(self):
        with self.msglock: 
            roll = self.roll
        return roll

    
    def setMagFeild(self, field):
        with self.msglock:
            #print('called setMagFeild')
            #print("MagneticField: \t"+ str(field[0])+ "  |  "+ str(field[1])+ "  |  "+ str(field[2]))
            self.phidgetMag = PhidgetMag(field[0], field[1], field[2])
    
    def getMagFeild(self):
        with self.msglock:
            field = self.phidgetMag
        
        #print('called getMagFeild')
        #print("MagneticField: \t"+ str(field.xmag) + "  |  " + str(field.ymag)+ "  |  "+ str(field.zmag))
        return field
            
    
    @classmethod
    def get_instance(cls):
        try:
            if cls._instance == None:
                cls._instance = PhidgetThread()
                cls._run_thread = True
                cls._instance.start()
            cls._instance_count += 1
            return cls._instance
        except:
            cls.put_instance()
            return None
    
    @classmethod
    def put_instance(cls):
        cls._instance_count -= 1
        if cls._instance_count <= 0:
            cls._instance_count = 0
            cls._run_thread = False
            cls._instance = None
        
    

if __name__ == '__main__':
    pdthd = PhidgetThread()        
    pdthd.start()
    time.sleep(20)
    pdthd.close()

    """pdthd = PhidgetThread.get_instance()
    count = 0
    while count < 30:
        yaw = pdthd.get_yaw()
        print('eulor yaw: ', yaw)
        
        field = pdthd.getMagFeild()
        print('magx {0}, magy {1}, magz {2}'.format(field.xmag, field.ymag, field.zmag))
        h = math.atan2(field.xmag, field.ymag)
        h = math.degrees(h)
        if h < 0:
            h = 360 + h

        h += 7
        if h > 360:
            h = h - 360

        print('heading ', h)
        time.sleep(.05)
        count += 1
    
    pdthd.put_instance()"""



    