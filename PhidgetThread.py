from math import trunc
from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Magnetometer import *
from Phidget22.Devices.Spatial import *

import time

from threading import Thread, Lock

def onMagneticFieldChange(self, magneticField, timestamp):
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
        self.magnetometer0 = Magnetometer()
        self.accelerometer0 = Accelerometer()
        self.gyroscope0 = Gyroscope()
        self.spatial0 = Spatial()

        self.accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
        self.gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)
        self.magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)
        self.spatial0.setOnSpatialDataHandler(onSpatialData)

        self.phidgetMag = PhidgetMag(-1,-1,-1)
        self.msglock = Lock()

    def run(self):
        #return
        print("started PhidgetThread thread")
        self.accelerometer0.openWaitForAttachment(5000)
        self.gyroscope0.openWaitForAttachment(5000)
        self.magnetometer0.openWaitForAttachment(5000)
        self.spatial0.openWaitForAttachment(5000)

        while PhidgetThread._run_thread:
            time.sleep(1)
            

        self.accelerometer0.close()
        self.gyroscope0.close()
        self.magnetometer0.close()
        self.spatial0.close()
        print("ended PhidgetThread thread")

    def setMagFeild(self, field):
        with self.msglock:
            #print('called setMagFeild')
            #print("MagneticField: \t"+ str(field[0])+ "  |  "+ str(field[1])+ "  |  "+ str(field[2]))
            self.phidgetMag = PhidgetMag(field[0], field[1], field[2])
    
    def getMagFeild(self):
        with self.msglock:
            field = self.phidgetMag
        
        return field
            
    
    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            cls._instance = PhidgetThread()
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
        
    

if __name__ == '__main__':
    pdthd = PhidgetThread.get_instance()
    count = 0
    while count < 30:
        field = pdthd.getMagFeild()
        print('magx {0}, magy {1}, magz {2}'.format(field.xmag, field.ymag, field.zmag))
        time.sleep(.1)
        count += 1
    
    pdthd.put_instance()



    