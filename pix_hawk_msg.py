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
    def __init__(self, roll, pitch, heading, altitude):
        self.roll = roll
        self.pitch = pitch
        self.heading = heading
        self.altitude = altitude

class mavlinkmsg (Thread):
    def __init__(self):
        Thread.__init__(self)
        self.roll = 0
        self.pitch = 0
        self.heading = 0
        self.altitude = 0
        self.msglock = Lock()
        
        self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00', baud=19200)
        #self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_48003D001851303139323937-if00')

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_WIND, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS3, -1)
        #self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, -1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
    
    def run(self):
        #return
        print("started mavlinkmsg thread")
        while True:
            #time.sleep(.1)
            try:
                msg = self.master.recv_match(blocking=True)
                if not msg:
                    continue
                if msg.get_type() == 'BAD_DATA':
                    continue
                #print(msg.get_type())
                #print("Message: %s" % msg)
                if msg.get_type() == 'WIND':
                    #print("\n\n*****Got message: %s*****" % msg.get_type())
                    #print("Message: %s" % msg)
                    pass
                
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
                    with self.msglock:
                        #print("\n\n*****Got message: %s*****" % msg.get_type())
                        #print("Message: %s" % msg)
                        dic = msg.to_dict()
                        self.heading = dic['heading']
                        #print("heading: ", self.heading)
                        alt = dic['alt']
                        #print("alt: ", alt*3.28084)
                        airspeed = dic['airspeed']*2.237 
                        #print("airspeed: ", airspeed)#m/s
                        climb = dic['climb']*2.237 
                        #print('climb: ', climb) #climb rate m/s
                        groundspeed = dic['groundspeed']*2.237 
                        #print('groundspeed: ', groundspeed)
                        altitude = dic['alt']
                        self.altitude = 3.2808 * altitude
                        #print("altitude: ", self.altitude)
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
            
    def getAharsData(self, inData):
        #print("getAharsData()")
        #return inData
        if(self.msglock.acquire(blocking=False)):
        #if(False):
            newData = aharsData(self.roll, self.pitch, self.heading, self.altitude)
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