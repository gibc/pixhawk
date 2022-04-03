
#!/usr/bin/env python3
# Original Code: https://gist.github.com/Lauszus/5785023#file-gps-py
# Created by: Kristian Sloth Lauszus

from asyncio import sleep
from ntpath import join
from pickle import NONE
from re import M
from struct import pack
import time
from turtle import pos
import serial
from threading import Thread, Lock, Timer
import traceback
#from pix_hawk_util import Global, Math

class GpsManager():
    def __init__(self):    
        self.gps_dict = {}
        self.lock = Lock()

    def update_gps_listener(self, type, fix, lat, lon, altitude, speed, climb, track):
        
        with self.lock:
            if not type in self.gps_dict:
                self.gps_dict[type] = GpsListener(type, fix, lat, lon, altitude, speed, climb, track, self)
                i = 5
            else:
                self.gps_dict[type].type = type
                self.gps_dict[type].fix = fix
                self.gps_dict[type].lat = lat
                self.gps_dict[type].lon = lon
                self.gps_dict[type].altitude = altitude
                self.gps_dict[type].speed = speed
                self.gps_dict[type].climb = climb
                self.gps_dict[type].track = track
                self.gps_dict[type].set_timeout()

    #----- Public Methods For Dependency Injection-----
    def get_listener(self):
        with self.lock:
            if len(self.gps_dict) == 0:
                return None

            if 'hg' in self.gps_dict:
                if int(self.gps_dict['hg'].fix) >= 3:
                    return self.gps_dict['hg']

            if 'dg' in self.gps_dict:
                if int(self.gps_dict['dg'].fix) >= 1:
                    return self.gps_dict['dg']

            if 'px' in self.gps_dict: 
                if int(self.gps_dict['px'].fix) >= 3:
                    return self.gps_dict['px']

            if 'sb' in self.gps_dict: 
                return self.gps_dict['sb']

            else:
                return None
            
            

    def get_listener_cnt(self):
        with self.lock:
            return len(self.gps_dict)
                
    def delete_listener(self, type):
        with self.lock:
            del self.gps_dict[type]


class GpsListener():
    def __init__(self, type, fix, lat, lon, alt, speed, climb, track, gps_manager = None):

        self.gps_manager = gps_manager

        self.type = type
        self.fix = fix
        self.lat = lat
        self.lon = lon
        self.altitude = alt
        self.speed = speed  
        self.climb = climb
        self.track = track
        
        self.time_out_interval = 5
        self.timeout_time = time.time() + self.time_out_interval
        self.timeout_thread = Thread(target = self.timeout_target)
        self.timeout_thread.start()
        
        self.is_timed_out = False
        self.set_timeout()
        
        
    def timeout_target(self):
        print('timeout_target thread started')
        while True:
            if time.time() < self.timeout_time:
                time.sleep(self.timeout_time - time.time())
            else:
                self.is_timed_out = True
                #from pix_hawk_util import Global
                #Global.set_gps_listener(None)
                self.gps_manager.delete_listener(self.type)
                print('timeout_target thread stopped')  
            return

    def set_timeout(self):
        self.timeout_time = time.time() + self.time_out_interval

class GpsThread():

    def __init__(self, path, gps_manager):
        self.path = path
        self.gps_manager = gps_manager
        self.lat = None
        self.lon = None
        self.fix = None
        self.speed = None
        self.track = None
        self.altitude = None
        self.climb = 0
        self.data_complete = False
        self.ser = None
       
        self.run_thread = True
        #self.gps_td = GpsThread('/dev/ttyACM2')
        self.gps_thread = Thread(target = self.thread_fun)
        #self.gps_thread.start()

    def connect(self):
        try:
            self.ser = serial.Serial(self.path, 9600, timeout=1)
            return True
        except:
            return False

    def start(self):

        self.gps_parser = self
        self.gps_thread.start()

    def readString(self):
        while 1:
            while self.ser.read().decode("utf-8") != '$':  # Wait for the begging of the string
                pass  # Do nothing
            line = self.ser.readline().decode("utf-8")  # Read the entire string
            return line


    def getTime(self,string, format, returnFormat):
        return time.strftime(returnFormat,
                            time.strptime(string, format))  # Convert date and time to a nice printable format

    def mins2degs(self, pos_str):
        i = pos_str.find('.')
        dd = pos_str[0:i-2]
        mm = pos_str[i-2:]
        mm = float(mm)
        mm = mm/60.0
        cor = int(dd) + mm
        return cor

    def getLatLng(self,latString, lngString):
        
        lat = self.mins2degs(latString)
        lng = self.mins2degs(lngString)

        return lat, lng


    def printRMC(self,lines):
        #print("========================================RMC========================================")
        # print(lines, '\n')
        #print("Fix taken at:", self.getTime(lines[1] + lines[9], "%H%M%S.%f%d%m%y", "%a %b %d %H:%M:%S %Y"), "UTC")
        #print("Status (A=OK,V=KO):", lines[2])
        if len(lines[3]) == 0 or len(lines[5]) == 0:
            return False

        latlng = self.getLatLng(lines[3], lines[5])
        self.lat = float(latlng[0])
        self.lon = float(latlng[1])
        if lines[6] == 'W':
            self.lon = -self.lon
        #print("Lat,Long: ", latlng[0], lines[4], ", ", latlng[1], lines[6], sep='')
        #print("Speed (knots):", lines[7])
        speed = float(lines[7])
        speed = speed * 1.150779 # knots to mph
        from pix_hawk_util import Math
        self.speed = Math.round_half_up( speed, 1)
        #print('speed (mph): ', self.speed)

        #print("Track angle (deg):", lines[8])
        if len(lines[8]) == 0:
            self.track = 0
        else:
            self.track = float(lines[8])

        #print("Magnetic variation: ", lines[10], end='')
        #if len(
        #        lines) == 13:  # The returned string will be either 12 or 13 - it will return 13 if NMEA standard used is above 2.3
        #    print(lines[11])
        #    print("Mode (A=Autonomous, D=Differential, E=Estimated, N=Data not valid):", lines[12].partition("*")[0])
        #else:
        #    print(lines[11].partition("*")[0])

        return True


    def printGGA(self,lines):
        #print("========================================GGA========================================")
        # print(lines, '\n')
        #print("Fix taken at:", self.getTime(lines[1], "%H%M%S.%f", "%H:%M:%S"), "UTC")
        if len(lines[2]) == 0 or len(lines[4]) == 0:
            return False

        latlng = self.getLatLng(lines[2], lines[4])
        self.lat = float(latlng[0])
        self.lon = float(latlng[1])
        if lines[5] == 'W':
            self.lon = -self.lon

        #print("Lat,Long: ", latlng[0], lines[3], ", ", latlng[1], lines[5], sep='')
        #print("Fix quality (0 = invalid, 1 = fix, 2..8):", lines[6])
        self.fix = lines[6]
        #print("Fix ", self.fix)

        #print("Satellites:", lines[7].lstrip("0"))
        #print("Horizontal dilution:", lines[8])
        #print("Altitude: ", lines[9], lines[10], sep="")
        alt = float(lines[9])
        alt = alt * .00328
        alt = alt * 1000
        self.altitude = int(alt)
        #print("Altitude: ", alt)
        #print("Height of geoid: ", lines[11], lines[12], sep="")
        #print("Time in seconds since last DGPS update:", lines[13])
        #print("DGPS station ID number:", lines[14].partition("*")[0])
        return True


    def printGSA(self,lines): #vt-162 does not generate
        #print("========================================GSA========================================")
        # print(lines, '\n')

        if len(lines[2]) == 0 or len(lines[4]) == 0:
            return False

        print("3D fix (1=No fix,2=2D fix, 3=3D fix):", lines[2])
        print("PRNs of satellites used for fix:", end='')
        for i in range(0, 12):
            prn = lines[3 + i].lstrip("0")
            if prn:
                print(" ", prn, end='')
        print("\nPDOP", lines[15])
        print("HDOP", lines[16])
        print("VDOP", lines[17].partition("*")[0])
        return


    def printGSV(self,lines): #vt-162 does not generate
        if lines[2] == '1':  # First sentence
            print("========================================GSV========================================")
        else:
            print("===================================================================================")
        # print(lines, '\n')

        print("Number of sentences:", lines[1])
        print("Sentence:", lines[2])
        print("Satellites in view:", lines[3].lstrip("0"))
        for i in range(0, int(len(lines) / 4) - 1):
            print("Satellite PRN:", lines[4 + i * 4].lstrip("0"))
            print("Elevation (deg):", lines[5 + i * 4].lstrip("0"))
            print("Azimuth (deg):", lines[6 + i * 4].lstrip("0"))
            print("SNR (higher is better):", lines[7 + i * 4].partition("*")[0])
        return


    def printGLL(self,lines):
        #print("========================================GLL========================================")
        # print(lines, '\n')
        if len(lines[1]) == 0 or len(lines[3]) == 0:
            return False

        latlng = self.getLatLng(lines[1], lines[3])
        self.lat = float(latlng[0])
        self.lon = float(latlng[1])
        if lines[4] == 'W':
            self.lon = -self.lon
        #print("Lat,Long: ", latlng[0], lines[2], ", ", latlng[1], lines[4], sep='')
        #print("Fix taken at:", self.getTime(lines[5], "%H%M%S.%f", "%H:%M:%S"), "UTC")
        #print("Status (A=OK,V=KO):", lines[6])
        #if lines[7].partition("*")[0]:  # Extra field since NMEA standard 2.3
        #    print("Mode (A=Autonomous, D=Differential, E=Estimated, N=Data not valid):", lines[7].partition("*")[0])
        return True


    def printVTG(self,lines):

        #if len(lines[1]) == 0 or len(lines[5]) == 0:
        #    return False

        #print("========================================VTG========================================")
        # print(lines, '\n')

        #print("True Track made good (deg):", lines[1], lines[2])
        if len(lines[1]) == 0:
            self.track = 0
        else:
            self.track = float(lines[1])

        #print("Magnetic track made good (deg):", lines[3], lines[4])
        #print("Ground speed (knots):", lines[5], lines[6])
        if len(lines[5]) == 0:
            self.speed = 0
        else:
            speed = float(lines[5])
            speed = speed * 1.150779 # knots to mph
            from pix_hawk_util import Math
            self.speed = Math.round_half_up( speed, 1)
        #print('speed (mph): ', self.speed)

        #print("Ground speed (km/h):", lines[7], lines[8].partition("*")[0])
        #if lines[9].partition("*")[0]:  # Extra field since NMEA standard 2.3
        #    print("Mode (A=Autonomous, D=Differential, E=Estimated, N=Data not valid):", lines[9].partition("*")[0])
        return True


    def checksum(self,line):
        checkString = line.partition("*")
        checksum = 0
        for c in checkString[0]:
            checksum ^= ord(c)

        try:  # Just to make sure
            inputChecksum = int(checkString[2].rstrip(), 16);
        except:
            print("Error in string")
            return False

        if checksum == inputChecksum:
            return True
        else:
            print("=====================================================================================")
            print("===================================Checksum error!===================================")
            print("=====================================================================================")
            print(hex(checksum), "!=", hex(inputChecksum))
            return False

    def close(self):
        self.run_thread = False


    def check_complete(self):
        self.data_complete = (self.fix != None and self.lat != None and self.lon != None and 
            self.altitude != None and self.speed != None and self.climb != None and self.track != None )
        


    def thread_fun(self):
        #gps_td = GpsThread('/dev/ttyACM2')
        #from pix_hawk_util import Global
        try:
            print('gps thread started')
            
            while self.run_thread:
                try: 
                    if self.ser == None:
                        if not self.connect():
                            print('gps thread try connect')
                            time.sleep(1)
                            continue

                    if self.ser.is_open == False:
                        if not self.connect():
                            print('gps thread try connect')
                            time.sleep(1)
                            continue
                    try:
                        line = self.gps_parser.readString()
                    except:
                        print('gps thread read exception')
                        self.ser.close()
                        continue

                    lines = line.split(",")
                    if self.gps_parser.checksum(line):
                        if lines[0] == "GPRMC":
                            if not self.gps_parser.printRMC(lines):
                                #print('GPRMC error')
                                continue
                            
                        elif lines[0] == "GPGGA":
                            if not self.gps_parser.printGGA(lines):
                                #print('GPGGA error')
                                continue
                            
                        elif lines[0] == "GPGSA":
                            # printGSA(lines)
                            pass
                        elif lines[0] == "GPGSV":
                            # printGSV(lines)
                            pass
                        elif lines[0] == "GPGLL":
                            if not self.gps_parser.printGLL(lines):
                                #print('GPGLL error')
                                continue
                            
                        elif lines[0] == "GPVTG":
                            if not self.gps_parser.printVTG(lines):
                                #print('GPVTG error')
                                continue
                            
                        elif lines[0] == "GPTXT":
                            print('GPTXT message')
                            print('severtiy:', lines[3])
                            print('message:', lines[4])
                            #print('GPTXT discarded')
                            continue
                        
                        elif line[0 == 'GPGSV']:
                            continue
                        elif line[0 == 'GPGSV']:
                            continue
                        else:
                            print("\n\nUnknown type:", lines[0], "\n\n")
                            continue
                except:
                    print('gps parser exception')
                    traceback.print_exc()
                    continue

                """if self.data_complete:
                    print('fix {0}, lat {1}, lon {2}, alt {3}, speed {4}, track {5}'.
                        format(self.fix, self.lat, self.lon, self.altitude, self.speed, self.track))"""
                
                if self.data_complete and int(self.fix) >= 1:
                    self.gps_manager.update_gps_listener('dg', self.fix, self.lat, self.lon, self.altitude, self.speed, self.climb, self.track)
                    #Global.update_gps_listener('dg', self.fix, self.lat, self.lon, self.altitude, self.speed, self.climb, self.track)
                else:
                    self.check_complete()
                    if not self.data_complete:
                        print("******** gps dongle reader error *********")
                        time.sleep(3)

            print('gps thread end')

        except: 
            print('gps thread exception')
            traceback.print_exc()
            self.run_thread = False

import gpsd


class HgGpsThread():

    def __init__(self,gps_manager):
        self.gps_manager = gps_manager
        self.lat = None
        self.lon = None
        self.fix = None
        self.speed = None
        self.track = None
        self.altitude = None
        self.climb = 0
        self.data_complete = False
        self.ser = None
       
        self.run_thread = True
        self.gps_thread = Thread(target = self.thread_fun)

    def close(self):
        self.run_thread = False
        join(self.gps_thread)

    def thread_fun(self):
        
        try:
            print('HG gps thread started')

            gpsd.connect()   
            
            while self.run_thread:
                packet = gpsd.get_current()
                if packet.mode < 3:
                    time.sleep(.5)
                    continue

                print(packet.position())
                self.fix = packet.mode
                self.lat = packet.lat
                self.lon = packet.lon
                self.speed = packet.hspeed * 2.23694 # m / sec to mph
                self.altitude = packet.alt * 3.28084 # meters to feet
                self.climb = packet.climb * 196.85 # m / sec to feet per min
                self.track = packet.track

                self.gps_manager.update_gps_listener('hg', self.fix, self.lat, self.lon, self.altitude, self.speed, self.climb, self.track)
                time.sleep(.5)

            print('HG gps thread stopped')

        except:
            print('HG gps thread exception')
            traceback.print_exc()
            self.run_thread = False


if __name__ == '__main__':
    #ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1)  # Open Serial port
    gps_mng = GpsManager()
    #gps_td = GpsThread('/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00', gps_mng)
    gps_td = HgGpsThread(gps_mng)
      
    #gps_td.start()
    gps_td.gps_thread.start()
    
    