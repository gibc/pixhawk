from pymavlink import mavutil
import serial
from threading import Thread, Lock
import pix_hawk_config
from pix_hawk_adsb import AdsbDict, AdsbVehicle
import traceback
from pix_hawk_gps_reader import GpsManager

class UARadio():
    def __init__(self, path, gps_manager=None):
        self.gps_manager = gps_manager
        self.gps_lsn = None
        self.path = path
        self.run_thread = True
        self.thread = Thread(target = self.target)
        self.adsb_dic = AdsbDict.get_instance()
        
    def connect(self):
        try:
            self.master = mavutil.mavlink_connection(self.path, baud=57600)
            return True
        except:
            return False

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

    def close(self):
        self.run_thread = False
        self.thread.join()
        self.master.close()
        if self.adsb_dic != None:
            self.adsb_dic.put_instance()

    def target(self):
        if not self.connect():
            print('uav radio connect failed')
            return

        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ADSB_VEHICLE, 10)

        if self.gps_manager == None:
            print('uav radio failed no gps manager')
            return False
        
        print("started uav radio thread")
        while self.run_thread:
            #time.sleep(.1)
            try:
                msg = self.master.recv_match(blocking=True)
                if not msg:
                    continue
                if msg.get_type() == 'BAD_DATA':
                    #print(msg)
                    continue
                
                #print(msg.get_type)

                if msg.get_type() == 'ADSB_VEHICLE':
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
                    if self.gps_manager == None:
                        print ('uav radio failed no gps manager')
                        return

                    self.gps_lsn = self.gps_manager.get_listener()
                    if self.gps_lsn != None:
                        gps_lat = self.gps_lsn.lat
                        gps_lon = self.gps_lsn.lon
                        gps_alt = self.gps_lsn.altitude
                        gps_track = self.gps_lsn.track
                    elif pix_hawk_config.DEBUG:
                        gps_lat = 39.932138
                        gps_lon = -105.065293
                        gps_alt = 5400
                        gps_track = 0
                    else:
                        gps_lat = 40.0086
                        gps_lon = -105.0492
                        gps_alt = 5400
                        gps_track = 0
                    
                        #print ('uav radio failed no gps fix')
                        #return

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
                    print("pix adsb_altitude: ", adsb_altitude)
                    ICAO_address = str(dic['ICAO_address'])
                    print("pix ICAO_address: ", ICAO_address)
                    #Don't create new adbs vehical if recieved from adasb out transmitter
                    #    instead, create synthetic vehical from gps msg data

                    if ICAO_address != pix_hawk_config.icao and callsign != pix_hawk_config.callsign:
                        if cord_valid and heading_valid and call_sign_valid:

                            dist = self.adsb_dic.vehicleInLimits(gps_lat, gps_lon, gps_alt, lat, lon, adsb_altitude)
                            if dist > 0:
                                #if pix_hawk_config.Use1090Radio:
                                #print("pix vehicale within disaply limits")
                                self.adsb_dic.updateVehicle('pix', ICAO_address, callsign, lat, lon, 
                                    adsb_altitude, hor_velocity, ver_velocity, adsb_heading, True, dist)
                    else:
                        if self.gps_manager != None:
                            self.gps_manager.update_gps_listener('sb', 3, lat, lon, 
                                adsb_altitude, hor_velocity, ver_velocity, adsb_heading)

            except:
                print('uav thread exception')
                traceback.print_exc()
                self.close()

        print('uav radio thread stop')

if __name__ == '__main__':
    gps_mng = GpsManager()
    uav_radio = UARadio('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DT04LJG6-if00-port0', gps_manager=gps_mng)
    uav_radio.thread.start()
                