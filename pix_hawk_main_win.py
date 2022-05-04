
"""NOTE!!!!
this script started from
/etc/xdg/lxsession/LXDE-pi/autostart
xterm -e python3 /home/pi/PhidgetInsurments/pix_hawk_ui.py
"""


from cmath import pi
from socketserver import ThreadingUDPServer
from numpy import True_
import pyglet
from pyglet import clock
from pyglet.window import key
from pix_hawk_msg import mavlinkmsg, aharsData
from pix_hawk_roll_pitch import RollGague
from pix_hawk_compass_tape import CompassTape
from pix_hawk_tape import Align
from pix_hawk_tape import Orient
from pix_hawk_alt_tape import AltTapeLeft
from pix_hawk_speed_tape import SpeedTapeRight
from pix_hawk_adsb import AdsbWindow, AdsbDict
from PhidgetThread import PhidgetThread
from pix_hawk_gps import GPS_Window
from pix_hawk_wind import WindChild
from pix_hawk_aoa import Aoa
from pix_hawk_beep import Beep
import traceback
from pix_hawk_util import FunTimer
import pix_hawk_config
from pix_hawk_gps_reader import GpsThread, GpsManager, HgGpsThread
from pix_hawk_978_radio import Radio
import time
import pix_hawk_config
from pix_hawk_uav_radio import UARadio
from pix_hawk_airspeed import AirSpeed
from pix_hawk_barometer import Barometer
from pix_hawk_log import Log




class MainWindow():
    def __init__(self, width, height, full_screen = False):
        self.rdo = None
        try:
            Log.enable_log("/home/pi/PhidgetInsurments/LogFiles", enabled=True)
            self.fun_timer = FunTimer(enable=False)
            #self.fun_timer = FunTimer()
            
            if full_screen:
                self.main_window = pyglet.window.Window(fullscreen=True)
            else:
                self.main_window = pyglet.window.Window(width, height)
            self.on_draw = self.main_window.event(self.on_draw)
            self.main_window.on_close = self.on_close
            self.main_window.on_key_press = self.on_key_press
            self.fpsd = pyglet.window.FPSDisplay(window=self.main_window)

            #self.beep = Beep()
            #self.beep.start()

            self.gps_manager = GpsManager()

            #self.msg_thread = pix_hawk_msg.mavlinkmsg.get_instance()  
            """self.msg_thread = mavlinkmsg(self.gps_manager)
            self.msg_thread.start()"""
            self.msg_thread = None

            self.ahdata = aharsData(-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1)

            self.phidget_thread = None
            self.aoa_gague = None
            self.adsb_window = None

            self.compass_tape = CompassTape(self.main_window, 150, 200, 800, 65, 6, 30, align=Align.LEFT, orient=Orient.VERT)

            self.roll_gague = RollGague(self.main_window, 1.7*(self.compass_tape.border_rect.height))

            self.alt_tape = AltTapeLeft(self.main_window, self.compass_tape.border_rect.height, 150, 100, 500, 55, 6, 100, 
                    align=Align.LEFT, orient=Orient.VERT, gps_manager= self.gps_manager)

            self.speed_tape = SpeedTapeRight(self.main_window, self.compass_tape.border_rect.height, self.compass_tape.border_rect.width, 
                150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)

            
            """self.adsb_window = AdsbWindow(self.msg_thread.adsb_dic, self.main_window, self.compass_tape.border_rect.width, 
                gps_manager= self.gps_manager)"""
            self.adsb_window = AdsbWindow(AdsbDict.get_instance(), self.main_window, self.compass_tape.border_rect.width, 
                gps_manager= self.gps_manager)

            self.gps_window = GPS_Window(self.main_window, self.compass_tape.border_rect.width, self.gps_manager)

            self.wind_gague = WindChild(self.main_window, self.compass_tape.border_rect.width, 0, 0, 350, 160, 
                self.gps_manager, self.alt_tape)

            self.aoa_gague = Aoa(self.main_window, self.compass_tape.border_rect.width, 100, 200, 20, self.alt_tape)

            #self.phidget_thread = PhidgetThread.get_instance()
            self.phidget_thread = PhidgetThread()        
            self.phidget_thread.start()
            
            self.gps_td = GpsThread('/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00', self.gps_manager)
            self.gps_td.start()

            self.hg_gps_td = HgGpsThread(self.gps_manager)
            self.hg_gps_td.gps_thread.start()
                        
            time.sleep(1)
            self.rdo = Radio('/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0', self.gps_manager)
            if not self.rdo.mkpipe():
                print('make pipe failed\n')
            self.rdo.radio2frame()
            self.rdo.radio_thread.start()

            self.uav_radio = UARadio('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DT04LJG6-if00-port0', gps_manager=self.gps_manager)
            self.uav_radio.thread.start()

        
            self.barometer = Barometer('ftdi://ftdi:232h:FT4VTTQV/1')   
            self.barometer.baro_thread.start()
            
            #self.airspeed  = AirSpeed('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN00EDLI-if00-port0',self.barometer) # in plane address
            self.airspeed = AirSpeed('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN00EERE-if00-port0',self.barometer) # bench address
            self.airspeed.airspeed_thread.start() 
        
        except Exception:
            self.ex_stop()

        
        
    
    def on_draw(self):

        try:
            #self.beep.beep()
            self.main_window.clear()
            
            

            """self.ahdata = self.msg_thread.getAharsData(self.ahdata)"""

            gps_lsn = self.gps_manager.get_listener()
            if gps_lsn != None:
                gps_alt = gps_lsn.altitude
                gps_climb = gps_lsn.climb
                gps_speed = gps_lsn.speed
                gps_fix = gps_lsn.fix
                gps_track = gps_lsn.track
                gps_lat = gps_lsn.lat
                gps_lon = gps_lsn.lon
            #gps_speed = 30


            if self.phidget_thread != None:
                yaw = self.phidget_thread.get_yaw()
                pitch = self.phidget_thread.get_pitch()
                roll = self.phidget_thread.get_roll()
            else:
                yaw = 0
                pitch = 0
                roll = 0
            #yaw =360-45

            # draw all controls on top of roll gague to overaly top and bot rects
            self.fun_timer.start('roll_gague')
            #self.roll_gague.draw(self.ahdata.roll, self.ahdata.pitch)
            self.roll_gague.draw(roll, pitch)
            self.fun_timer.stop('roll_gague')

            """if self.phidget_thread != None:
                yaw = self.phidget_thread.get_yaw()
            else:
                yaw = 0"""
            
            
            self.fun_timer.start('compass_tape')
            self.compass_tape.draw(yaw)
            self.fun_timer.stop('compass_tape')

            self.fun_timer.start('alt_tape')
            #if Global.get_gps_listener() != None:
            #    gps_lsn = Global.get_gps_listener()
            #    self.alt_tape.draw(gps_lsn.altitude, gps_lsn.climb, self.ahdata.baro_press)
            #else:
            """self.alt_tape.draw(self.ahdata.altitude, self.ahdata.climb, self.ahdata.baro_press)"""
            self.alt_tape.draw(gps_alt, gps_climb, self.barometer.get_pressure(), self.barometer.get_density_alt()) # self.ahdata.baro_press)
            self.fun_timer.stop('alt_tape')

            self.fun_timer.start('speed_tape')
            """self.speed_tape.draw(self.ahdata.airspeed, self.ahdata.groundspeed)"""
            self.speed_tape.draw(self.airspeed.get_airspeed(), gps_speed, self.airspeed.get_temp(), 
                                 self.airspeed.get_true_airspeed())
            self.fun_timer.stop('speed_tape')

            self.fun_timer.start('gps_window')
            #self.gps_window.draw(self.ahdata.fix_type, self.ahdata.gnd_track, self.ahdata.groundspeed, yaw, self.ahdata.gps_alt)
            self.gps_window.draw(gps_fix, gps_track, gps_speed, yaw, gps_alt)
            self.fun_timer.stop('gps_window')

            self.fun_timer.start('wind_gague')
            #self.wind_gague.draw_calc(self.ahdata.airspeed, yaw, self.ahdata.groundspeed, self.ahdata.gnd_track, self.ahdata.altitude)
            self.wind_gague.draw_calc(self.airspeed.get_true_airspeed(), yaw, gps_speed, gps_track, gps_alt)
            self.fun_timer.stop('wind_gague')

            self.fun_timer.start('aoa_gague')
            #self.aoa_gague.draw(self.ahdata.airspeed, self.ahdata.climb, self.ahdata.pitch)
            self.aoa_gague.draw(0, gps_climb, pitch)
            self.fun_timer.stop('aoa_gague')

            self.fun_timer.start('adsb_window')
            #self.adsb_window.draw(self.ahdata.lat, self.ahdata.lon, self.ahdata.gps_alt, yaw) # display relative to heading self.ahdata.gnd_track)
            self.adsb_window.draw(gps_lat, gps_lon, gps_alt, yaw) 
            self.fun_timer.stop('adsb_window')

            self.fpsd.draw()

        except Exception:
            self.ex_stop()

        

        

    def ex_stop(self):
        #print("\033c", end="")
        print("***************************************************")
        traceback.print_exc()
        print("***************************************************")

        if self.msg_thread != None:
            mavlinkmsg._run_thread = False
            self.msg_thread.join()
            
            

        if self.phidget_thread != None:
            #PhidgetThread.put_instance()
            #if not PhidgetThread._run_thread:
            self.phidget_thread.close()
            
        if self.aoa_gague != None:
            self.aoa_gague.close()
        
        if self.adsb_window != None:
            self.adsb_window.close()

        if self.gps_td != None:
             self.gps_td.close()

        if self.rdo != None:
            self.rdo.close()

        if self.uav_radio != None:
            self.uav_radio.close()

        if self.hg_gps_td != None:
            self.hg_gps_td.close()

        if self.airspeed !=None:
            self.airspeed.close()

        if self.barometer != None:
            self.barometer.close()


        pyglet.app.exit()
        quit() #exit the python interpretor

    def on_close(self):
        
        self.fun_timer.close()

        print('main_window on closed called')
        self.ex_stop()

    def on_key_press(self, symbol, modifiers):
        if symbol == key.ESCAPE:
            self.on_close()
            return

        if symbol == key.F2:
            self.main_window.set_fullscreen(False)
            self.main_window.set_size(1800, 800)
            return

        if symbol == key.F3:
            self.main_window.set_fullscreen(True)
            return

        if self.alt_tape != None:
            if symbol == key.UP or symbol == key.DOWN or symbol == key.LEFT or symbol == key.RIGHT or symbol == key.TAB:
                self.alt_tape.on_key_press(symbol, modifiers)
                return

        if self.adsb_window != None:
            if symbol == key.SPACE or symbol == key.F1:
                self.adsb_window.on_key_press(symbol, modifiers)
                return
                                                       
    def update(self, dt):
        x=0

    def write_log(self, dt):
        Log.write_log()

if __name__ == '__main__':
    # unit test code

    try:
        if pix_hawk_config.DEBUG:
            mw = MainWindow(1500,700, full_screen=False)
        else:
            mw = MainWindow(1500,750, full_screen=True)
            
            
        # pyglet event loop will run each time update funtion is due and call
        # window draw evet at the beginning of each loop
        pyglet.clock.schedule_interval(mw.update, .05)
        pyglet.clock.schedule_interval(mw.write_log, .5)
        


        pyglet.app.run()

    except Exception:
            traceback.print_exc()
