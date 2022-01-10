
"""NOTE!!!!
this script started from
/etc/xdg/lxsession/LXDE-pi/autostart
xterm -e python3 /home/pi/PhidgetInsurments/pix_hawk_ui.py
"""
#from re import S
import pyglet
from pyglet import clock
import pix_hawk_msg
from pix_hawk_roll_pitch import RollGague
from pix_hawk_compass_tape import CompassTape
from pix_hawk_tape import Align
from pix_hawk_tape import Orient
from pix_hawk_alt_tape import AltTapeLeft
from pix_hawk_speed_tape import SpeedTapeRight
from pix_hawk_adsb import AdsbWindow
from PhidgetThread import PhidgetThread
from pix_hawk_gps import GPS_Window
from pix_hawk_wind import WindChild
from pix_hawk_aoa import Aoa
import traceback

class MainWindow():
    def __init__(self, width, height, full_screen = False):

        try:
            if full_screen:
                self.main_window = pyglet.window.Window(fullscreen=True)
            else:
                self.main_window = pyglet.window.Window(width, height)
            self.on_draw = self.main_window.event(self.on_draw)
            self.main_window.on_close = self.on_close

            self.msg_thread = pix_hawk_msg.mavlinkmsg.get_instance()  

            self.ahdata = pix_hawk_msg.aharsData(-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1)

            self.compass_tape = CompassTape(self.main_window, 150, 200, 800, 65, 6, 30, align=Align.LEFT, orient=Orient.VERT)

            self.roll_gague = RollGague(self.main_window, 1.7*(self.compass_tape.border_rect.height))

            self.alt_tape = AltTapeLeft(self.main_window, self.compass_tape.border_rect.height, 150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)

            self.speed_tape = SpeedTapeRight(self.main_window, self.compass_tape.border_rect.height, self.compass_tape.border_rect.width, 
                150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)

            self.adsb_window = AdsbWindow(self.msg_thread.adsb_dic, self.main_window, self.compass_tape.border_rect.width)

            self.gps_window = GPS_Window(self.main_window, self.compass_tape.border_rect.width)

            self.wind_gague = WindChild(self.main_window, self.compass_tape.border_rect.width, 0, 0, 350, 160)

            self.aoa_gague = Aoa(self.main_window, self.compass_tape.border_rect.width, 100, 200, 20)

            self.phidget_thread = PhidgetThread.get_instance()

            self.adsbwin = AdsbWindow(self.msg_thread.adsb_dic, self.main_window, self.compass_tape.border_rect.width) 

        except Exception:
            self.ex_stop()

        
        
    
    def on_draw(self):

        try:
            self.main_window.clear()

            self.ahdata = self.msg_thread.getAharsData(self.ahdata)

            # draw all controls on top of roll gague to overaly top and bot rects
            self.roll_gague.draw(self.ahdata.roll, self.ahdata.pitch)

            if self.phidget_thread != None:
                yaw = self.phidget_thread.get_yaw()
            else:
                yaw = 0
            self.compass_tape.draw(yaw)

            self.alt_tape.draw(self.ahdata.altitude, self.ahdata.climb)

            self.speed_tape.draw(self.ahdata.airspeed, self.ahdata.groundspeed)

            self.gps_window.draw(self.ahdata.fix_type, self.ahdata.gnd_track, self.ahdata.groundspeed, yaw, self.ahdata.gps_alt)

            #draw_calc(self, airspeed, heading, gnd_speed, gnd_track, altitude):
            self.wind_gague.draw_calc(self.ahdata.airspeed, yaw, self.ahdata.groundspeed, self.ahdata.gnd_track, self.ahdata.altitude)

            self.aoa_gague.draw(self.ahdata.airspeed, self.ahdata.climb, self.ahdata.pitch)

            self.adsbwin.draw(self.ahdata.lat, self.ahdata.lon, self.ahdata.gps_alt, self.ahdata.gnd_track)

        except Exception:
            self.ex_stop()

        

        

    def ex_stop(self):
        print("\033c", end="")
        print("***************************************************")
        traceback.print_exc()
        print("***************************************************")

        if pix_hawk_msg != None:
            pix_hawk_msg.mavlinkmsg.put_instance()
            if not pix_hawk_msg.mavlinkmsg._run_thread:
                self.msg_thread.join()

        if self.phidget_thread != None:
            PhidgetThread.put_instance()
            if not PhidgetThread._run_thread:
                self.phidget_thread.join()

        if self.aoa_gague != None:
            self.aoa_gague.close()


        pyglet.app.exit()
        quit() #exit the python interpretor

    def on_close(self):
        print('main_window on closed called')
        
        """pix_hawk_msg.mavlinkmsg.put_instance()
        if not pix_hawk_msg.mavlinkmsg._run_thread:
            self.msg_thread.join()

        PhidgetThread.put_instance()
        if not PhidgetThread._run_thread:
            self.phidget_thread.join()"""

        self.ex_stop()

    def update(self, dt):
        x=0

if __name__ == '__main__':
    # unit test code

    try:
        mw = MainWindow(1500,700, full_screen=True)

        pyglet.clock.schedule_interval(mw.update, .1)

        pyglet.app.run()

    except Exception:
            traceback.print_exc()
