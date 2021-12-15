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
import traceback

class MainWindow():
    def __init__(self, width, height, full_screen = False):
        if full_screen:
            self.main_window = pyglet.window.Window(fullscreen=True)
        else:
            self.main_window = pyglet.window.Window(width, height)
        self.on_draw = self.main_window.event(self.on_draw)
        self.main_window.on_close = self.on_close

        self.ahdata = pix_hawk_msg.aharsData(-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1)

        self.compass_tape = CompassTape(self.main_window, 150, 200, 800, 65, 6, 30, align=Align.LEFT, orient=Orient.VERT)

        self.roll_gague = RollGague(self.main_window, 1.7*(self.compass_tape.border_rect.height))

        self.alt_tape = AltTapeLeft(self.main_window, self.compass_tape.border_rect.height, 150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)

        self.speed_tape = SpeedTapeRight(self.main_window, self.compass_tape.border_rect.height, self.compass_tape.border_rect.width, 
                150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)

        self.adsb_window = AdsbWindow(self.main_window, self.compass_tape.border_rect.width)

        self.phidget_thread = PhidgetThread.get_instance()

        self.msg_thread = pix_hawk_msg.mavlinkmsg.get_instance()  

        
        
    
    def on_draw(self):
        self.main_window.clear()

        self.ahdata = self.msg_thread.getAharsData(self.ahdata)

        # draw all controls on top of roll gague to overaly top and bot rects
        self.roll_gague.draw(self.ahdata.roll, self.ahdata.pitch)

        yaw = self.phidget_thread.get_yaw()
        self.compass_tape.draw(yaw)

        self.alt_tape.draw(self.ahdata.altitude, self.ahdata.climb)

        self.speed_tape.draw(self.ahdata.airspeed, self.ahdata.groundspeed)

        self.adsb_window.draw()

    def ex_stop(self):
        print("\033c", end="")
        print("***************************************************")
        #print(e)
        print("***************************************************")
        quit()

    def on_close(self):
        print('main_window on closed called')
        
        pix_hawk_msg.mavlinkmsg.put_instance()
        if not pix_hawk_msg.mavlinkmsg._run_thread:
            self.msg_thread.join()

        PhidgetThread.put_instance()
        if not PhidgetThread._run_thread:
            self.phidget_thread.join()

        self.ex_stop()

    def update(self, dt):
        x=0

if __name__ == '__main__':
    # unit test code

    try:
        mw = MainWindow(1500,700, full_screen=False)

        pyglet.clock.schedule_interval(mw.update, .1)

        pyglet.app.run()

    except Exception:
            traceback.print_exc()
