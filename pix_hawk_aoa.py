from re import A, M
from threading import Event
from tkinter.constants import W
import pyglet
from pyglet import clock
from pyglet import shapes
import math
from math import floor
import traceback
from pix_hawk_test import MockVar
from pix_hawk_sound import SoundThread
import traceback


class Aoa():

    def __init__(self, window, compass_width, wd, ht, stripe_count):
        self.wd = wd
        self.ht = ht
        self.x = window._x + compass_width/8
        self.y = window._y + window.height * (.2)
        self.stripe_ht = ht/stripe_count
        self.sound = SoundThread.get_instance()
        self.stall_aoa_range = 4
        self.stall_aoa_start = 4
        self.border_rect = shapes.BorderedRectangle(self.x, self.y, self.wd,
                                                       self.ht, border=0, color=(0,0,0),
                                                       border_color = (255,255,255))

        self.aoa_label = pyglet.text.Label('AOA:',
                          font_size=50,
                          x=self.border_rect.x ,
                          y=self.border_rect.y ,
                          color=(255,255,255,255),
                          anchor_y='top', anchor_x='left')

        self.stripe_rects = []
        for i in range(0, stripe_count):
            stripe_wd = (i+1)/stripe_count * wd
            x = window._x + self.border_rect.x + wd/2
            y = window._y + self.border_rect.y + i*self.stripe_ht
            red = i * 255/stripe_count
            if red > 50:
                red = 255
            green = (stripe_count - i) * 255/stripe_count
            if green > 150:
                green = 255
            rect = shapes.Rectangle(x,y,stripe_wd, self.stripe_ht, color=(int(red),int(green),0))
            rect.anchor_x = rect.width/2
            #rect.anchor_y = 'bottom'
            self.stripe_rects.append(rect)

    

    def round_half_up(self, n, decimals=0):
        multiplier = 10 ** decimals
        ret = floor(n*multiplier + 0.5) / multiplier
        if decimals == 0:
            ret = int(ret)
        return ret
        
    def draw(self, airspeed_, climb, pitch):
        try:
            #if airspeed_ == 0 or climb == 0 or pitch == 0:
            #    self.border_rect.draw() 
            #    return
            if airspeed_ < 30:
                airspeed_  = 30
            if climb == 0:
                climb = .1
            if pitch == 0:
                pitch = .1

            pix_per_degree = self.border_rect.height/10
        # convert mph to feet per min
            airspeed = airspeed_ * 88
            air_sin = climb/airspeed
            if abs(air_sin) > 1:
                air_sin = 0
            air_angle = math.asin(air_sin)
            air_angle = math.degrees(air_angle)
            aoax = pitch - air_angle
            aoa_pix = aoax*pix_per_degree

            if aoax < 0:
                aoax = 0

            self.set_tone(aoax)
    
            self.aoa_label.text = str(self.round_half_up(aoax,1)) #+':'+str(airspeed_)+':'+str(climb)+':'+str(pitch)
            self.stripe_rects[0].draw()
            for i in range(0,len(self.stripe_rects)):
                rect = self.stripe_rects[i]
                if i*self.stripe_ht <= aoa_pix:
                    rect.draw()
            
            self.aoa_label.draw()

        except Exception as ex:
            traceback.print_exc()
            self.border_rect.draw()
            self.aoa_label.text = str(ex)
            self.aoa_label.draw()

    def set_tone(self, aoa):
        try:
            if(self.sound != None):
                if aoa >= self.stall_aoa_start:
                    volume = (aoa - self.stall_aoa_start) / self.stall_aoa_range
                    if volume > .7:
                        volume = .7
                    self.sound.set_tone_mode(True)
                    self.sound.start_tone(volume)
                elif self.sound.get_tone_mode():
                    self.sound.stop_tone()
                    self.sound.set_tone_mode(False)
                    
        except Exception:
            traceback.print_exc()
            


    def close(self):
        if self.sound != None:
            SoundThread.put_instance()

if __name__ == '__main__':

    window = pyglet.window.Window(1500,700)
    #window = pyglet.window.Window(fullscreen=True)
        
    
    aoa_gague = Aoa(window, 750, 100, 200, 20)

    def on_draw():
        window.clear()
        aoa_gague.draw(speed.current, climb.current, pitch.current)
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    def mock_data(dt):
        pitch.inc_var()

    window.on_draw = on_draw

    speed = MockVar(50,150,10)
    climb = MockVar(-1000,1000, 100, speed)
    pitch = MockVar(-15,+15, 1, climb)

    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .1)
    
    pyglet.app.run()