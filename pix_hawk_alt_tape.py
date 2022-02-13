import pyglet
from pyglet import shapes
from pyglet.window import key
from pix_hawk_tape import Tape
from pix_hawk_tape import Align
from pix_hawk_tape import Orient
from pix_hawk_tape import TapeUnit
from pix_hawk_util import Global, DebugPrint
import time



class AltTape(Tape):

    def __init__(self, x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.LEFT, orient=Orient.HORZ):
        super().__init__(x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=align, tape_unit=TapeUnit.FEET_ALT, orient=orient)
        
        
        self.climb_val_label = pyglet.text.Label('',
                          font_size=50,
                          x=self.current_val_rect.x,
                          y=self.border_rect.y+self.border_rect.height+1,
                          anchor_y='bottom', anchor_x='left')
        
        self.climb_val_rect = shapes.BorderedRectangle(self.climb_val_label.x, self.climb_val_label.y, self.current_val_rect.width,
                                                       self.current_val_rect.height, border=10, color = (0, 0, 0),
                                                       border_color = (255,255,255))
        self.pix2climb = self.border_rect.height/2 / 1000
        
        self.up_rect = shapes.Rectangle(self.x-22,self.y+self.border_rect.height/2.,20,200, (0,255,255))
    
        
    def draw(self, alt, climb):
        super().draw(alt)
        DebugPrint.print('climb', str(climb))
        self.climb_val_label.text = str(int(round(climb)))
        self.climb_val_rect.draw()
        
        if climb > 1000:
            climb = 1000
        if climb < -1000:
            climb = -1000
        self.up_rect.height = climb * self.pix2climb
        
        if climb > 0:
            self.up_rect.color = (0,255,0)
            self.climb_val_label.color = (0,255,0,255)
        else:
            self.up_rect.color = (255,0,0)
            self.climb_val_label.color = (255,0,0,255)
        #self.climb_val_label.color = self.up_rect.color
        self.climb_val_label.draw()
        self.up_rect.draw()

class AltTapeLeft(Tape):

    def __init__(self, pyglet_window, compass_height, x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.LEFT, orient=Orient.HORZ):
        super().__init__(x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=align, tape_unit=TapeUnit.FEET_ALT, orient=orient)

        
        self.border_rect.x = pyglet_window._x + 30
        self.border_rect.y = pyglet_window._y
        self.current_val_rect.x = pyglet_window._x + 30
        
        self.border_rect.height = pyglet_window.height - (self.current_val_rect.height + compass_height )
        self.current_val_rect.y = self.border_rect.height/2 
        self.current_val_label.y = self.border_rect.height/2

        self.baro_label = pyglet.text.Label('',
                          font_size=35,
                          x=self.current_val_rect.x + self.current_val_rect.width,
                          y=self.current_val_label.y,
                          color=(0,255,0,255),
                          anchor_y='bottom', anchor_x='left')

        self.climb_val_label = pyglet.text.Label('',
                          font_size=50,
                          x=self.current_val_rect.x,
                          #y=self.border_rect.y+self.border_rect.height+1,
                          y=self.border_rect.y+self.border_rect.height+1,
                          anchor_y='bottom', anchor_x='left')
        
        self.climb_val_rect = shapes.BorderedRectangle(self.climb_val_label.x, self.climb_val_label.y, self.current_val_rect.width,
                                                       self.current_val_rect.height, border=10, color = (0, 0, 0),
                                                       border_color = (255,255,255))

        self.pix2climb = self.border_rect.height/2 / 1000

        
        self.up_rect = shapes.Rectangle(pyglet_window._x+5, pyglet_window._y+self.border_rect.height/2.,20,200, (0,255,255))

        self.tick_pixels = self.border_rect.height/self.tick_count
        self.units2pix_scale = self.tick_pixels/self.units_interval
        self.baro_val = 29.29
        self.alt_mode_gps = True
        self.climb_list = []
        self.climb_alt = 0
        self.climb_time = time.time()
        self.climb_buf_len = 3
        self.climb_weight = self.get_climb_wts(self.climb_buf_len)
        

        
    def draw(self, alt, climb, baro_press):
        if not self.alt_mode_gps:
            alt = self.get_baro_alt(baro_press)
            climb = self.get_baro_climb()
        super().draw(alt)
        print('climb', str(climb))
        if not self.alt_mode_gps:
            self.baro_label.text = '[' + str(self.round_half_up(self.baro_val,decimals=2)) + ']'
            self.baro_label.draw()
        self.climb_val_label.text = str(int(round(climb)))
        self.climb_val_rect.draw()
        
        if climb > 1000:
            climb = 1000
        if climb < -1000:
            climb = -1000
        self.up_rect.height = climb * self.pix2climb
        
        if climb > 0:
            self.up_rect.color = (0,255,0)
            self.climb_val_label.color = (0,255,0,255)
        else:
            self.up_rect.color = (255,0,0)
            self.climb_val_label.color = (255,0,0,255)
        #self.climb_val_label.color = self.up_rect.color
        self.climb_val_label.draw()
        self.up_rect.draw()
    
    def get_climb_wts(self, buf_len):
        wts = []
        for i in range(buf_len):
            wts.append(1 / (i+1))
        return wts

    def get_baro_climb(self):
        num = [a*b for a,b in zip(self.climb_list, self.climb_weight)]
        num = sum(num)
        denom = sum(self.climb_weight)
        w_mean = num/denom
        #print('w mean climb rate:', w_mean)
        Global.set_baro_climb(w_mean)
        return w_mean

    def set_baro_climb(self, alt):
        alt_dif = alt - self.climb_alt
        time_dif = time.time() - self.climb_time
        climb_rate = alt_dif / time_dif # feet / sec
        climb_rate /= 60 # feet / min
        self.climb_list.insert(0, climb_rate)
        
        if len(self.climb_list) > self.climb_buf_len:
            self.climb_list.pop()
        

    def get_baro_alt(self, cur_pressure_hpa):
        in_mecury = self.baro_val
        hpa = 33.86389 * in_mecury
        baro_pressure = hpa # set this from atis, convert from in to pa
        alt = 44330 * (1 - (cur_pressure_hpa/baro_pressure ) ** (1/5.255))
        baro_alt = alt * 3.28084 # convert meters to ftSS

        self.set_baro_climb(baro_alt)
        self.climb_time = time.time()
        self.climb_alt = baro_alt
        return baro_alt


    def on_key_press(self,symbol, modifiers):
        inc = .1
        if modifiers & key.MOD_SHIFT:
            inc = 1
        if symbol == key.UP: 
            if self.baro_val < 35:
                self.baro_val += inc
        elif symbol == key.DOWN:
            if self.baro_val > 25:
                self.baro_val -= inc
        elif symbol == key.LEFT:
            if self.baro_val < 35:
                self.baro_val += .01
        elif symbol == key.RIGHT:
            if self.baro_val > 25:
                self.baro_val -= .01
        elif symbol == key.TAB:
            self.alt_mode_gps = not self.alt_mode_gps
            Global.set_alt_mode_gps(self.alt_mode_gps)
                
        
    
if __name__ == '__main__':
    # unit test code

    import cProfile, pstats
    profiler = cProfile.Profile()
    
    
    print('__main__')
    mock_angle = 5000
    mock_delta = 2

    mock_pressure = 900
    mock_pressure_delta = 20

    climb = 0
    climb_delta = 100
    def on_draw():
        #profiler.enable()
        window.clear()
        #rect.draw()
        tape.draw(mock_angle, climb, mock_pressure)
        #profiler.disable()
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    def mock_pressure_fn(dt):
        global mock_pressure
        global mock_pressure_delta
        mock_pressure = mock_pressure + mock_pressure_delta
        if mock_pressure > 900:
            mock_pressure = 900
            mock_pressure_delta = -mock_pressure_delta
        if mock_pressure < 700:
            mock_pressure = 700
            mock_pressure_delta = -mock_pressure_delta

        
    def mock_data(dt):
        print('mock_data')
        global mock_angle
        global mock_delta
        global mock_units
        global climb
        global climb_delta

            
        #if mock_units == TapeUnit.FEET_ALT:
        mock_angle = mock_angle + mock_delta
        if mock_angle > 8000:
            mock_angle = 8000
            mock_delta = -2
        if mock_angle < 0:
            mock_angle = 0
            mock_delta = 2
            
        climb = climb + climb_delta
        if climb > 1500:
            climb = 1500
            climb_delta = -100
        if climb < -1500:
            climb = -1500
            climb_delta = 100

        
    window = pyglet.window.Window(1000,1000)
    window.on_draw = on_draw
    center_x = window.width/2
    center_y = window.height/2
    #rect = shapes.BorderedRectangle(center_x, center_y,  100, 100, border=3, color = (0, 0, 255),
                                            #border_color = (255,255,255))
    tape = AltTapeLeft(window, 75, 150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)
    tape.alt_mode_gps = False
    #tape = Tape(50, 100, 500, 75, 6, 100, align=Align.RIGHT, tape_unit=TapeUnit.FEET_VERT_SPEED, orient=Orient.VERT)
    
    pyglet.clock.schedule_interval(update, .1)
    #pyglet.clock.schedule_interval(mock_data, .5)
    pyglet.clock.schedule_interval(mock_pressure_fn, .5)

    #profiler.enable()

    pyglet.app.run()

    #profiler.disable()
    #stats = pstats.Stats(profiler).sort_stats('cumtime')
    
    #stats.print_stats(.1)