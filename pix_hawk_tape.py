from re import A, S
import pyglet
from pyglet import shapes
from enum import Enum
from math import floor
import tkinter as Tkinter 
from tkinter import font as tkFont
import time

class TapeUnit(Enum):
    MPH = 1
    DEGREE = 2
    FEET_ALT = 3
    FEET_VERT_SPEED = 4
    
class Orient(Enum):
    HORZ = 1
    VERT = 2
    
class Align(Enum):
    RIGHT = 1
    LEFT = 2
    CENTER = 3
 
"""
import tkinter as Tkinter 
from tkinter import font as tkFont

Tkinter.Frame().destroy()
txt = tkFont.Font(family="Times New Roman", size=14)
width = txt.measure("What the heck?")
print(width)

label = pyglet.text.Label(text,
                          font_name ='Times New Roman',
                          font_size = 28,
                          x = 20, y = window.height//2, )
"""
Tkinter.Frame().destroy()
def get_str_wd(string, size):
    #tic = time.perf_counter()
    txt = tkFont.Font(family="Times New Roman", size=size)
    toc = time.perf_counter()
    width = txt.measure(string)
    #toc = time.perf_counter()
    #print(f"get_str_wd in {toc - tic:0.4f} seconds")

    return width
    

class Tape:

    def __init__(self, x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.LEFT, tape_unit=TapeUnit.DEGREE, orient=Orient.HORZ):
        self.x = x #postion of tape in window
        self.y = y
        self.orient = orient
        self.align = align
        self.pixel_wd = pixel_wd
        self.pixel_ht = pixel_ht
        self.tick_count = tick_count
        self.units_interval = units_interval  #eg 30 degrees
        self.tape_unit = tape_unit
        self.origin_offset = (tick_count * units_interval) / 2
        
        """
        self.lable_wd = 120
        self.lable_ht = pixel_ht  
        if not horizontal:
            self.lable_wd = pixel_ht
            self.lable_ht = 120
        """
        
        self.tick_labels = []
        """
        if not self.orient == Orient.HORZ:
            self.pixel_wd = pixel_ht
            self.pixel_ht = pixel_wd
        """
            
        self.tick_pixels = self.pixel_wd/self.tick_count
        self.units2pix_scale = self.tick_pixels/self.units_interval
 
        #self.border_rect = shapes.BorderedRectangle(self.x, self.y,  self.pixel_wd, self.pixel_ht, border=3, color = (0, 0, 255),
        #                                    border_color = (255,255,255))
        self.border_rect = self.get_border_rect()
        
        self.curval_wd = 125
        self.current_val_rect = self.get_value_rect()
        
        #if self.orient == Orient.HORZ:
        #    self.current_val_rect = shapes.BorderedRectangle(self.x+self.pixel_wd/2-self.curval_wd/2., self.y,  self.curval_wd, self.pixel_ht, border=10, color = (0, 0, 0),
        #                                    border_color = (255,255,255))
        #else:
        #    self.current_val_rect = shapes.BorderedRectangle(self.x, y+self.border_rect.height/2-self.pixel_wd/2,  120, 50, border=10, color = (0, 0, 0),
        #                                    border_color = (255,255,255))
        self.current_val_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.current_val_rect.x,
                          y=self.current_val_rect.y,
                          anchor_y='bottom', anchor_x='left')
        for i in range(tick_count):
            self.tick_labels.append(pyglet.text.Label('****',
                          font_size=30,
                          x=self.current_val_rect.x,
                          y=self.current_val_rect.y,
                          anchor_y='bottom', anchor_x='center'))
            
            
            
    def draw(self, current_val):
        if self.tape_unit == TapeUnit.DEGREE:
            heading_origin = self.get_90_origin(current_val)
        if self.tape_unit == TapeUnit.MPH or self.tape_unit == TapeUnit.FEET_ALT or self.tape_unit == TapeUnit.FEET_VERT_SPEED:
            heading_origin = self.get_mph_origin(current_val)
        self.border_rect.draw()
        
        
        #for i in range (self.tick_count): allow deived class to override tick count to all commpass to have 7 tick lables
        for i in range (len(self.tick_labels)):
                
            nxt = self.get_tick_value(heading_origin, i, self.units_interval)
            #print('nxt', nxt)
            if nxt < 0 and self.tape_unit != TapeUnit.FEET_VERT_SPEED:
                continue
            self.tick_labels[i].text = self.get_value_str(nxt)
            #self.tick_labels[i].text = str(nxt)
            #print('self.tick_labels[i].text ', self.tick_labels[i].text)
        
            org_offset = self.angle_dif_right(heading_origin, nxt)
            #print('****org_offset****', org_offset)
            
            if self.orient == Orient.HORZ:
                org_offset -= 30 # shift origin to the left to allow 7 tick lables
                self.tick_labels[i].x = int(self.border_rect.x) + int(self.units2pix_scale*org_offset)
            else:
                self.tick_labels[i].y = int(self.border_rect.y) + int(self.units2pix_scale*org_offset)
                if self.align == Align.LEFT:
                    self.tick_labels[i].x = self.current_val_rect.x
                else:
                    self.tick_labels[i].x = self.border_rect.x
                self.tick_labels[i].x = self.get_str_pos_in_rect(self.get_value_str(nxt), self.border_rect, Align.RIGHT, 37 )
                self.tick_labels[i].anchor_x = 'left'
                self.tick_labels[i].anchor_y = 'center'
                
            str_wd = self.get_str_wd(self.tick_labels[i].text, 30)
            if self.orient == Orient.HORZ:
                if self.tick_labels[i].x + str_wd/4 < self.border_rect.x: #+ str_wd:
                    continue
                if self.tick_labels[i].x - str_wd/4 > self.border_rect.x+self.border_rect.width: #- str_wd:
                    continue
                pass
            else:
                if self.tick_labels[i].y < self.border_rect.y:
                    continue
                if self.tick_labels[i].y > self.border_rect.y + self.border_rect.height-5:
                    continue

            
            #print('int(self.units2pix_scale*org_offset)',int(self.units2pix_scale*org_offset))
            #print('self.border_rect.x', self.border_rect.x)
            #print('self.units2pix_scale', self.units2pix_scale)
            #print('self.tick_labels[i].x',self.tick_labels[i].x)
        
            self.tick_labels[i].color = self.get_value_color(nxt)
        
            self.tick_labels[i].draw()
            #print('self.tick_labels[i].x: ', self.tick_labels[i].x)
            
        if self.tape_unit == TapeUnit.DEGREE:
            self.current_val_label.text = self.get_value_str((abs(current_val)))
        if self.tape_unit == TapeUnit.MPH or self.tape_unit == TapeUnit.FEET_VERT_SPEED:
            self.current_val_label.x = self.current_val_rect.x
            if current_val < 100:
                self.current_val_label.x += 25
            if current_val < 10:
                self.current_val_label.x += 25
            #self.current_val_label.text = str(current_val)
            self.current_val_label.text = str(self.round_half_up(current_val,1))
        if self.tape_unit == TapeUnit.FEET_ALT:
            self.current_val_label.x = self.current_val_rect.x
            if current_val < 1000:
                self.current_val_label.x += 25
            if current_val < 100:
                self.current_val_label.x += 25
            if current_val < 10:
                self.current_val_label.x += 25
            self.current_val_label.text = str(self.round_half_up(current_val,1))
        #if self.tape_unit == TapeUnit.DEGREE:
        #    self.current_val_label.x = self.self.current_val_rect.x

        #self.current_val_label.x = self.get_str_pos_in_rect(str(current_val), self.current_val_rect, Align.RIGHT, 37 )
        self.current_val_label.x = self.get_str_pos_in_rect(self.current_val_label.text, self.current_val_rect, Align.RIGHT, 37 )
            
        self.current_val_rect.draw()
        self.current_val_label.draw()
        
    def get_border_rect(self):
        br_ht = self.pixel_ht
        br_wd = self.pixel_wd
        if self.orient == Orient.VERT:
            br_ht = self.pixel_wd
            br_wd = self.pixel_ht
            
        rect = shapes.BorderedRectangle(self.x, self.y,  br_wd, br_ht, border=3, color = (100, 100, 100),
                                            border_color = (255,255,255))
        rect.opacity = 100
        #rect = shapes.Rectangle(self.x, self.y,  br_wd, br_ht, color = (0, 0, 255, 100))
        return rect
    
    def get_value_rect(self):
        br_ht = self.pixel_ht
        br_wd = self.curval_wd
        x = self.x+self.pixel_wd/2-self.curval_wd/2
        y = self.y
        
        if self.orient == Orient.VERT:
            font_ht = 73
            if self.tape_unit == TapeUnit.MPH:
                br_wd = self.pixel_ht*2.4
            elif self.tape_unit == TapeUnit.FEET_ALT:
                br_wd = self.pixel_ht*3.2
            br_ht = font_ht
            if self.align == Align.LEFT:
                x = self.x#+self.border_rect.width
            else:
                #x = self.x - self.border_rect.width
                x = self.border_rect.x + self.border_rect.width - br_wd
            y = self.y+self.border_rect.height/2-br_ht/2
            
        
        return shapes.BorderedRectangle( x, y, br_wd, br_ht, border=10, color = (0, 0, 0), border_color = (255,255,255))

    def get_str_pos_in_rect(self, string, rect, align, font_ht):
        #if align == Align.LEFT:
        if self.align == Align.LEFT:
            return rect.x

        if self.tape_unit == TapeUnit.DEGREE:
            wd = self.get_str_wd(string, font_ht)
           
            if "<" not in string:
                wd *= 1.65
            else:
                wd *= 1.75
            return rect.x + rect.width/2 - wd/2
        
        else:
            wd = self.get_str_wd(string, font_ht)
            return rect.x + rect.width -wd
        
        
    
    def get_str_wd(self, str, font_ht):
        return get_str_wd(str, font_ht)
        #return len(str)*8.5
        
    def get_90_origin(self, a1):
        #of = 90
        of = 120
        if a1 > of:
            return a1 - of
        else:
            return 360+(a1-of)
        
    def get_mph_origin(self, cur_val):
        return cur_val - self.origin_offset
        
    def get_tick_value(self, origin, tick_number, tick_interval):
        
        if self.tape_unit == TapeUnit.DEGREE:
    
            tick1_pos = self.round_down(origin, tick_interval)
            tick1_pos = tick1_pos+tick_interval
            tick_n_pos = self.get_sum_right(tick1_pos, tick_interval*tick_number)
    
            return tick_n_pos
        
        if self.tape_unit == TapeUnit.MPH or self.tape_unit == TapeUnit.FEET_ALT or self.tape_unit == TapeUnit.FEET_VERT_SPEED:
    
            tick1_pos = self.round_down(origin, tick_interval)
            tick1_pos = tick1_pos+tick_interval
            tick_n_pos = tick1_pos + tick_interval*tick_number

        return tick_n_pos


    def round_down(self, num, divisor):
        return floor(num / divisor) * divisor
    
    def round_half_up(self, n, decimals=0):
        multiplier = 10 ** decimals
        return int(floor(n*multiplier + 0.5) / multiplier)

    def get_sum_right(self, a1, a2):
    #print('get_sum_right', a1, a2)
        sum = a1+a2
        if sum < 360:
            return sum
        else:
            return sum - 360

    def angle_dif_right(self, a1, a2):
        if a2 > a1:
            return a2 - a1
        else:
            to360 = 360 - a1
        return to360 + a2
       
    def get_value_str(self, value):
        if self.tape_unit == TapeUnit.DEGREE:
            if value == 0:
                return '<N>'
            if value == 360:
                return '<N>'
            if value == 90:
                return '<E>'
            if value == 180:
                return '<S>'
            if value == 270:
                return '<W>'
            else:
                return str(value)+'°'
            
        if self.tape_unit == TapeUnit.MPH or self.tape_unit == TapeUnit.FEET_VERT_SPEED:
            value = self.round_half_up(value, 1)
            value = int(value/10)
            """
            if value < 10 and self.tape_unit != TapeUnit.FEET_VERT_SPEED:
                return '  '+str(value)
            if self.tape_unit == TapeUnit.FEET_VERT_SPEED:
                if value >= 0:
                    return ' '+str(value)
                else:
                    return str(value)
            """
            return str(value)#+"m"
        
        if self.tape_unit == TapeUnit.FEET_ALT:
            value = self.round_half_up(value, 2)
            value = int(value/100)
            return str(value)#+"ft"
            
    
    def get_value_color(self, heading):
        if self.tape_unit == TapeUnit.DEGREE:
            ordinal_color = (255,255,0,255)
            if heading == 0:
                return ordinal_color
            if heading == 360:
                return ordinal_color
            if heading == 90:
                return ordinal_color
            if heading == 180:
                return ordinal_color
            if heading == 270:
                return ordinal_color
            else:
                return (255,255,255,255)
        else:
            return (255,255,255,255)

if __name__ == '__main__':
    # unit test code
    #mock_angle = 5000
    
    #wd = get_str_wd('a long string a long string a long string ', 30)
    #print('str wd ', wd)
    #quit()
    
    mock_angle = 0
    mock_delta = 2
    mock_units = TapeUnit.MPH
    def on_draw():
        window.clear()
        #rect.draw()
        tape.draw(mock_angle)
    
    def update(dt):
        x=0
        #print("dt: ", dt)
        
    def mock_data(dt):
        global mock_angle
        global mock_delta
        global mock_units
        if mock_units == TapeUnit.DEGREE:
            mock_angle = mock_angle + mock_delta
            if mock_angle > 360:
                mock_angle = 360
                mock_delta = -5
            if mock_angle < 0:
                mock_angle = 0
                mock_delta = 5
            print('mock_angle ',mock_angle)
        
        if mock_units == TapeUnit.MPH:
            mock_angle = mock_angle + mock_delta
            if mock_angle > 150:
                mock_angle = 150
                mock_delta = -2
            if mock_angle < 0:
                mock_angle = 0
                mock_delta = 2
            print('mock_mph ',mock_angle)
            
        if mock_units == TapeUnit.FEET_ALT:
            mock_angle = mock_angle + mock_delta
            if mock_angle > 8000:
                mock_angle = 8000
                mock_delta = -2
            if mock_angle < 0:
                mock_angle = 0
                mock_delta = 2
            print('mock_feet ',mock_angle)
            
        if mock_units == TapeUnit.FEET_VERT_SPEED:
            mock_angle = mock_angle + mock_delta
            if mock_angle > 1000:
                mock_angle = 1000
                mock_delta = -100
            if mock_angle < -1000:
                mock_angle = 0
                mock_delta = 100
            print('mock_feet ',mock_angle) 
            
        pass
        
    window = pyglet.window.Window(1000,700)
    window.on_draw = on_draw
    center_x = window.width/2
    center_y = window.height/2
    #rect = shapes.BorderedRectangle(center_x, center_y,  100, 100, border=3, color = (0, 0, 255),
                                            #border_color = (255,255,255))
    tape = Tape(50, 100, 500, 55, 6, 10, align=Align.LEFT, tape_unit=TapeUnit.MPH, orient=Orient.VERT)
    #tape = Tape(50, 100, 500, 75, 6, 100, align=Align.RIGHT, tape_unit=TapeUnit.FEET_VERT_SPEED, orient=Orient.VERT)
    
    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .5)

    pyglet.app.run()
    
    
    
        