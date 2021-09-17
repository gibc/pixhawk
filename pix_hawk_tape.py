import pyglet
from pyglet import shapes
from enum import Enum
from math import floor

class TapeUnit(Enum):
    MPH = 1
    DEGREE = 2
    FEET = 3

class Tape:

    def __init__(self, x, y, pixel_wd, pixel_ht, tick_count, units_interval, origin_offset, tape_unit=TapeUnit.DEGREE, horizontal=True):
        self.x = x #postion of tape in window
        self.y = y
        self.pixel_wd = pixel_wd
        self.pixel_ht = pixel_ht
        self.tick_count = tick_count
        self.units_interval = units_interval  #eg 30 degrees
        self.tape_unit = tape_unit
        self.origin_offset = origin_offset
        self.current_value_lable = pyglet.text.Label('--',
                          font_size=30,
                          x=0,
                          y=0,
                          anchor_y='center', anchor_x='center')
        self.tick_labels = []
        if not horizontal:
            self.pixel_wd = pixel_ht
            serl.pixel_ht = pixel_wd
            
        tick_pixels = self.pixel_wd/self.tick_count
        self.units2pix_scale = self.units_interval/tick_pixels
 
        self.border_rect = shapes.BorderedRectangle(self.x, self.y,  self.pixel_wd, self.pixel_ht, border=3, color = (0, 0, 255),
                                            border_color = (255,255,255))
        
        self.current_val_rect = shapes.BorderedRectangle(self.x+self.pixel_wd/2, self.y,  125, self.pixel_ht, border=10, color = (0, 0, 255),
                                            border_color = (255,255,255))
        for i in range(tick_count):
            self.tick_labels.append(pyglet.text.Label('--',
                          font_size=30,
                          x=0,
                          y=0,
                          anchor_y='center', anchor_x='center'))
            
            
            
    def draw(self, current_val):
        
        heading_origin = self.get_90_origin(current_val)
        
        for i in range (self.tick_count):
                
            nxt = self.get_tick_angle(heading_origin, i, self.units_interval)
            self.tick_labels[i].text = self.get_heading_str(nxt)
        
        #heading_label_1.text = get_heading_str(nxt)
            org_offset = self.angle_dif_right(heading_origin, nxt)
        #heading_label_1.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
            self.tick_labels[i].x = int(self.border_rect.x - self.border_rect.width/2) + int(self.units2pix_scale*org_offset)
        #heading_label_1.color = get_heading_color(nxt)
            self.tick_labels[i].color = self.get_heading_color(nxt)
        #heading_label_1.draw()
            self.tick_labels[i].draw()
            
        
        
        #heading_label_rect.draw()
        self.border_rect.draw()
        self.current_val_rect.draw()
        #heading_label.text = get_heading_str(round(abs(ahdata.heading)))
        self.current_value_lable.text = self.get_heading_str(round(abs(current_val)))
        #heading_label.draw()
        self.current_value_lable.draw()
        
    def get_90_origin(self, a1):
        if a1 > 90:
            return a1 - 90
        else:
            return 360+(a1-90)
        
    def get_tick_angle(self, origin, tick_number, tick_interval):
    #print('origin', origin)
    #print('tick_number', tick_number)
    #print('tick_interval', tick_interval)
    #dist = fmod(origin, tick_interval) # dist to tick 1
    #print("math mod", fmod(origin, tick_interval))
        tick1_pos = self.round_down(origin, tick_interval)
        tick1_pos = tick1_pos+tick_interval
    #print('origin', origin)
    #print('tick1_pos', tick1_pos)
    
        tick_number = tick_number - 1
        tick_n_pos = self.get_sum_right(tick1_pos, tick_interval*tick_number)
    #print('tick_n_pos', tick_n_pos)
        return tick_n_pos


    def round_down(self, num, divisor):
        return floor(num / divisor) * divisor

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
       
    def get_heading_str(self, heading):
        if heading == 0:
            return '<N>'
        if heading == 360:
            return '<N>'
        if heading == 90:
            return '<E>'
        if heading == 180:
            return '<S>'
        if heading == 270:
            return '<W>'
        else:
            return str(heading)+'°'
    
    def get_heading_color(self, heading):
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


if __name__ == '__main__':
    tape = Tape(100, 100, 200, 50, 6, 30, 90, tape_unit=TapeUnit.FEET, horizontal=True)
    tape.draw(299)
        