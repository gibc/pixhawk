import pyglet
from pyglet import shapes
from enum import Enum
from math import floor

class TapeUnit(Enum):
    MPH = 1
    DEGREE = 2
    FEET = 3
    
class Orient(Enum):
    HORZ = 1
    VERT = 2
    

class Tape:

    def __init__(self, x, y, pixel_wd, pixel_ht, tick_count, units_interval, origin_offset, tape_unit=TapeUnit.DEGREE, orient=Orient.HORZ):
        self.x = x #postion of tape in window
        self.y = y
        self.orient = orient
        self.pixel_wd = pixel_wd
        self.pixel_ht = pixel_ht
        self.tick_count = tick_count
        self.units_interval = units_interval  #eg 30 degrees
        self.tape_unit = tape_unit
        self.origin_offset = origin_offset
        
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
                          font_size=30,
                          x=self.current_val_rect.x+self.curval_wd/2,
                          y=self.current_val_rect.y,
                          anchor_y='bottom', anchor_x='center')
        for i in range(tick_count):
            self.tick_labels.append(pyglet.text.Label('****',
                          font_size=30,
                          x=self.current_val_rect.x,
                          y=self.current_val_rect.y,
                          anchor_y='bottom', anchor_x='center'))
            
            
            
    def draw(self, current_val):
        
        heading_origin = self.get_90_origin(current_val)
        self.border_rect.draw()
        
        
        for i in range (self.tick_count):
                
            nxt = self.get_tick_angle(heading_origin, i, self.units_interval)
            print('nxt', nxt)
            self.tick_labels[i].text = self.get_heading_str(nxt)
        
            org_offset = self.angle_dif_right(heading_origin, nxt)
            print('org_offset', org_offset)
            
            if self.orient == Orient.HORZ:
                self.tick_labels[i].x = int(self.border_rect.x) + int(self.units2pix_scale*org_offset)
            else:
                self.tick_labels[i].y = int(self.border_rect.y) + int(self.units2pix_scale*org_offset)
                self.tick_labels[i].x = self.current_val_rect.x
                self.tick_labels[i].anchor_x = 'left'
                self.tick_labels[i].anchor_y = 'center'
                
            str_wd = self.get_str_wd(self.tick_labels[i].text)
            if self.orient == Orient.HORZ:
                if self.tick_labels[i].x < self.border_rect.x + str_wd:
                    continue
                if self.tick_labels[i].x > self.border_rect.x+self.border_rect.width - str_wd:
                    continue
            else:
                pass
            
            #print('int(self.units2pix_scale*org_offset)',int(self.units2pix_scale*org_offset))
            print('self.border_rect.x', self.border_rect.x)
            print('self.units2pix_scale', self.units2pix_scale)
            #print('self.tick_labels[i].x',self.tick_labels[i].x)
        
            self.tick_labels[i].color = self.get_heading_color(nxt)
        
            self.tick_labels[i].draw()
            #print('self.tick_labels[i].x: ', self.tick_labels[i].x)
            
        
        self.current_val_label.text = self.get_heading_str(round(abs(current_val)))
        self.current_val_rect.draw()
        self.current_val_label.draw()
        
    def get_border_rect(self):
        br_ht = self.pixel_ht
        br_wd = self.pixel_wd
        if self.orient == Orient.VERT:
            br_ht = self.pixel_wd
            br_wd = self.pixel_ht
            
        return shapes.BorderedRectangle(self.x, self.y,  br_wd, br_ht, border=3, color = (0, 0, 255),
                                            border_color = (255,255,255))
    
    def get_value_rect(self):
        br_ht = self.pixel_ht
        br_wd = self.curval_wd
        x = self.x+self.pixel_wd/2-self.curval_wd/2
        y = self.y
        
        if self.orient == Orient.VERT:
            font_ht = 50
            br_wd = self.pixel_ht
            br_ht = font_ht
            x = self.x
            y = self.y+self.border_rect.height/2-br_ht/2
            
        
        return shapes.BorderedRectangle( x, y, br_wd, br_ht, border=10, color = (0, 0, 0), border_color = (255,255,255))

        
    
    def get_str_wd(self, str):
        return len(str)*8.5
        
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
    
        #tick_number = tick_number - 1
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
    # unit test code
    mock_angle = 0
    mock_delta = 5
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
        mock_angle = mock_angle + mock_delta
        if mock_angle > 360:
            mock_angle = 360
            mock_delta = -5
        if mock_angle < 0:
            mock_angle = 0
            mock_delta = 5
        print('mock_angle ',mock_angle)
            
            
        pass
        
    window = pyglet.window.Window(1000,700)
    window.on_draw = on_draw
    center_x = window.width/2
    center_y = window.height/2
    #rect = shapes.BorderedRectangle(center_x, center_y,  100, 100, border=3, color = (0, 0, 255),
                                            #border_color = (255,255,255))
    tape = Tape(50, 100, 500, 120, 6, 30, 90, tape_unit=TapeUnit.DEGREE, orient=Orient.VERT)
    
    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .5)

    pyglet.app.run()
    
    
    
    #tape = Tape(100, 100, 200, 50, 6, 30, 90, tape_unit=TapeUnit.FEET, horizontal=True)
    #tape.draw(299)
        