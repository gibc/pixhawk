
from numpy import float32
from pymavlink.generator.mavgen_objc import camel_case_from_underscores
from pix_hawk_util import Math
import pyglet
from pyglet import shapes
from math import sin, cos, radians, fmod

class RollGague():
    
    def __init__(self, pyglet_window, compass_height):
        self.pyglet_window = pyglet_window
        self.compass_height = compass_height

        self.roll_tick_labels = []
        self.roll_tick_lines = []
        
        self.center_x=self.pyglet_window.width/2
        self.center_y=self.pyglet_window.height/2
        self.horz_y = self.center_y
        self.roll_top = self.pyglet_window.height - self.compass_height
        self.tick_length = 40
        self.arc_radius = 300
        self.arc_center = self.roll_top - self.arc_radius
        self.roll_arc = shapes.Arc(self.center_x, self.arc_center, self.arc_radius, angle=radians(120), start_angle=radians(30))
        self.roll_line = shapes.Line(self.center_x, 0, self.center_x, 0, 16, color = (0,255,100))
    
        self.roll_label = pyglet.text.Label(' roll: ',
                          font_size=40,
                          x=self.center_x,
                          y=self.roll_top - 100,
                          anchor_x='center',
                          anchor_y='top')

        self.pitch_label = pyglet.text.Label('pitch: ',
                          font_size=40,
                          x=self.center_x+105,
                          y= self.pyglet_window.height // 2,
                          anchor_x='left',
                          anchor_y='center')

        self.horz_rect_size = 1000                  

        self.top_rect = shapes.Rectangle(self.center_x, self.center_y, self.horz_rect_size, self.horz_rect_size,  color = (0, 0, 255))
        self.top_rect.anchor_x = self.horz_rect_size/2
        self.top_rect.anchor_y = 0

        self.bot_rect = shapes.Rectangle(self.center_x, self.center_y, self.horz_rect_size, self.horz_rect_size,  color = (139,69,19))
        self.bot_rect.anchor_x = self.horz_rect_size/2
        self.bot_rect.anchor_y = self.horz_rect_size

        self.pitch_scale = (self.pyglet_window.height/2) / 90.0
        self.pitch_5_y = int(self.pitch_scale * 5)

        self.pitch_tick_list = []
        self.line_width = 2

        self.nose_line1 = shapes.Line(self.center_x-100, self.center_y , self.center_x-25, self.center_y, 2*self.line_width, color = (0,255,100))
        self.nose_line2 = shapes.Line(self.center_x+25, self.center_y , self.center_x+100, self.center_y, 2*self.line_width, color = (0,255,100))
    
        self.nose_line3 = shapes.Line(self.center_x-25, self.center_y , self.center_x, self.center_y-15, 2*self.line_width, color = (0,255,100))
        self.nose_line4 = shapes.Line(self.center_x+25, self.center_y , self.center_x, self.center_y-15, 2*self.line_width, color = (0,255,100))
    
        self.set_up()

    #draw_pitch_ticks(pitch_tick_list, ahdata.roll, (center_x, center_y), pitch_5_y, 50, -pitch_y)
    def draw_pitch_ticks(self, rotate, pitch):
        pitch = -pitch
        center = (self.center_x, self.horz_y)
        interval = self.pitch_5_y
        length = 50
        idx = 0
        offset = len(self.pitch_tick_list)*interval
        offset = center[1] + offset/2
        offset = offset+pitch
        for line in self.pitch_tick_list:
            pt1 = Math.rotate_point((center[0]-length/2, offset-interval*idx), rotate, center)
            pt2 = Math.rotate_point((center[0]+length/2, offset-interval*idx), rotate, center)
            line.position = pt2[0], pt2[1], pt1[0], pt1[1]
            #line.color = (0,0,0)
            line.draw()
            idx = idx+1
            if idx == len(self.pitch_tick_list)/2:
                idx = idx+1

    
    def make_roll_tick(self, angle, length, start_x, start_y, center_x, center_y):

        width = 2
    
        pt = Math.rotate_point((start_x, start_y), angle, center_point=(center_x, center_y))
        pta = Math.rotate_point((start_x, start_y-length), angle, center_point=(center_x, center_y))
        self.roll_tick_lines.append(pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255)))
                
        if angle > 0:
            self.roll_tick_labels.append(pyglet.text.Label(str(angle),font_size=40, x=pt[0], y=pt[1], anchor_x='right',anchor_y='bottom'))
        elif angle == 0:
            self.roll_tick_labels.append(pyglet.text.Label(str(angle),font_size=40, x=pt[0], y=pt[1]-3, anchor_x='center',anchor_y='bottom'))
        else:
            angle = abs(angle)
            self.roll_tick_labels.append(pyglet.text.Label(str(angle),font_size=40, x=pt[0], y=pt[1], anchor_x='left',anchor_y='bottom'))

        
    def set_up(self):

        self.make_roll_tick(60, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(45, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(30, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(15, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(0, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        
        self.make_roll_tick(-60, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(-45, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(-30, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)
        self.make_roll_tick(-15, self.tick_length, self.center_x, self.roll_top, self.center_x, self.arc_center)

        for i in range(1,3):
            self.pitch_tick_list.append(shapes.Line(self.center_x-30, self.center_y+i*self.pitch_5_y , self.center_x+30, 
                self.center_y+i*self.pitch_5_y, self.line_width, color = (255,255,255)))
            #self.pitch_tick_list.append(shapes.Line(center_x-30, center_y+2*pitch_5_y , center_x+30, center_y+2*pitch_5_y , width, color = (255,255,255)))
            #self.pitch_tick_list.append(shapes.Line(center_x-30, center_y+3*pitch_5_y , center_x+30, center_y+3*pitch_5_y , width, color = (255,255,255)))
        
            self.pitch_tick_list.append(shapes.Line(self.center_x-30, self.center_y-i*self.pitch_5_y , self.center_x+30, 
                self.center_y-self.pitch_5_y, self.line_width, color = (255,255,255)))
            #self.pitch_tick_list.append(shapes.Line(center_x-30, center_y-2*pitch_5_y , center_x+30, center_y-2*pitch_5_y , width, color = (255,255,255)))
            #self.pitch_tick_list.append(shapes.Line(center_x-30, center_y-3*pitch_5_y , center_x+30, center_y-3*pitch_5_y , width, color = (255,255,255)))
        
        
        
                    

    def draw(self, roll, pitch):

        pitch = float32(pitch)
        pitch_y = pitch*self.pitch_scale

        self.top_rect.rotation = -roll
        self.top_rect.position = (self.center_x, self.horz_y-pitch_y)
        self.top_rect.draw()
    
        self.bot_rect.rotation = -roll
        self.bot_rect.position = (self.center_x, self.horz_y-pitch_y)
        self.bot_rect.draw()

        for lnl in self.roll_tick_lines:
            lnl.draw()

        for lbl in self.roll_tick_labels:
            lbl.draw()

        self.roll_arc.draw()

        pt1 = Math.rotate_point((self.center_x, self.roll_top), -roll, center_point=(self.center_x, self.arc_center))
        pt2 = Math.rotate_point((self.center_x, self.roll_top-100), -roll, center_point=(self.center_x, self.arc_center))
        self.roll_line.position = pt2[0], pt2[1], pt1[0], pt1[1]
        self.roll_line.draw()
        self.roll_label.text = str(round(abs(roll), 1)) 
        self.roll_label.draw()

        self.draw_pitch_ticks(roll, pitch_y)
        self.pitch_label.text = str(round(pitch, 1))
        self.pitch_label.draw()

        self.nose_line1.draw()
        self.nose_line2.draw()
        self.nose_line3.draw()
        self.nose_line4.draw()
    
if __name__ == '__main__':
    # unit test code

    
    window = pyglet.window.Window(900,900)

    rg = RollGague(window, 150)


    def update(dt):
        x=0

    def on_draw():
        window.clear()
        rg.draw(0,0)
        
     
    window.on_draw = on_draw

    pyglet.clock.schedule_interval(update, 100)
    

    pyglet.app.run()

    