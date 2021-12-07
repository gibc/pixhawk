
from pix_hawk_util import Math
import pyglet
from pyglet import shapes
from math import sin, cos, radians, fmod

class RollGague():
    
    def __init__(self, xpos, ypos, width, height):
        self.roll_tick_labels = []
        self.roll_tick_lines = []
        self.xpos = xpos
        self.ypos = ypos
        self.width = width
        self.height = height
        self.center_x=xpos + width / 2
        self.center_y=ypos + height / 2
        self.roll_top = self.ypos+self.height #win_rect.height+win_rect.y - compass_height - 40
        self.tick_length = 40
        self.arc_radius = 300
        self.arc_center = self.roll_top - self.arc_radius
        self.roll_arc = shapes.Arc(self.center_x, self.arc_center, self.arc_radius, angle=radians(120), start_angle=radians(30))
        self.roll_line = shapes.Line(self.center_x, self.ypos, self.center_x, self.ypos+self.height, 16, color = (0,255,100))
    
        self.roll_label = pyglet.text.Label(' roll: ',
                          font_size=40,
                          x=self.center_x,
                          y=self.center_y,
                          anchor_x='center',
                          anchor_y='top')

        
        self.set_up()

    
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
                    

    def draw(self, roll):

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
    
if __name__ == '__main__':
    # unit test code

    rg = RollGague(300, 150, 200, 200)

    window = pyglet.window.Window(900,900)

    def update(dt):
        x=0

    def on_draw():
        window.clear()
        rg.draw(0)
        
     
    window.on_draw = on_draw

    pyglet.clock.schedule_interval(update, 100)
    

    pyglet.app.run()

    