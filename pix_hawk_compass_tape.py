from re import S
import pyglet
from pyglet import shapes
from pix_hawk_tape import Tape
from pix_hawk_tape import Align
from pix_hawk_tape import Orient
from pix_hawk_tape import TapeUnit

class CompassTape(Tape):

    def __init__(self, pyglet_window, x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.LEFT, orient=Orient.HORZ):
        print("CompassTape init")
        super().__init__(x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.CENTER, tape_unit=TapeUnit.DEGREE, orient=Orient.HORZ)

        # locate tape at upper left conner of parent window
        self.current_val_rect.height = self.current_val_rect.height+12
        self.current_val_rect.width = self.current_val_rect.width+48

        self.border_rect.height = self.current_val_rect.height
        self.border_rect.width = pyglet_window.width/2

        self.border_rect.x = pyglet_window._x
        self.border_rect.y = pyglet_window._y + pyglet_window.height - self.border_rect.height


        self.current_val_rect.x = self.border_rect.x  + self.border_rect.width/2 - self.current_val_rect.width/2
        self.current_val_rect.y = self.border_rect.y

        self.current_val_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.current_val_rect.x,
                          y=self.current_val_rect.y,
                          anchor_y='bottom', anchor_x='left')

        self.tick_labels = []        
        for i in range(tick_count+1): 
            self.tick_labels.append(pyglet.text.Label('****',
                          font_size=30,
                          x=self.current_val_rect.x,
                          y=self.current_val_rect.y+8,
                          anchor_y='bottom', anchor_x='center'))

        self.border_rect.color = (0,0,255)

        self.tick_pixels = self.border_rect.width/self.tick_count
        self.units2pix_scale = self.tick_pixels/self.units_interval
 
    

    def draw(self, heading):
        heading = super().round_half_up(heading, decimals=1)
        super().draw(heading)
        return

        heading_origin = self.get_90_origin(heading)

        self.border_rect.draw()
        compass_rect = self.border_rect
        for i in range (self.tick_count):
                
            nxt = self.get_tick_value(heading_origin, i, self.units_interval)
            labl = self.tick_labels[i]
            labl.text = self.get_value_str(nxt)
            org_offset = self.angle_dif_right(heading_origin, nxt)
            labl.x = int(compass_rect.x - compass_rect.width/2) + int(self.units2pix_scale*org_offset)
            labl.color = self.get_value_color(nxt)
            labl.draw()
            
        
        
        self.current_val_rect.draw()
        
        self.current_val_label.text = self.get_value_str(round(abs(heading)))
        
        self.current_val_label.draw()


if __name__ == '__main__':
    # unit test code

    
    def on_draw():
        window.clear()
            #rect.draw()
        tape.draw(mock_heading)
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    mock_heading = 0
    def mock_data(dt):
        global mock_heading
        #mock_heading = 0
        return

        mock_heading += 1
        if(mock_heading >360):
            mock_heading = 0

    

    window = pyglet.window.Window(1500,700)
    window.on_draw = on_draw
    center_x = window.width/2
    center_y = window.height/2
    #rect = shapes.BorderedRectangle(center_x, center_y,  100, 100, border=3, color = (0, 0, 255),
                                            #border_color = (255,255,255))
    tape = CompassTape(window, 150, 200, 800, 65, 6, 30, align=Align.LEFT, orient=Orient.VERT)
    #tape = Tape(50, 100, 500, 75, 6, 100, align=Align.RIGHT, tape_unit=TapeUnit.FEET_VERT_SPEED, orient=Orient.VERT)
    
    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .5)


    pyglet.app.run()
    