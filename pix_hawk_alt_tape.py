import pyglet
from pyglet import shapes
from pix_hawk_tape import Tape
from pix_hawk_tape import Align
from pix_hawk_tape import Orient
from pix_hawk_tape import TapeUnit

class AltTape(Tape):

    def __init__(self, x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.LEFT, orient=Orient.HORZ):
        super().__init__(x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=align, tape_unit=TapeUnit.FEET_ALT, orient=orient)
        
        
        self.climb_val_label = pyglet.text.Label('****',
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
        print('climb', str(climb))
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
    
if __name__ == '__main__':
    # unit test code
    
    print('__main__')
    mock_angle = 5000
    mock_delta = 2
    climb = 0
    climb_delta = 100
    def on_draw():
        window.clear()
        #rect.draw()
        tape.draw(mock_angle, climb)
    
    def update(dt):
        x=0
        #print("dt: ", dt)
        
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

        
    window = pyglet.window.Window(1000,700)
    window.on_draw = on_draw
    center_x = window.width/2
    center_y = window.height/2
    #rect = shapes.BorderedRectangle(center_x, center_y,  100, 100, border=3, color = (0, 0, 255),
                                            #border_color = (255,255,255))
    tape = AltTape(150, 100, 500, 55, 6, 100, align=Align.LEFT, orient=Orient.VERT)
    #tape = Tape(50, 100, 500, 75, 6, 100, align=Align.RIGHT, tape_unit=TapeUnit.FEET_VERT_SPEED, orient=Orient.VERT)
    
    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .5)

    pyglet.app.run()