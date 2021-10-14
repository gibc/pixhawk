import pyglet
from pyglet import shapes
from pix_hawk_tape import Tape
from pix_hawk_tape import Align
from pix_hawk_tape import Orient
from pix_hawk_tape import TapeUnit

class SpeedTape(Tape):

    def __init__(self, x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=Align.LEFT, orient=Orient.HORZ):
        super().__init__(x, y, pixel_wd, pixel_ht, tick_count, units_interval, align=align, tape_unit=TapeUnit.FEET_ALT, orient=orient)
        
        
        self.grd_speed_val_label = pyglet.text.Label('****',
                          font_size=40,
                          x=self.current_val_rect.x,
                          y=self.border_rect.y+self.border_rect.height+1,
                          anchor_y='bottom', anchor_x='left')
        
        self.grd_speed_val_rect = shapes.BorderedRectangle(self.grd_speed_val_label.x, self.grd_speed_val_label.y, self.current_val_rect.width,
                                                       self.current_val_rect.height, border=10, color = (0, 0, 0),
                                                       border_color = (255,255,255))
        #self.pix2climb = self.border_rect.height/2 / 1000
        
        #self.up_rect = shapes.Rectangle(self.x-22,self.y+self.border_rect.height/2.,20,200, (0,255,255))
    
        
    def draw(self, air_speed, gnd_speed):
        super().draw(air_speed)
        print('gnd_speed', str(gnd_speed))
        self.grd_speed_val_label.text = str(gnd_speed)
        self.grd_speed_val_rect.draw()
        
        """
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
        """
        self.grd_speed_val_label.draw()
        #self.up_rect.draw()
    
if __name__ == '__main__':
    # unit test code
    
    print('__main__')
    mock_speed = 0
    mock_speed_delta = 2
    mock_gnd_speed = 0
    mock_gnd_speed_delta = 4
    def on_draw():
        window.clear()
        #rect.draw()
        tape.draw(mock_speed, mock_gnd_speed)
    
    def update(dt):
        x=0
        #print("dt: ", dt)
        
    def mock_data(dt):
        
        print('mock_data')
        global mock_speed
        global mock_speed_delta
        global mock_gnd_speed
        global mock_gnd_speed_delta
        #global climb_delta

            
        #if mock_units == TapeUnit.FEET_ALT:
        mock_speed = mock_speed + mock_speed_delta
                
        if mock_speed > 160:
            mock_speed = 160
            mock_speed_delta = -2
        if mock_speed < 0:
            mock_speed = 0
            mock_speed_delta = 2
            
        mock_gnd_speed = mock_gnd_speed + mock_gnd_speed_delta
        if mock_gnd_speed > 150:
            mock_gnd_speed = 150
            mock_gnd_speed_delta = -4
        if mock_gnd_speed < 0:
            mock_gnd_speed = 0
            mock_gnd_speed_delta = 4

        
    window = pyglet.window.Window(1000,700)
    window.on_draw = on_draw
    center_x = window.width/2
    center_y = window.height/2
    #rect = shapes.BorderedRectangle(center_x, center_y,  100, 100, border=3, color = (0, 0, 255),
                                            #border_color = (255,255,255))
    tape = SpeedTape(150, 100, 500, 45, 6, 100, align=Align.LEFT, orient=Orient.VERT)
    #tape = Tape(50, 100, 500, 75, 6, 100, align=Align.RIGHT, tape_unit=TapeUnit.FEET_VERT_SPEED, orient=Orient.VERT)
    
    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .5)

    pyglet.app.run()
