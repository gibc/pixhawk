import pyglet
from pyglet import shapes

class GPS_Window():
    def __init__(self, pyglet_window, compass_width):
        rect_width = 200
        rect_ht = 50
        x_pos = pyglet_window._x + compass_width/2 - rect_width/2
        y_pos = pyglet_window._y
        
        self.border_rect = shapes.BorderedRectangle(x_pos, y_pos, rect_width, rect_ht,
                                                    border=10, color = (0,0, 0),  border_color = (255,255,255))

    def draw(self, fix, track, commpass_heading, alt, speed):
        self.border_rect.draw()

if __name__ == '__main__':

    window = pyglet.window.Window(1000,700)

    gps_win = GPS_Window(window, 1000/2)

    def on_draw():
        window.clear()
        #rect.draw()
        gps_win.draw(0,0,0,0,5)
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    window.on_draw = on_draw

    pyglet.clock.schedule_interval(update, .1)
    
    pyglet.app.run()