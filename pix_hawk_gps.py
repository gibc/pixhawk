import pyglet
from pyglet import shapes

#from pix_hawk_util import Global

class GPS_Window():
    def __init__(self, pyglet_window, compass_width, gps_manager = None):
        self.gps_manager = gps_manager
        rect_width = 600
        rect_ht = 130
        x_pos = pyglet_window._x + compass_width/2 - rect_width/2
        y_pos = pyglet_window._y
        
        self.border_rect = shapes.BorderedRectangle(x_pos, y_pos, rect_width, rect_ht,
                                                    border=5, color = (0, 0, 0),  border_color = (255,255,255))
        self.border_rect.opacity = 255

        label_width = 150

        self.gps_name = pyglet.text.Label('GPS:',
                          font_size=40,
                          x=self.border_rect.x + self.border_rect.width/2,
                          y=self.border_rect.y + self.border_rect.height -10,
                          color=(255,255,0,255),
                          anchor_y='bottom', anchor_x='right')

        self.gps_error = pyglet.text.Label('ero',
                          font_size=40,
                          x=self.gps_name.x,
                          y=self.border_rect.y + self.border_rect.height -10,
                          anchor_y='bottom', anchor_x='left')

        self.fix_name = pyglet.text.Label('fix:',
                          font_size=40,
                          x=self.border_rect.x+label_width,
                          y=self.border_rect.y + self.border_rect.height/2,
                          color=(255,255,0,255),
                          anchor_y='bottom', anchor_x='right')

        self.fix_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.fix_name.x,
                          y=self.border_rect.y + self.border_rect.height/2,
                          width = 120,
                          anchor_y='bottom', anchor_x='left')

        
        self.track_name = pyglet.text.Label('track:',
                          font_size=34,
                          x=self.fix_name.x,
                          y=self.border_rect.y,
                          color=(255,255,0,255),
                          anchor_y='bottom', anchor_x='right')

        
        self.track_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.track_name.x,
                          y=self.border_rect.y,
                          anchor_y='bottom', anchor_x='left')

        self.speed_name = pyglet.text.Label('climb:',
                          font_size=35,
                          x=self.fix_label.x+ 2*label_width,
                          y=self.border_rect.y ,
                          color=(255,255,0,255),
                          anchor_y='bottom', anchor_x='right')

        self.speed_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.track_label.x + 2*label_width,
                          y=self.border_rect.y,
                          anchor_y='bottom', anchor_x='left')

        self.alt_name = pyglet.text.Label('alt:',
                          font_size=40,
                          x=self.fix_label.x+ 2*label_width,
                          y=self.border_rect.y + self.border_rect.height/2,
                          color=(255,255,0,255),
                          anchor_y='bottom', anchor_x='right')

        self.alt_label = pyglet.text.Label('****',
                          font_size=45,
                          x=self.alt_name.x,
                          y=self.border_rect.y + self.border_rect.height/2,
                          anchor_y='bottom', anchor_x='left')

    def draw(self, fix, track, speed, commpass_heading, alt):
        gps_type = 'xx'
        gps_climb = None
        if self.gps_manager != None:
            #if self.gps_manager.get_listener() != None:
            #    self.border_rect.color=(0,255,255)
            gps_lsn = self.gps_manager.get_listener()
            if gps_lsn != None:
                fix = gps_lsn.fix
                track = gps_lsn.track
                speed = gps_lsn.speed
                alt = gps_lsn.altitude
                gps_type = gps_lsn.type
                gps_climb = gps_lsn.climb

        #elif Global.get_gps_listener() != None:
            #self.border_rect.color=(0,255,0)
            #gps_lsn = Global.get_gps_listener()
            #fix = gps_lsn.fix
            #track = gps_lsn.track
            #speed = gps_lsn.speed
            #alt = gps_lsn.altitude
            #gps_type = gps_lsn.type
            
        #elif fix < 3:
        #    self.border_rect.color=(255,0,0)
        #else:
        #    self.border_rect.color=(0,0,0)

        if gps_type == 'hg':
            self.border_rect.color=(94,101,201)
        if gps_type == 'sb':
            self.border_rect.color=(0,0,255)
        elif gps_type == 'px':
            self.border_rect.color=(0,255,0)
        elif gps_type == 'dg':
            self.border_rect.color=(205,133,63)
        elif gps_type == 'xx':
            self.border_rect.color=(255,0,0)

        self.border_rect.draw()

        self.fix_label.text = gps_type +' '+ str(fix)
        self.fix_label.draw()
        self.fix_name.draw()

        self.track_label.text = str(int(track))
        self.track_label.draw()
        self.track_name.draw()

        if(gps_climb != None):
            self.speed_label.text = str(gps_climb)
            self.speed_name.draw()
            self.speed_label.draw()

        self.alt_label.text = str(int(alt))
        self.alt_name.draw()
        self.alt_label.draw()

        self.gps_name.draw()
        self.gps_error.text = '(t-h)' + str(int(track-commpass_heading))
        self.gps_error.draw()

        #self.border_rect.draw()




if __name__ == '__main__':

    window = pyglet.window.Window(1500,700)
    #window = pyglet.window.Window(fullscreen=True)
        
    gps_win = GPS_Window(window, 1500/2)

    def on_draw():
        window.clear()
        #rect.draw()
        gps_win.draw(3.0, 111.1, 105.333, 100.0, 6000)
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    window.on_draw = on_draw

    pyglet.clock.schedule_interval(update, .1)
    
    pyglet.app.run()