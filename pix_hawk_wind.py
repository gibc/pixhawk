import pyglet
from pyglet import clock
from pyglet import shapes
import numpy as np

class Wind():
    
    

    def cart2pol(self, x, y):
        #print('cart2pol x', x)
        #print('cart2pol y', y)
        rho = np.sqrt(x**2 + y**2)
        #phi = np.arctan2(y, x)
        phi = np.arctan2(x, y)
        #print('cart2pol mag', rho)
        
        ang = np.degrees(phi)
        #print('cart2pol ang', ang)
        """
        ang = ang - 90
        print('cart2pol ang', ang)
        if ang < 0:
            ang = 360 + ang
        """
        #print('cart2pol ang', ang)
        #print()
        return(rho, ang)

    def pol2cart(self, rho, phi):
        print('pol2cart rho', rho)
        print('pol2cart ang degrees', phi)
        #convert to 90 origin going in
        #phi = phi + 90
        #if phi > 360:
        #    phi = phi - 360
        #print('pol2cart ang degrees', phi)
        phi = np.radians(phi)
        #print('pol2cart phi radians', phi)
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        #x = rho * np.cos(phi)
        #y = rho * np.sin(phi)
        #print('pol2cart x', x)
        #print('pol2cart y', y)
        #print()
        return(x, y)
    
    def calulateWind(self, airspeed, heading, gndspeed, track, altitude):
        """
         air_vec + wind_vec = gnd_vec
         wind_vec = gnd_vec - air_vec
        
        
        print('airspeed ', airspeed)
        print('heading ', heading)
        print('gndspeed ', gndspeed)
        print('track ', track)
        """
        alt_adj = altitude / 1000 # alt in thousands
        alt_adj = (alt_adj * .02) # alt pec adj at 2% percent per 1000ft
        alt_adj = alt_adj + 1 # alt adj factor > 1
        airspeed = airspeed * alt_adj
        
        gnd_vec = self.pol2cart(gndspeed, track)
        air_vec = self.pol2cart(airspeed, heading,)
        
        #wind_x = gnd_vec[0] - air_vec[0]
        #wind_y = gnd_vec[1] - air_vec[1]
        wind_x = air_vec[0] - gnd_vec[0]
        wind_y = air_vec[1] - gnd_vec[1]
        
        wind_vec = self.cart2pol(wind_x, wind_y)
        ang = wind_vec[1]
        mag = wind_vec[0]
        ang = 90 - ang
        
        if ang > 360:
            ang = ang - 360
        if ang < 0:
            ang = 360 + ang
        
        
        return (ang, mag)
    
    def get_wind_comps(self, heading, wind_speed, wind_dir):
        plane_wind_dir = heading - wind_dir
        print('heading ', heading)
        print('wind_dir ', wind_dir)
        if plane_wind_dir < 0:
            plane_wind_dir = 360 + plane_wind_dir
        print('plane_wind_dir ', plane_wind_dir)
        
        wind_comps = self.pol2cart(wind_speed, plane_wind_dir)
        wind_comps = (wind_comps[1], wind_comps[0])
        return wind_comps
        
    def set_wind_comps_text(self, heading, wind_speed, wind_dir):
        wind_comps = self.get_wind_comps(heading, wind_speed, wind_dir)
        
        self.head_wind_label.text = 'h:' + str(abs(int(round(wind_comps[1]))))
        self.head_wind_label.color = (0,255,0,255)
        if wind_comps[1] < 0:
            self.head_wind_label.text = 't:' + str(abs(int(round(wind_comps[1]))))
            self.head_wind_label.color = (255,0,0,255)
            
        self.head_wind_label.draw()
        self.x_wind_label.text = '<:' + str(abs(int(round(wind_comps[0]))))
        if wind_comps[0] < 0:
            self.x_wind_label.text = '>:' + str(abs(int(round(wind_comps[0]))))
        self.x_wind_label.color = (255,255,255,255)
        if(abs(int(round(wind_comps[0])))) >= 5:
            self.x_wind_label.color = (255,0,0,255)
           
        self.x_wind_label.draw()
        
    
    
    def __init__(self, x, y, wd, ht):
        self.x = x
        self.y = y
        self.ht = ht
        self.wd = wd
        
        
        self.wind_rect = shapes.BorderedRectangle(self.x, self.y, self.wd,
                                                       self.ht, border=10, color = (0,0,0),
                                                       border_color = (255,255,255))
        self.wind_speed_label = pyglet.text.Label('wind speed',
                          font_size=50,
                          x=self.wind_rect.x+200,
                          y=self.wind_rect.y+3*self.ht/4,
                          anchor_y='center', anchor_x='right')
        self.wind_dir_label = pyglet.text.Label('wind_dir',
                          font_size=50,
                          x=self.wind_rect.x+200,
                          y=self.wind_rect.y+3*self.ht/4,
                          anchor_y='center', anchor_x='left')
        self.head_wind_label = pyglet.text.Label('wind_dir',
                          font_size=50,
                          x=self.wind_rect.x,
                          y=self.wind_rect.y+self.ht/4,
                          anchor_y='center', anchor_x='left')
        self.x_wind_label = pyglet.text.Label('wind_dir',
                          font_size=50,
                          x=self.wind_rect.x+180,
                          y=self.wind_rect.y+self.ht/4,
                          anchor_y='center', anchor_x='left')
        
    def draw(self, airspeed, heading, gnd_speed, gnd_track, msg_wind_speed, msg_wind_dir):
        self.wind_rect.draw()
        
        self.wind_speed_label.text = str(msg_wind_speed) + "@"
        self.wind_speed_label.draw()
        self.wind_dir_label.text = str(round(msg_wind_dir))
        self.wind_dir_label.draw()
    
        wind = self.calulateWind(airspeed, heading, gnd_speed, gnd_track, 0)
        print('wind ang', wind[0])
        print('wind speed', wind[1])
        print()
        
        wind_comps = self.get_wind_comps(heading, msg_wind_speed, msg_wind_dir)
        
        """
        self.head_wind_label.text = 'h:' + str(int(round(wind_comps[1])))
        self.head_wind_label.draw()
        self.x_wind_label.text = 'x:' + str(int(round(wind_comps[0])))
        self.x_wind_label.draw()
        """
        self.set_wind_comps_text(heading, msg_wind_speed, msg_wind_dir)
        
    def draw_msg(self, msg_wind_speed, msg_wind_dir, heading):
        self.wind_rect.draw()
        self.wind_speed_label.text = str(int(round(msg_wind_speed))) + "@"
        self.wind_speed_label.draw()
        self.wind_dir_label.text = str(round(msg_wind_dir))
        self.wind_dir_label.draw()
        
        wind_comps = self.get_wind_comps(heading, msg_wind_speed, msg_wind_dir)
        """
        self.head_wind_label.text = 'h:' + str(int(round(wind_comps[1])))
        self.head_wind_label.draw()
        self.x_wind_label.text = 'x:' + str(int(round(wind_comps[0])))
        self.x_wind_label.draw()
        """
        self.set_wind_comps_text(heading, msg_wind_speed, msg_wind_dir)
    
    def draw_calc(self, airspeed, heading, gnd_speed, gnd_track, altitude):
        wind = self.calulateWind(airspeed, heading, gnd_speed, gnd_track, altitude)
        #print('wind ang', wind[0])
        #print('wind speed', wind[1])
        
        
        self.wind_rect.draw()
        self.wind_speed_label.text = str(int(round(wind[0]))) + "@"
        self.wind_speed_label.draw()
        self.wind_dir_label.text = str(int(round(wind[1])))
        self.wind_dir_label.draw()
        """
        wind_comps = self.get_wind_comps(heading, wind[0], wind[1])        
        self.head_wind_label.text = 'h:' + str(int(round(wind_comps[1])))
        self.head_wind_label.draw()
        self.x_wind_label.text = 'x:' + str(int(round(wind_comps[0])))
        self.x_wind_label.draw()
        """
        
        self.set_wind_comps_text(heading, wind[1], wind[0])
        
        
if __name__ == '__main__':
    # unit test code
    
    print('__main__')
    mock_wind = 0
    mock_wind_delta = 2
    mock_dir = 0
    mock_dir_delta = 1.5
    
    mock_heading = 0
    mock_heading_delta = 10
    mock_track = 180
    mock_track_delta = -10
    mock_gnd_speed = 100
    mock_gnd_speed_delta = 2
    mock_air_speed = 50
    mock_air_speed_delta = -2
    
    
    def on_draw():
        global mock_wind
        global mock_dir
        global mock_heading
        global mock_track
        global mock_gnd_speed
        global mock_air_speed
        
        
        window.clear()
        #rect.draw()
        #wind.draw(mock_air_speed, mock_heading, mock_gnd_speed, mock_track, mock_wind, mock_dir)
        wind.draw(85, 355, 95, 345, mock_wind, mock_dir)
    
    def update(dt):
        x=0
        #print("dt: ", dt)
    
    def mock_gnd_air(dt):
        #print('mock_gnd_air')
        global mock_heading
        global mock_heading_delta
        global mock_track
        global mock_track_delta
        global mock_gnd_speed
        global mock_gnd_speed_delta
        global mock_air_speed
        global mock_air_speed_delta
        
        mock_heading += mock_heading_delta
        if mock_heading > 360:
            mock_heading = 360
            mock_heading_delta = -10
        if mock_heading < 0:
            mock_heading = 0
            mock_heading_delta = 10
        
        mock_track += mock_track_delta
        if mock_track > 360:
            mock_track = 360
            mock_track_delta = -8
        if mock_track < 0:
            mock_track = 0
            mock_track_delta = 8
            
        mock_gnd_speed += mock_gnd_speed_delta
        if mock_gnd_speed > 150:
            mock_gnd_speed = 150
            mock_gnd_speed_delta = -2
        if mock_gnd_speed < 50:
            mock_gnd_speed = 50
            mock_gnd_speed_delta = 2
        
        mock_air_speed += mock_air_speed_delta
        if mock_air_speed > 150:
            mock_air_speed = 150
            mock_air_speed_delta = -2
        if mock_air_speed < 50:
            mock_air_speed = 50
            mock_air_speed_delta = 2
        
    def mock_data(dt):
        
        #print('mock_data')
        global mock_wind
        global mock_wind_delta
        global mock_dir
        global mock_dir_delta
        
        mock_wind =  mock_wind + mock_wind_delta
                
        if mock_wind > 40:
            mock_wind = 40
            mock_wind_delta = -2
        if mock_wind < 0:
            mock_wind = 0
            mock_wind_delta = 2
            
        mock_dir = mock_dir + mock_dir_delta
        if mock_dir > 360:
            mock_dir = 360
            mock_dir_delta = -2.5
        if mock_dir < 0:
            mock_dir = 0.0
            mock_dir_delta = 2.5

        
    window = pyglet.window.Window(1000,700)
    """
    window.on_draw = on_draw
    """
    center_x = window.width/2
    center_y = window.height/2
    
    wind = Wind(100, 100, 350, 160)
    
    """
    #draw(airspeed, heading, gnd_speed, gnd_track, msg_wind_speed, msg_wind_dir)
    wind.draw(85, 355, 95, 345,   0, 0)
    wind.draw(100, 99, 120, 44,   0, 0)
    wind.draw(75,  97, 70,  85,   0, 0)
    wind.draw(100, 180,111, 180,  0, 0)
    wind.draw(100, 95, 100, 90,   0, 0)
    """
    """
    #get_wind_comps(heading, wind_speed, wind_dir)
    comps = wind.get_wind_comps(90, 20, 270)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(90, 20, 90)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(180, 20, 360)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(180, 20, 180)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(45, 20, 45)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(45, 20, 180+45)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(90, 20, 180)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(90, 20, 0)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(90, 20, 45)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    comps = wind.get_wind_comps(90, 20, 45+180)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    """
    comps = wind.get_wind_comps(307, 2, 295)
    print('x', round(comps[0]))
    print('y', round(comps[1]))
    print()
    
    
    pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(mock_data, .5)
    pyglet.clock.schedule_interval(mock_gnd_air, .5)

    pyglet.app.run()
        