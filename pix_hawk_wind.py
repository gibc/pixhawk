import pyglet
from pyglet import clock
from pyglet import shapes
import numpy as np

class Arrow():
    def __init__(self, alen):
        self.ln_len = alen
        self.x_arrow_line = pyglet.shapes.Line(0,0, 0,0, 5)
        self.x_arrow_hd1_line = pyglet.shapes.Line(0,0, 0,0, 5)
        self.x_arrow_hd2_line = pyglet.shapes.Line(0,0, 0,0, 5)
                
    def set_xwind_color(self, comp):
            if abs(comp) > 5:
                self.x_arrow_line.color=(255,0,0)
                self.x_arrow_hd1_line.color=(255,0,0)
                self.x_arrow_hd2_line.color=(255,0,0)
            else:
                self.x_arrow_line.color=(255,255,255)
                self.x_arrow_hd1_line.color=(255,255,255)
                self.x_arrow_hd2_line.color=(255,255,255)
            
    def set_hwnd_color(self, comp):
            if comp >= 0:
                self.x_arrow_line.color=(0,255,0)
                self.x_arrow_hd1_line.color=(0,255,0)
                self.x_arrow_hd2_line.color=(0,255,0)
            else:
                self.x_arrow_line.color=(255,0,0)
                self.x_arrow_hd1_line.color=(255,0,0)
                self.x_arrow_hd2_line.color=(255,0,0)
            
    def draw_left(self,x,y,comp):
        self.set_xwind_color(comp)
        self.x_arrow_line.position =  (x,y, x+self.ln_len,y)
        self.x_arrow_line.draw()
            
        self.x_arrow_hd1_line.position =  x,y,x+20,y+15
        self.x_arrow_hd1_line.draw()
        self.x_arrow_hd2_line.position =  x,y,x+20,y-15
        self.x_arrow_hd2_line.draw()
            
            
    def draw_right(self,x,y,comp):
        self.set_xwind_color(comp)
        self.x_arrow_line.position =  (x,y, x+self.ln_len, y)
        self.x_arrow_line.draw()
               
        self.x_arrow_hd1_line.position =  (x+self.ln_len,y,x+self.ln_len-20,y+15)
        self.x_arrow_hd1_line.draw()
        self.x_arrow_hd2_line.position =  (x+self.ln_len,y,x+self.ln_len-20,y-15)
        self.x_arrow_hd2_line.draw()
            
    def draw_up(self,x,y,comp):
        self.set_hwnd_color(comp)
        self.x_arrow_line.position =  x,y,x,y+self.ln_len
        self.x_arrow_line.draw()
            
        self.x_arrow_hd1_line.position =  x,y+self.ln_len,x-15,y+self.ln_len-20
        self.x_arrow_hd1_line.draw()
        self.x_arrow_hd2_line.position =  x,y+self.ln_len,x+15,y+self.ln_len-20
        self.x_arrow_hd2_line.draw()
            
            
    def draw_down(self,x,y,comp):
        self.set_hwnd_color(comp)
        self.x_arrow_line.position =  x,y,x,y+self.ln_len
        self.x_arrow_line.draw()
            
        self.x_arrow_hd1_line.position =  x,y,x-15,y+20
        self.x_arrow_hd1_line.draw()
        self.x_arrow_hd2_line.position =  x,y,x+15,y+20
        self.x_arrow_hd2_line.draw()
            
    
class WindComp():
    def __init__(self):
        self.x_wind_label = pyglet.text.Label('wind_dir',
                          x=10,
                          y=10,
                          font_size=50,                    
                          anchor_y='center', anchor_x='left')
        self.arrow = Arrow(85)
        self.font_wd = 45
        
    def get_digits(self, comp):
        #print('\n>>>>>>>>get_digits comp ', comp)
        digits = 0
        comp = abs(int(comp))
        if comp == 0:
            return 1
        if comp < 1.0:
            return digits
        while(True):
            digits += 1
            comp = int(comp/10)
            if comp < 1.0:
                break
        #print('\n>>>>>>>>get_digits digits ', digits)
        return digits
        
    def set_xw_color(self,comp):
            if abs(comp) > 5:
                self.x_wind_label.color = (255,0,0,255)
            else:
                self.x_wind_label.color = (255,255,255,255)
                
    def set_hw_color(self,comp):
            if comp >= 0:
                self.x_wind_label.color = (0, 255,0,255)
            else:
                self.x_wind_label.color = (255,0,0,255)
        
    def draw_left_xw(self,x,y,comp):
        self.set_xw_color(comp)
        digits = self.get_digits(comp)
        bl_wd = digits*self.font_wd
        self.x_wind_label.x = x
        self.x_wind_label.y = y
        self.x_wind_label.text = str(abs(int(round(comp))))
        self.x_wind_label.draw()
        
        self.arrow.draw_left(x+bl_wd,y,comp)

            
    def draw_right_xw(self,x,y,comp):
        self.set_xw_color(comp)
        self.arrow.draw_right(x,y,comp)
        #digits = self.get_digits(comp)
        #lbl_wd = digits*self.font_wd
        self.x_wind_label.x = x+self.arrow.ln_len
        self.x_wind_label.y = y
        self.x_wind_label.text = str(abs(int(round(comp))))
        self.x_wind_label.draw()
        
        digits = self.get_digits(comp)
        
            
    def draw_tail_wind(self,x,y,comp):
        self.set_hw_color(comp)
        self.x_wind_label.x = x+10
        self.x_wind_label.y = y
        self.x_wind_label.text = str(abs(int(round(comp))))
        self.arrow.draw_up(x,y-self.arrow.ln_len/2,comp)
        self.x_wind_label.draw()
            
    def draw_head_wind(self,x,y,comp):
        self.set_hw_color(comp)
        self.x_wind_label.x = x+10
        self.x_wind_label.y = y
        self.x_wind_label.text = str(abs(int(round(comp))))
        self.arrow.draw_down(x,y-self.arrow.ln_len/2,comp)
        self.x_wind_label.draw()
            
    

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
        print('pol2cart mag', rho)
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
        print('pol2cart x', x)
        print('pol2cart y', y)
        #print()
        return(x, y)
    
    def calulateWind(self, airspeed, heading, gndspeed, track, altitude):
        """
        print('\n***call calulateWind***')
        print('airspeed ',airspeed)
        print('heading ',heading)
        print('gndspeed ',gndspeed)
        print('track ',track)
        print('altitude ',altitude)
        """
        alt_adj = altitude / 1000 # alt in thousands
        alt_adj = (alt_adj * .02) # alt pec adj at 2% percent per 1000ft
        alt_adj = alt_adj + 1 # alt adj factor > 1
        #print('alt_adj ',alt_adj)
        airspeed = airspeed * alt_adj
        #print('airspeed ',airspeed)
        
        #print('\nget gnd comps')
        gnd_vec = self.pol2cart(gndspeed, track)
        #print('\nget air comps')
        air_vec = self.pol2cart(airspeed, heading,)
        
        #print('\nget wind comps')
        wind_x = air_vec[0] - gnd_vec[0]
        #print('wind_x ', wind_x)
        wind_y = air_vec[1] - gnd_vec[1]
        #print('wind_y ', wind_y)
        
        #print('\nget polar from wind cart')
        wind_vec = self.cart2pol(wind_x, wind_y)
        ang = wind_vec[1]
        #print('wind ang', ang)
        mag = wind_vec[0]
        #print('wind mag', mag)
        
        #print('\nconvert from 90 origin to 360 origin')
        ang = 90 - ang  
        if ang > 360:
            ang = ang - 360
        if ang < 0:
            ang = 360 + ang
        
        """
        print('wind ang', ang)
        print('wind mag', mag)
        print('***exit calulateWind***')
        print()
        """
        return (ang, mag)
    
    def get_wind_comps(self, heading, wind_speed, wind_dir):
        plane_wind_dir = heading - wind_dir
        #print('heading ', heading)
        #print('wind_dir ', wind_dir)
        if plane_wind_dir < 0:
            plane_wind_dir = 360 + plane_wind_dir
        #print('plane_wind_dir ', plane_wind_dir)
    
            
        wind_comps = self.pol2cart(wind_speed, plane_wind_dir)
        wind_comps = (wind_comps[1], wind_comps[0])
        return wind_comps
    
    def set_wind_comps_text(self, heading, wind_speed, wind_dir):
        
        wind_comps = self.get_wind_comps(heading, wind_speed, wind_dir)
        xw = wind_comps[0]
        hw = wind_comps[1]
        x = self.x +10
        y = self.y +50
        if xw < 0:
            self.wind_comp.draw_right_xw(x, y, xw)
        else:
            self.wind_comp.draw_left_xw(x, y, xw)
        
        x = self.x+240
        if hw >0:
            self.wind_comp.draw_head_wind(x, y, hw)
        else:
            self.wind_comp.draw_tail_wind(x, y, hw)
        
        """
        self.head_wind_label.text = 'h:' + str(abs(int(round(wind_comps[1]))))
        self.head_wind_label.color = (0,255,0,255)
        if wind_comps[1] < 0:
            self.head_wind_label.text = 't:' + str(abs(int(round(wind_comps[1]))))
            self.head_wind_label.color = (255,0,0,255)
            
        #self.head_wind_label.draw()
        
        self.draw_xwind(self.wind_rect.x, self.wind_rect.y+40, int(round(wind_comps[0])))
        
        self.x_wind_label.text = '<:' + str(abs(int(round(wind_comps[0]))))
        if wind_comps[0] < 0:
            self.x_wind_label.text = '>:' + str(abs(int(round(wind_comps[0]))))
        self.x_wind_label.color = (255,255,255,255)
        if(abs(int(round(wind_comps[0])))) >= 5:
            self.x_wind_label.color = (255,0,0,255)
           
        self.x_wind_label.draw()
        """
    
    """
    def draw_arrow(self, x, y, len, wind_comp, x_wind=True, head_bgn=True):
        
        if x_wind == True:
            if wind_comp >= 0:
                self.x_arrow_line.color=(255,255,255)
                self.x_arrow_hd1_line.color=(255,255,255)
                self.x_arrow_hd2_line.color=(255,255,255)
            else:
                self.x_arrow_line.color=(255,0,0)
                self.x_arrow_hd1_line.color=(255,0,0)
                self.x_arrow_hd2_line.color=(255,0,0)
                
            self.x_arrow_line.position =  x,y,x+len,y
            self.x_arrow_line.draw()
            if head_bgn:
                self.x_arrow_hd1_line.position =  x,y,x+20,y+15
                self.x_arrow_hd1_line.draw()
                self.x_arrow_hd2_line.position =  x,y,x+20,y-15
                self.x_arrow_hd2_line.draw()
            else:
                self.x_arrow_hd1_line.position =  x+len,y,x+len-20,y+15
                self.x_arrow_hd1_line.draw()
                self.x_arrow_hd2_line.position =  x+len,y,x+len-20,y-15
                self.x_arrow_hd2_line.draw()
        
        else:
            self.x_arrow_line.position =  x,y,x,y+len
            self.x_arrow_line.draw()
            if head_bgn:
                self.x_arrow_hd1_line.position =  x,y+len,x-15,y+len-20
                self.x_arrow_hd1_line.draw()
                self.x_arrow_hd2_line.position =  x,y+len,x+15,y+len-20
                self.x_arrow_hd2_line.draw()
            else:
                self.x_arrow_hd1_line.position =  x,y,x-15,y+20
                self.x_arrow_hd1_line.draw()
                self.x_arrow_hd2_line.position =  x,y,x+15,y+20
                self.x_arrow_hd2_line.draw()
    
    def draw_xwind(self,x, y, x_wind_comp):
        color = (255,255,255,255)
        if x_wind_comp < 0:
            color = (255,0,0,255)
        self.x_wind_label.color = color
            
        self.x_wind_label.text = str(abs(int(round(x_wind_comp))))
        if x_wind_comp >= 0:
            self.x_wind_label.position = (x+150,y)
            if x_wind_comp < 10:
                self.x_wind_label.position = (x+100,y)    
            self.draw_arrow(x,y,100,x_wind_comp,True,False)
        else:
            self.x_wind_label.position = (x,y)
            if abs(x_wind_comp) >= 10:
                self.draw_arrow(x+85,y,100,x_wind_comp,True,True)
            else:
                self.draw_arrow(x+85/2,y,100,x_wind_comp,True,True)
                
        self.x_wind_label.draw()
    """
         
    def __init__(self, x, y, wd, ht):
        self.x = x
        self.y = y
        self.ht = ht
        self.wd = wd
        self.wind_comp = WindComp()
        
        
        self.wind_rect = shapes.BorderedRectangle(self.x, self.y, self.wd,
                                                       self.ht, border=10, color = (0,0,0),
                                                       border_color = (255,255,255))
        self.wind_speed_label = pyglet.text.Label('wind speed',
                          font_size=50,
                          x=self.wind_rect.x+200+20,
                          y=self.wind_rect.y+3*self.ht/4,
                          anchor_y='center', anchor_x='right')
        self.wind_dir_label = pyglet.text.Label('wind_dir',
                          font_size=50,
                          x=self.wind_rect.x+200+20,
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
        self.x_arrow_line = pyglet.shapes.Line(self.wind_rect.x,
                                              self.wind_rect.y+20,
                                              self.wind_rect.x+self.wind_rect.width,
                                              self.wind_rect.y+20,
                                              5, color = (255,255,255))
        self.x_arrow_hd1_line = pyglet.shapes.Line(self.wind_rect.x,
                                              self.wind_rect.y+20,
                                              self.wind_rect.x+30,
                                              self.wind_rect.y+30,
                                              5, color = (255,255,255))
        self.x_arrow_hd2_line = pyglet.shapes.Line(self.wind_rect.x,
                                              self.wind_rect.y+20,
                                              self.wind_rect.x+30,
                                              self.wind_rect.y+10,
                                              5, color = (255,255,255))
    """  
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
        
        
        #self.head_wind_label.text = 'h:' + str(int(round(wind_comps[1])))
        #self.head_wind_label.draw()
        #self.x_wind_label.text = 'x:' + str(int(round(wind_comps[0])))
        #self.x_wind_label.draw()
        
        self.set_wind_comps_text(heading, msg_wind_speed, msg_wind_dir)
        
    def draw_msg(self, msg_wind_speed, msg_wind_dir, heading):
        self.wind_rect.draw()
        self.wind_speed_label.text = str(int(round(msg_wind_speed))) + "@"
        self.wind_speed_label.draw()
        self.wind_dir_label.text = str(round(msg_wind_dir))
        self.wind_dir_label.draw()
        
        wind_comps = self.get_wind_comps(heading, msg_wind_speed, msg_wind_dir)
        
        self.head_wind_label.text = 'h:' + str(int(round(wind_comps[1])))
        self.head_wind_label.draw()
        self.x_wind_label.text = 'x:' + str(int(round(wind_comps[0])))
        self.x_wind_label.draw()
        
        self.set_wind_comps_text(heading, msg_wind_speed, msg_wind_dir)
    """
    def draw_calc(self, airspeed, heading, gnd_speed, gnd_track, altitude):
        wind = self.calulateWind(airspeed, heading, gnd_speed, gnd_track, altitude)
        #print('wind ang', wind[0])
        #print('wind speed', wind[1])
        
        
        self.wind_rect.draw()
        self.wind_speed_label.text = str(int(round(wind[0]))) + "@"
        self.wind_speed_label.draw()
        self.wind_dir_label.text = str(int(round(wind[1])))
        self.wind_dir_label.draw()
        
        #self.draw_arrow(50,50, 300, True, True)
        #self.draw_arrow(50,50, 300, False, True)
        #self.draw_xwind(self.wind_rect.x, self.wind_rect.y+30, -15)
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
    
    count = 0
    d_count = 0
    def on_draw():
        global mock_wind
        global mock_dir
        global mock_heading
        global mock_track
        global mock_gnd_speed
        global mock_air_speed
        global count
        global d_count
        
        
        window.clear()
        #rect.draw()
        #wind.draw(mock_air_speed, mock_heading, mock_gnd_speed, mock_track, mock_wind, mock_dir)
        rect.draw()
        #wind.draw(85, 355, 95, 345, mock_wind, mock_dir)
        #wind.draw_calc(85, 355, 95, 345, 5000)
        #count = 0
        print(test_case[0][0])
        wind.draw_calc(test_case[count][0],test_case[count][1],test_case[count][2],test_case[count][3],test_case[count][4])
        #return
        d_count += 1
        if d_count > 10:
            count += 1
            if count > 4:
                count = 0
            d_count = 0
        
        
    
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

        
    #window = pyglet.window.Window(1900,1000,fullscreen=True)
    window = pyglet.window.Window(900,400)
    
    window.on_draw = on_draw
    
    center_x = window.width/2
    center_y = window.height/2
    
    wind = Wind(0, 100, 350, 160)
    rect = shapes.BorderedRectangle(0,0, window.width, window.height, border=10, color = (0, 0, 0),
                                            border_color = (255,255,255))
    
    
    #draw(airspeed, heading, gnd_speed, gnd_track, msg_wind_speed, msg_wind_dir)
    wind.draw_calc(85, 355, 95, 345, 5000)
    wind.draw_calc(100, 99, 120, 44, 5000)
    wind.draw_calc(75,  97, 70,  85, 5000)
    wind.draw_calc(100, 180,111, 180, 5000)
    wind.draw_calc(100, 95, 100, 90,  5000)
    
    test_case = [
    (85, 355, 95, 345, 0),
    (100, 99, 120, 44, 0),
    (75,  97, 70,  85, 0),
    (100, 180,111, 180, 0),
    (100, 95, 100, 90,  0)
    ]
    
    
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
    
    
    #pyglet.clock.schedule_interval(update, .1)
    pyglet.clock.schedule_interval(update, 100)
    pyglet.clock.schedule_interval(mock_data, .5)
    pyglet.clock.schedule_interval(mock_gnd_air, .5)

    pyglet.app.run()
        