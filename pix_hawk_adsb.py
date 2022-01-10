from re import A, L, M, S
import pyglet
from pyglet import shapes
from threading import Lock, Thread
import time
import math

from pyglet.graphics import draw
from mavextra import gps_offset
from pix_hawk_util import Math
import numpy
import pix_hawk_config

"""
1	ADSB_FLAGS_VALID_COORDS	
2	ADSB_FLAGS_VALID_ALTITUDE	
4	ADSB_FLAGS_VALID_HEADING	
8	ADSB_FLAGS_VALID_VELOCITY	
16	ADSB_FLAGS_VALID_CALLSIGN	
32	ADSB_FLAGS_VALID_SQUAWK	
64	ADSB_FLAGS_SIMULATED	
128	ADSB_FLAGS_VERTICAL_VELOCITY_VALID	
256	ADSB_FLAGS_BARO_VALID	
32768	ADSB_FLAGS_SOURCE_UAT	


ADSB_VEHICLE ( #246 )
[Message] The location and information of an ADSB vehicle

Field Name	Type	Units	Values	Description
ICAO_address	uint32_t			ICAO address
lat	int32_t	degE7		Latitude
lon	int32_t	degE7		Longitude
altitude_type	uint8_t		ADSB_ALTITUDE_TYPE	ADSB altitude type.
altitude	int32_t	mm		Altitude(ASL)
heading	uint16_t	cdeg		Course over ground
hor_velocity	uint16_t	cm/s		The horizontal velocity
ver_velocity	int16_t	cm/s		The vertical velocity. Positive is up
callsign	char[9]			The callsign, 8+null
emitter_type	uint8_t		ADSB_EMITTER_TYPE	ADSB emitter type.
tslc	uint8_t	s		Time since last communication in seconds
flags	uint16_t		ADSB_FLAGS	Bitmap to indicate various statuses including valid data fields
squawk	uint16_t			Squawk code

"""


class AdsbDict():
    _instance = None
    _instance_count = 0
    _run_thread = True
    

    @classmethod
    def get_instance(cls):
        try:
            if cls._instance == None:
                cls._instance = AdsbDict()
                #cls._run_thread = True
                #cls._instance.start()
            cls._instance_count += 1
            return cls._instance
        except Exception as ex:
            print(ex)
            cls.put_instance()
            return None
    
    @classmethod
    def put_instance(cls):
        cls._instance_count -= 1
        if cls._instance_count <= 0:
            cls._instance_count = 0
            cls._run_thread = False
            cls._instance = None

    def __init__(self):
        #Thread.__init__(self)
        self.lock = Lock()
        self.dict = {}
        #vh = AdsbVehicle('123', '456', 0, 0, 5000, 0,0,0)
        #self.updateVehicle(vh)
        
    def addVehicle(self, vehicle):
        with self.lock:
            if not str(vehicle.icao) in self.dict:
                self.dict[str(vehicle.icao)] = vehicle
    
    #AdsbVehicle('1234546', callsign, lat, lon, adsb_altitude, hor_velocity, ver_velocity, adsb_heading)
    def updateVehicle(self, icao, callsign, lat, lon, adsb_altitude, hor_velocity, ver_velocity, adsb_heading, all_valid):
        with self.lock:
            if not icao in self.dict:
                if not all_valid:
                    return
                self.dict[str(icao)] = AdsbVehicle(icao, callsign, lat, lon, adsb_altitude, hor_velocity, ver_velocity, adsb_heading)
                self.dict[str(icao)].time = time.time()
            else:
                #heading = Math.get_bearing(self.dict[str(icao)].lat, self.dict[str(icao)].lon, lat, lon)
                #heading = Math.get_bearing(lat, lon, self.dict[str(icao)].lat, self.dict[str(icao)].lon)
                #if heading < 0:
                    #heading = adsb_heading
                #print('lat lon heading ********', heading)
                if not all_valid:
                    self.dict[str(icao)].retry_count -= 1
                    if self.dict[str(icao)].retry_count <= 0:
                        del self.dict[str(icao)]
                        return

                #if icao != 'myicao1234':
                if icao != pix_hawk_config.icao:
                    self.dict[str(icao)].update_tail(lat,lon)
                self.dict[str(icao)].retry_count = 3
                self.dict[str(icao)].icao = icao
                self.dict[str(icao)].call_sign = callsign
                self.dict[str(icao)].lat = lat
                self.dict[str(icao)].lon = lon
                self.dict[str(icao)].altitude = adsb_altitude
                self.dict[str(icao)].h_speed = hor_velocity
                self.dict[str(icao)].v_speed = ver_velocity
                self.dict[str(icao)].heading = adsb_heading
                #self.dict[str(icao)].heading = int(heading)
                self.dict[str(icao)].time = time.time()
            
    def getVehicle(self, icao):
        with self.lock:
            if str(icao) in self.dict:
                return self.dict[str(icao)]
            else:
                return None
    """
    def run(self):
        while AdsbDict._run_thread:
            time.sleep(.1)
            with self.lock:
                delete = []
                for key in self.dict:
                    cur_time = time.time()
                    vh = self.dict[key]
                    if cur_time - vh.time > 90:
                        delete.append(str(vh.icao))

                #for key in delete:
                #    del self.dict[key]
    """
            

        
    
class AdsbVehicle():
    def __init__(self, icao, call_sign, lat, lon, altitude, h_speed, v_speed, heading):
        self.icao = str(icao)
        self.call_sign = str(call_sign)
        self.lat = lat
        self.lon = lon
        self.altitude = altitude
        self.h_speed = h_speed
        self.v_speed = v_speed
        self.heading = heading
        self.time = time.time()
        self.vh_label2 = None
        self.fuse_line = None
        self.wing_line = None
        self.retry_count = 3
        self.tail_list = []
        

    def draw(self, x_pos, y_pos, gps_alt, gps_track, sprite, distance, adsb_window):
        
        try:
            if self.vh_label2 == None:
                self.vh_label2 = pyglet.text.Label('****',
                          font_size=20,
                          x=0,
                          y=0,
                          anchor_y='bottom', anchor_x='center')
            if self.fuse_line == None:
                self.fuse_line = shapes.Line(0,    30, 0,   -30, 3, color = (0, 255, 0))
            if self.wing_line == None:
                self.wing_line = shapes.Line(20,    10, -20,   10,  3, color = (255, 0, 0))
            
            self.vh_label2.x = x_pos
            self.vh_label2.y = y_pos
            lsrt = str(self.icao)
            alt_dif = int(self.altitude/100 - gps_alt / 100)
            self.vh_label2.text = str(alt_dif)  #+ ':' + self.call_sign #str(self.heading) + ':' + self.call_sign
            #self.vh_label2.draw()
            #self.fuse_line.anchor_x = x_pos
            #self.fuse_line.anchor_y = y_pos
            #self.fuse_line.draw()

            self.wing_line.position = (20+x_pos, 10+y_pos, -20+x_pos, 10+y_pos)
            #self.wing_line.position = self.get_line_pos(self.wing_line.position, x_pos, y_pos)
            #self.wing_line.draw()
            sprite.scale = 1
            sprite.rotation = self.heading
            sprite.position = x_pos, y_pos
            #sprite.draw()
            
            #line = shapes.Line(x_pos, y_pos, x_pos+8, y_pos+8, 10, color=(255,0,0))
            #line.draw()
            #if self.icao != 'myicao1234':
            if self.icao != pix_hawk_config.icao:
                radious = (50 - 2*abs(alt_dif))
                if radious < 15:
                    radious = 15
                circle = shapes.Circle(x_pos, y_pos, radious, color=(0,255,255))
                if abs(alt_dif) < 15:
                    if alt_dif >= 0:
                        circle.color=(255,0,0)
                    else:
                        circle.color =(255,255,0)
                circle.opacity = (125)
                circle.anchor_x=0
                circle.anchor_y=0
                if distance > 4:
                    circle.radius = 15
                    circle.color = (0,255,255)
                circle.draw()
                self.vh_label2.draw()

            else:
                radious = adsb_window.get_pix_mile_y() * 3
                circle = shapes.Circle(x_pos, y_pos, radious, color=(200,200,200))
                circle.opacity = (50)
                circle.anchor_x=0
                circle.anchor_y=0
                sprite.scale = .8
                rot = self.heading #- 180
                if rot < 360:
                    rot = 360 - rot
                sprite.rotation = 0
                sprite.position = x_pos, y_pos
                #sprite.color = (255,255,0)
                sprite.draw()

                circle2 = shapes.Circle(x_pos, y_pos, 7, color=(0,255,255))
                circle2.anchor_x=0
                circle2.anchor_y=0
                circle2.draw()

                circle.draw()



            #circle.draw()
            #self.vh_label2.draw()

        except Exception as ex:
            print(ex)

    def get_line_pos(self, line_pos, x_pos, y_pos):
        x = (line_pos[0] + x_pos)
        y = (line_pos[1] + y_pos)
        x2 = (line_pos[2] + x_pos)
        y2 = (line_pos[3] + y_pos)
        return x,y,x2,y2

    def update_tail(self, lat, lon):
        self.tail_list.insert(0, (lat,lon))
        if len(self.tail_list) > 20:
            self.tail_list.pop()
        

class AdsbWindow():
    def __init__(self, adsb_dic, pyglet_window, compass_width):
        #self.adsb_dic = AdsbDict.get_instance()
        self.adsb_dic = adsb_dic
        x_pos = pyglet_window._x + compass_width
        wd = pyglet_window.width - compass_width
        self.border_rect = shapes.BorderedRectangle(x_pos, pyglet_window._y, wd, pyglet_window.height,
                                                    border=10, color = (0,0, 0),  border_color = (255,255,255))
        self.vh_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.border_rect.x,
                          y=self.border_rect.y,
                          anchor_y='bottom', anchor_x='left')
        self.vh_icao = pyglet.text.Label('****',
                          font_size=30,
                          x=self.border_rect.x,
                          y=self.border_rect.y + 100,
                          anchor_y='bottom', anchor_x='left')
        self.miles_per_degree_lat = 69
        self.miles_per_degree_lon = 53
        #self.win_wd_degress = 3 * self.miles_per_degree_lat
        #self.win_ht_degrees = 3 * self.miles_per_degree_lon
        self.win_max_miles = 1.5
        self.win_x_org = x_pos + self.border_rect.width / 2
        self.win_y_org = pyglet_window._y + self.border_rect.height / 2
        self.arrow_image = pyglet.image.load('/home/pi/Downloads/_arrow.jpg')
        self.arrow_image.anchor_x = int(50/2)
        self.arrow_image.anchor_y = int(100/2)
        self.arrow_sprite = pyglet.sprite.Sprite(self.arrow_image, x=150, y=150)
        
        self.N_img = pyglet.image.load('/home/pi/Downloads/N_img.jpg')
        self.N_sprite = pyglet.sprite.Sprite(self.N_img, 0,0)
        self.N_sprite.anchor_x = 0
        self.N_sprite.ahchor_y = 0
        
    def latlon_distance(self, lat1, lon1, lat2, lon2):
        origin = (lat1,lon1)
        destination = (lat2,lon2)
        radius = 6371
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
            math.sin(dlon / 2) * math.sin(dlon / 2))

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        d = (radius * c) / 1.609344 # convet to miles

        return d

    def get_bearing(self, lat1, long1, lat2, long2):
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x,y)
        brng = numpy.degrees(brng)

        brng = (brng + 360) % 360

        return brng

    def get_pixel_pos(self, N423DS, lat, lon, gps_lat, gps_lon):

        angle = Math.get_bearing(N423DS.lat, N423DS.lon, lat, lon)
        angle += N423DS.heading
        if angle > 360:
            angle = angle - 360

        dist = self.latlon_distance(N423DS.lat, N423DS.lon, lat, lon)

        xy = Math.pol2cart(dist, angle)

        x_miles = (lon-gps_lon) * self.miles_per_degree_lon
        x_pos = x_miles/self.win_max_miles * self.border_rect.width + self.win_x_org

        y_miles = (lat - gps_lat) * self.miles_per_degree_lat
        y_pos = y_miles/self.win_max_miles
        y_pos = y_pos * self.border_rect.height + self.win_y_org

               

        pix_mile_x = .75/4 * self.border_rect.width/2
        pix_mile_y = .75/4* self.border_rect.height/2

        x_pos = pix_mile_x*xy[0]+self.border_rect.x+self.border_rect.width/2
        y_pos = pix_mile_y*xy[1]+self.border_rect.y+self.border_rect.height/2

        return (x_pos, y_pos)

    def get_pix_mile_x(self):
        return .75/4 * self.border_rect.width/2

    def get_pix_mile_y(self):
        return .75/4* self.border_rect.height/2

        

    def draw(self, gps_lat, gps_lon, gps_alt, gps_track):
        

        self.border_rect.draw()

        self.N_sprite.position = (self.border_rect.x + self.border_rect.width - 60, self.border_rect.y + self.border_rect.height-110)
        self.N_sprite.scale = 1.5
        #self.N_sprite.draw()

        self.arrow_sprite.position = (self.border_rect.x + self.border_rect.width - 45, self.border_rect.y + self.border_rect.height-60)
        N423DS = self.adsb_dic.dict[pix_hawk_config.icao]
        rot = N423DS.heading
        if rot < 360:
            rot = 360 - rot
        self.arrow_sprite.rotation = rot
        self.arrow_sprite.scale_y = .75
        self.arrow_sprite.scale_x = 1
        self.arrow_sprite.draw()
        self.N_sprite.draw()


        if len(self.adsb_dic.dict) <= 1:
            self.vh_label.color = (0,255,0,255)
        else:
            self.vh_label.color = (255,255,255,255)

        self.vh_label.text = 'PC:' + str(len(self.adsb_dic.dict)-1)
        if len(self.adsb_dic.dict) == 0:
            self.vh_icao.text = "none"
            self.vh_icao.draw()
        self.vh_label.draw()
        with self.adsb_dic.lock:

            del_list=[]
            #N423DS = self.adsb_dic.dict['myicao1234']
            N423DS = self.adsb_dic.dict[pix_hawk_config.icao]
            for key in self.adsb_dic.dict:

                vh = self.adsb_dic.dict[key]
                dist = self.latlon_distance(N423DS.lat, N423DS.lon, vh.lat, vh.lon)
                #vh = self.adsb_dic.getVehicle(key)
                if time.time() - vh.time > 10:
                    del_list.append(vh.icao)
                    continue

                dist = self.latlon_distance(N423DS.lat, N423DS.lon, vh.lat, vh.lon)
                print('distance', dist)
                if dist > 5:
                   continue

                """#angle = self.get_bearing(N423DS.lat, N423DS.lon, vh.lat, vh.lon)
                angle = Math.get_bearing(N423DS.lat, N423DS.lon, vh.lat, vh.lon)

                xy = Math.pol2cart(dist, angle)

                x_miles = (vh.lon-gps_lon) * self.miles_per_degree_lon
                x_pos = x_miles/self.win_max_miles * self.border_rect.width + self.win_x_org

                y_miles = (vh.lat - gps_lat) * self.miles_per_degree_lat
                y_pos = y_miles/self.win_max_miles
                y_pos = y_pos * self.border_rect.height + self.win_y_org

                pix_mile_x = .75/4 * self.border_rect.width/2
                pix_mile_y = .75/4* self.border_rect.height/2

                x_pos = pix_mile_x*xy[0]+self.border_rect.x+self.border_rect.width/2
                y_pos = pix_mile_y*xy[1]+self.border_rect.y+self.border_rect.height/2
                """

                x_pos, y_pos = self.get_pixel_pos(N423DS, vh.lat, vh.lon, gps_lat, gps_lon)


                #vh.draw(x_pos, y_pos, gps_alt, gps_track, self.arrow_sprite)

                for pos in vh.tail_list:
                    xy = self.get_pixel_pos(N423DS, pos[0], pos[1], gps_lat, gps_lon)
                    #dot = shapes.Circle(xy[0], xy[0], 20, color=(255,0,0))
                    line = shapes.Line(xy[0], xy[1], xy[0]+5, xy[1]+5, 5, color=(0,255,0))
                    #dot.draw()
                    line.draw()
                
                vh.draw(x_pos, y_pos, gps_alt, gps_track, self.arrow_sprite, dist, self)

            for key in del_list:                    
                del self.adsb_dic.dict[key]



    def close(self):
        AdsbDict.put_instance()



if __name__ == '__main__':
    # unit test code

    import pix_hawk_msg

    msg_thread = pix_hawk_msg.mavlinkmsg.get_instance() 

    ahdata = pix_hawk_msg.aharsData(-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1)
    
    #adsbDict = AdsbDict.get_instance()
    
    print('__main__')
    
    #adsb1 = AdsbVehicle(123, 'n123', 2000, 3000, 5500, 70, 5, 90)
    #adsb2 = AdsbVehicle(456, 'n789', 4000, 5000, 7500, 100, -5, 180)
    
    #print(adsb1.call_sign)
    
    #print(adsb2.call_sign)
    
    #adsbDict.addVehicle(adsb1)
    #adsbDict.addVehicle(adsb2)
    
    #print(adsbDict.getVehicle(adsb1.icao).call_sign)
    
    #adsb1.call_sign = "newsign"
    #adsbDict.updateVehicle(adsb1)
    #print(adsbDict.getVehicle(adsb1.icao).call_sign)
    
    #print(len(adsbDict.dict))

   
    window = pyglet.window.Window(1200,700)

    adsbwin = AdsbWindow(msg_thread.adsb_dic, window, 1000/2)
    
    def on_draw():
        global ahdata
        window.clear()
        #rect.draw()
        ahdata = msg_thread.getAharsData(ahdata)
        adsbwin.draw(ahdata.lat, ahdata.lon, ahdata.gps_alt, ahdata.gnd_track)
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    window.on_draw = on_draw

    pyglet.clock.schedule_interval(update, .1)
    
    pyglet.app.run()

    pix_hawk_msg.mavlinkmsg.put_instance()
    adsbwin.close()



    
    