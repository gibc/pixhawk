from re import A, M, S
import pyglet
from pyglet import shapes
from threading import Lock, Thread
import time
import math
from pix_hawk_util import Math
import numpy

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
    def updateVehicle(self, icao, callsign, lat, lon, adsb_altitude, hor_velocity, ver_velocity, adsb_heading):
        with self.lock:
            if not icao in self.dict:
                self.dict[str(icao)] = AdsbVehicle(icao, callsign, lat, lon, adsb_altitude, hor_velocity, ver_velocity, adsb_heading)
                self.dict[str(icao)].time = time.time()
            else:
                self.dict[str(icao)].icao = icao
                self.dict[str(icao)].call_sign = callsign
                self.dict[str(icao)].lat = lat
                self.dict[str(icao)].lon = lon
                self.dict[str(icao)].altitude = adsb_altitude
                self.dict[str(icao)].h_speed = hor_velocity
                self.dict[str(icao)].v_speed = ver_velocity
                self.dict[str(icao)].heading = adsb_heading
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
        
    def draw(self, x_pos, y_pos, gps_alt, gps_track):
        
        try:
            if self.vh_label2 == None:
                self.vh_label2 = pyglet.text.Label('****',
                          font_size=30,
                          x=0,
                          y=0,
                          anchor_y='bottom', anchor_x='center')
            self.vh_label2.x = x_pos
            self.vh_label2.y = y_pos
            lsrt = str(self.icao)
            self.vh_label2.text = self.call_sign
            self.vh_label2.draw()
        except Exception as ex:
            print(ex)
        

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

        d = (radius * c) / 1.609344 # convet to mph

        return d

    def get_bearing(self, lat1, long1, lat2, long2):
        dLon = (long2 - long1)
        x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
        y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
        brng = numpy.arctan2(x,y)
        brng = numpy.degrees(brng)

        return brng



    def draw(self, gps_lat, gps_lon, gps_alt, gps_track):
        self.border_rect.draw()
        self.vh_label.text = 'plane count ' + str(len(self.adsb_dic.dict))
        if len(self.adsb_dic.dict) == 0:
            self.vh_icao.text = "none"
            self.vh_icao.draw()
        self.vh_label.draw()
        with self.adsb_dic.lock:

            del_list=[]
            N423DS = self.adsb_dic.dict['myicao1234']
            for key in self.adsb_dic.dict:

                vh = self.adsb_dic.dict[key]
                dist = self.latlon_distance(N423DS.lat, N423DS.lon, vh.lat, vh.lon)
                #vh = self.adsb_dic.getVehicle(key)
                if time.time() - vh.time > 30:
                    del_list.append(vh.icao)
                    continue

                dist = self.latlon_distance(N423DS.lat, N423DS.lon, vh.lat, vh.lon)
                print('distance', dist)
                if dist > 10:
                   continue

                angle = self.get_bearing(N423DS.lat, N423DS.lon, vh.lat, vh.lon)

                xy = Math.pol2cart(dist, angle)

                x_miles = (vh.lon-gps_lon) * self.miles_per_degree_lon
                x_pos = x_miles/self.win_max_miles * self.border_rect.width + self.win_x_org

                y_miles = (vh.lat - gps_lat) * self.miles_per_degree_lat
                y_pos = y_miles/self.win_max_miles
                y_pos = y_pos * self.border_rect.height + self.win_y_org

                #sico = str(vh.icao)
                #self.vh_icao.text = sico
                #self.vh_icao.draw()

                pix_mile_x = .75/4 * self.border_rect.width/2
                pix_mile_y = .75/4* self.border_rect.height/2

                x_pos = pix_mile_x*xy[0]+self.border_rect.x+self.border_rect.width/2
                y_pos = pix_mile_y*xy[1]+self.border_rect.y+self.border_rect.height/2


                vh.draw(x_pos, y_pos, gps_alt, gps_track)

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

   
    window = pyglet.window.Window(1000,700)

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



    
    