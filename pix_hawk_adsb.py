from re import M
import pyglet
from pyglet import shapes
from threading import Lock, Thread
import time

class AdsbDict(Thread):
    _instance = None
    _instance_count = 0
    _run_thread = True
    

    @classmethod
    def get_instance(cls):
        try:
            if cls._instance == None:
                cls._instance = AdsbDict()
                cls._run_thread = True
                cls._instance.start()
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
        Thread.__init__(self)
        self.lock = Lock()
        self.dict = {}
        
    def addVehicle(self, vehicle):
        with self.lock:
            if not str(vehicle.icao) in self.dict:
                self.dict[str(vehicle.icao)] = vehicle
    
    def updateVehicle(self, vehicle):
        with self.lock:
            if not str(vehicle.icao) in self.dict:
                vehicle.time = time.time()
                self.dict[str(vehicle.icao)] = vehicle
            else:
                self.dict[str(vehicle.icao)].icao = vehicle.icao
                self.dict[str(vehicle.icao)].call_sign = vehicle.call_sign
                self.dict[str(vehicle.icao)].lat = vehicle.lat
                self.dict[str(vehicle.icao)].lon = vehicle.lon
                self.dict[str(vehicle.icao)].altitude = vehicle.altitude
                self.dict[str(vehicle.icao)].h_speed = vehicle.h_speed
                self.dict[str(vehicle.icao)].v_speed = vehicle.v_speed
                self.dict[str(vehicle.icao)].heading = vehicle.heading
                self.dict[str(vehicle.icao)].time = time.time()
            
    def getVehicle(self, icao):
        with self.lock:
            if str(icao) in self.dict:
                return self.dict[str(icao)]
            else:
                return None

    def run(self):
        while AdsbDict._run_thread:
            time.sleep(.5)
            with self.lock:
                delete = []
                for key in self.dict:
                    cur_time = time.time()
                    vh = self.dict[key]
                    if cur_time - vh.time > 90:
                        delete.append(str(vh.icao))

                for key in delete:
                    del self.dict[key]
            

        
    
class AdsbVehicle():
    def __init__(self, icao, call_sign, lat, lon, altitude, h_speed, v_speed, heading):
        self.icao = icao
        self.call_sign = call_sign
        self.lat = lat
        self.lon = lon
        self.altitude = altitude
        self.h_speed = h_speed
        self.v_speed = v_speed
        self.heading = heading
        self.time = None

class AdsbWindow():
    def __init__(self, adsb_dic, pyglet_window, compass_width):
        #self.adsb_dic = AdsbDict.get_instance()
        self.adsb_dic = adsb_dic
        x_pos = pyglet_window._x + compass_width
        wd = pyglet_window.width - compass_width
        self.border_rect = shapes.BorderedRectangle(x_pos, pyglet_window._x, wd, pyglet_window.height,
                                                    border=10, color = (0,0, 0),  border_color = (255,255,255))
        self.vh_label = pyglet.text.Label('****',
                          font_size=50,
                          x=self.border_rect.x,
                          y=self.border_rect.y,
                          anchor_y='bottom', anchor_x='left')

    def draw(self):
        self.border_rect.draw()
        self.vh_label.text = 'plane count ' + str(len(self.adsb_dic.dict))
        self.vh_label.draw()

    def close(self):
        AdsbDict.put_instance()



if __name__ == '__main__':
    # unit test code

    import pix_hawk_msg

    msg_thread = pix_hawk_msg.mavlinkmsg.get_instance() 
    
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
        window.clear()
        #rect.draw()
        adsbwin.draw()
    
    def update(dt):
        x=0
        #print("dt: ", dt)

    window.on_draw = on_draw

    pyglet.clock.schedule_interval(update, .1)
    
    pyglet.app.run()

    pix_hawk_msg.mavlinkmsg.put_instance()
    adsbwin.close()



    
    