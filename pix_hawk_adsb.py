import pyglet
from pyglet import shapes

class AdsbDict():
    def __init__(self):
        self.dict = {}
        
    def addVehicle(self, vehicle):
        if not str(vehicle.icao) in self.dict:
            self.dict[str(vehicle.icao)] = vehicle
    
    def updateVehicle(self, vehicle):
        if not str(vehicle.icao) in self.dict:
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
            
    def getVehicle(self, icao):
        if str(icao) in self.dict:
            return self.dict[str(icao)]
        else:
            return None
        
    
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

class AdsbWindow():
    def __init__(self, pyglet_window, compass_width):
        x_pos = pyglet_window._x + compass_width
        wd = pyglet_window.width - compass_width
        self.border_rect = shapes.BorderedRectangle(x_pos, pyglet_window._x, wd, pyglet_window.height,
                                                    border=10, color = (0,0, 0),  border_color = (255,255,255))

    def draw(self):
        self.border_rect.draw()



if __name__ == '__main__':
    # unit test code
    
    adsbDict = AdsbDict()
    
    print('__main__')
    
    adsb1 = AdsbVehicle(123, 'n123', 2000, 3000, 5500, 70, 5, 90)
    adsb2 = AdsbVehicle(456, 'n789', 4000, 5000, 7500, 100, -5, 180)
    
    #print(adsb1.call_sign)
    
    #print(adsb2.call_sign)
    
    adsbDict.addVehicle(adsb1)
    adsbDict.addVehicle(adsb2)
    
    print(adsbDict.getVehicle(adsb1.icao).call_sign)
    
    adsb1.call_sign = "newsign"
    adsbDict.updateVehicle(adsb1)
    print(adsbDict.getVehicle(adsb1.icao).call_sign)
    
    print(len(adsbDict.dict))

   
    window = pyglet.window.Window(1000,700)

    adsbwin = AdsbWindow(window, 1000/2)
    
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


    
    