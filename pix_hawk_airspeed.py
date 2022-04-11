from threading import Lock, Thread
import i2cdriver
import math
import time

class AirSpeed():
    def __init__(self, con_str):
        self.con_str = con_str
        self.run_thread = True
        self.airspeed_thread = Thread(target = self.target)
        self.connected = False
        self.lock = Lock()
        self.airspeed = -1
        self.temperature = 0

    def connect(self):
        try:
            self.i2c = i2cdriver.I2CDriver(self.con_str)
            self.connected = True
            self.airspeed = 0
            print('airspeed connected to USB\n')
            return True
        except:
            self.connected = False
            self.airspeed = -1
            return False

    def get_airspeed(self):
        with self.lock:
            airspd = self.airspeed
        return airspd

    def set_aispeed(self, airspd):
        with self.lock:
            self.airspeed = airspd

    def get_temp(self):
        with self.lock:
            temp = self.temperature
            temp = round(temp, 1)
        return temp

    def set_temp(self, temp):
        with self.lock:
            self.temperature = temp


    def indicated_airspeed(self, pressure):
        if pressure <= 0:
            return 0
        mps = math.sqrt(2*(pressure/1.225))
        mph = mps * 2.237
        return mph


    def raw_pres2pa(self, dp_raw):
        P_min = -1.0
        P_max = 1.0
        PSI_to_Pa = 6894.757

        diff_press_PSI = ((dp_raw - 0.1 *16383)*(P_max - P_min)/ (0.8 * 16383)+P_min)
        diff_press_pa_raw = diff_press_PSI * PSI_to_Pa
        return diff_press_pa_raw


    def parse(self, str):
        pa = bytearray()
        ta = bytearray()
        ln = len(str)
        for i in range(ln):
            v = str[i]
            if i < 2:
                pa.append(v)
            else:
                ta.append(v)
        #print(v)
    
        pres = int.from_bytes(pa,byteorder='big')
        pa_pressure = self.raw_pres2pa(pres)
        #print('pa pressure ', pa_pressure)
        speed_mph = self.indicated_airspeed(pa_pressure)
        #print('air speed mph ', speed_mph)
        pres -= 8000

        temp = int.from_bytes(ta,byteorder='big')
        temp /= 32
        temp = ((200.0 * temp) / 2047) - 50
        temp = (temp*9/5)+32 # C to F
    
        #print('prss ', pres)
        #print('temp ', temp)
        self.set_aispeed(speed_mph)
        self.set_temp(temp)
        return speed_mph, temp

    def close(self):
        self.run_thread = False
        self.airspeed_thread.join()


    def target(self):
        print('airspeed thread started.')
        while self.run_thread:
            if not self.connected:
                self.connect()
                time.sleep(.5)
                continue

            self.i2c.start(0x28, 1)
    
            time.sleep(.1)
    
            r = self.i2c.read(4)
    
            self.parse(r)

            time.sleep(.1)

        print('airspeed thread stopped.')   

if __name__ == '__main__':
    # unit test code
    ascls = AirSpeed('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN00EDLI-if00-port0')
    ascls.airspeed_thread.start()
    while True:
        print('air speed ', ascls.get_airspeed())
        print('temperature ', ascls.get_temp())
        time.sleep(.5)