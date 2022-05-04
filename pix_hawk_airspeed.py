from threading import Lock, Thread
import i2cdriver
import math
import time
from pix_hawk_log import Log
from pix_hawk_barometer import Barometer

class AirSpeed():
    def __init__(self, con_str, baro):
        self.baro = baro
        self.con_str = con_str
        self.run_thread = True
        self.airspeed_thread = Thread(target = self.target)
        self.connected = False
        self.lock = Lock()
        self.airspeed = -1
        self.temperature = 0
        self.true_air_sp = -1

    def connect(self):
        try:
            #Log.log_val('ap_connect', self.con_str)
            self.i2c = i2cdriver.I2CDriver(self.con_str)
            self.connected = True
            self.airspeed = 0
            print('airspeed connected to USB\n')
            Log.log_val('air_sp_connect', 'success')
            return True
        except:
            self.connected = False
            self.airspeed = -1
            Log.log_val('air_sp_connect', 'fail')
            return False

    def get_airspeed(self):
        with self.lock:
            airspd = self.airspeed
        #Log.log_val('air_spd', int(airspd))
        return airspd

    def set_aispeed(self, airspd):
        with self.lock:
            self.airspeed = airspd

    def get_true_airspeed(self):
        with self.lock:
            tairspd = self.true_air_sp
        #Log.log_val('air_spd', int(airspd))
        return tairspd

    def set_true_aispeed(self, true_air_sp):
        with self.lock:
            self.true_air_sp = true_air_sp

    def get_temp(self):
        with self.lock:
            temp = self.temperature
            temp = round(temp, 1)
        return temp

    def set_temp(self, temp):
        with self.lock:
            self.temperature = temp


    def indicated_airspeed(self, pressure):
        #Log.log_val('air_sp_press', int(pressure))
        if pressure <= 0:
            return 0
        mps = math.sqrt(2*(pressure/1.225))
        mph = mps * 2.23694 #meters /sec to mph
        rescale = 15/10 * 1.0567193 # from in plane test
        mph *= rescale
        Log.log_val('air_sp_mph', int(mph))
        return mph


    def raw_pres2pa(self, dp_raw):
        P_min = -1.0
        P_max = 1.0
        PSI_to_Pa = 6894.757

        #diff_press_PSI = ((dp_raw - 0.1 *16383)*(P_max - P_min)/ (0.8 * 16383)+P_min)
        diff_press_PSI = -((dp_raw - 0.1 *16383)*(P_max - P_min)/ (0.8 * 16383)+P_min)
        diff_press_pa_raw = diff_press_PSI * PSI_to_Pa

        psi = -(dp_raw-8000) * 1/14746
        #Log.log_val('air_sp_psi', int(psi))
        diff_press_pa_raw = psi * PSI_to_Pa
        diff_press_pa_raw -= 3  # calibration from log
        if diff_press_pa_raw <= 0:
            diff_press_pa_raw = 0
        Log.log_val('air_sp_diff_press_pa', diff_press_pa_raw)
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
        Log.log_val('air_sp_sensor_press', int(pres))
        pres -= 27 # calibration off set from log
        
        pa_pressure = self.raw_pres2pa(pres)
        #print('pa pressure ', pa_pressure)
        speed_mph = self.indicated_airspeed(pa_pressure)
        #print('air speed mph ', speed_mph)
        pres -= 8000

        temp = int.from_bytes(ta,byteorder='big')
        temp /= 32
        temp = ((200.0 * temp) / 2047) - 50
        tempK = temp + 273.15
        temp = (temp*9/5)+32 # C to F

    
        den_alt = self.baro.get_density_alt()
        den_alt = self.baro.get_alt()
        #print('den_alt', den_alt)
        true_air_sp_factor = 1 + den_alt/1000 * .02
        true_air_sp_alt = speed_mph * true_air_sp_factor
        #print('true_air_sp_alt', true_air_sp_alt)
        #Log.log_val('true_air_sp_alt', int(true_air_sp_alt))

        true_air_sp_density = -1
        baro_pressure = self.baro.get_pressure()
        if baro_pressure > 0:
            baro_pressure *= 100
            density = baro_pressure / (287.05 * tempK) # air density = pressure / gas constant (287.05)*temp in kelvin
            #Log.log_val('air_density', density)
            #print('air_density', density)
            density_factor = math.sqrt(1.225/density)
            Log.log_val('density_factor', round(density_factor,3))
            Log.log_val('alt_factor', round(true_air_sp_factor,3))
            true_air_sp_density = speed_mph * density_factor # math.sqrt(1.225/density) # std density at sea level /density at altitude and temp
            
        #Log.log_val('true_air_sp_density', int(true_air_sp_density))

        
        alt_2_true = False
        if alt_2_true:
            self.set_true_aispeed(true_air_sp_alt)
        else:
            self.set_true_aispeed(true_air_sp_density)


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

            try:
                self.i2c.start(0x28, 1)
    
                time.sleep(.1)
    
                r = self.i2c.read(4)

            except:
                Log.log_val('air_sp_read', 'failed')
    
            self.parse(r)

            time.sleep(.1)

        print('airspeed thread stopped.')   
        Log.log_val('air_sp_thread', 'stopped')

if __name__ == '__main__':
    # unit test code
    Log.enable_log("/home/pi/PhidgetInsurments/LogFiles", enabled=True)
    barometer = Barometer('ftdi://ftdi:232h:FT4VTTQV/1')   
    barometer.baro_thread.start()
    time.sleep(.5)
    print("baro.connected:", barometer.connected) 
        
    #ascls = AirSpeed('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN00EDLI-if00-port0') # in plane address
    ascls = AirSpeed('/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DN00EERE-if00-port0', barometer) # bench address
    ascls.airspeed_thread.start()
    time.sleep(.5)
    if barometer.connected and ascls.connected:
        for j in range(0, 10):
            for i in range(0,5):
                print('air speed ', ascls.get_airspeed())
                print('temperature ', ascls.get_temp())
                time.sleep(.5)
                Log.write_log()
    
        barometer.close()
        ascls.close()