
import serial
import time

SERIAL_PORT = "/dev/ttyACM2"
running = True

# In the NMEA message, the position gets transmitted as:
# DDMM.MMMMM, where DD denotes the degrees and MM.MMMMM denotes
# the minutes. However, I want to convert this format to the following:
# DD.MMMM. This method converts a transmitted string to the desired format
def formatDegreesMinutes(coordinates, digits):
    
    parts = coordinates.split(".")

    if (len(parts) != 2):
        return coordinates

    if (digits > 3 or digits < 2):
        return coordinates
    
    left = parts[0]
    right = parts[1]
    degrees = str(left[:digits])
    minutes = str(right[:3])

    return degrees + "." + minutes

# This method reads the data from the serial port, the GPS dongle is attached to,
# and then parses the NMEA messages it transmits.
# gps is the serial port, that's used to communicate with the GPS adapter
def getPositionData(gps):
    data = gps.readline()
    data = data.decode('utf-8')
    message = data[0:6]
    #print(message)
    if (message == "$GPRMC"):
        # GPRMC = Recommended minimum specific GPS/Transit data
        # Reading the GPS fix data is an alternative approach that also works
        parts = data.split(",")
        if parts[2] == 'V':
            # V = Warning, most likely, there are no satellites in view...
            print ("GPS receiver warning")
        else:
            # Get the position data that was transmitted with the GPRMC message
            # In this example, I'm only interested in the longitude and latitude
            # for other values, that can be read, refer to: http://aprs.gids.nl/nmea/#rmc
            longitude = formatDegreesMinutes(parts[5], 3)
            latitude = formatDegreesMinutes(parts[3], 2)
            speed = parts[7]
            track = parts[8]
            if len(track) == 0:
                track = 'missing'
            print ("position: lon = " + str(longitude) + ", lat = " + str(latitude))
            print('speed {0}, course {1}'.format(speed, track))

    if message == "$GPGGA":
        #global positioning System Fix Data
        parts = data.split(",")
        longitude = formatDegreesMinutes(parts[4], 3)
        latitude = formatDegreesMinutes(parts[2], 2)
        fix = parts[6]
        alt = float(parts[9])
        alt = alt * .00328
        alt = alt * 1000
        num_sat = parts[7]
        
        print('fix {0}, altitude {1}, num sats {2}'.format( fix, alt, num_sat))

    """if message == "$GPRMA":
        parts = data.split(",")
        track = parts[9]
        print('gnd course {0}'.format(track))"""

    if message == '$GPVTG': # ground ref vars
        parts = data.split(",")
        track = parts[1]
        if len(track) == 0:
                track = 'missing'
        print('track made good {0}'.format(track))


    else:
        pass

print ("Application started!")
gps = serial.Serial(SERIAL_PORT, baudrate = 9600, timeout = 0.5)

while running:
    try:
        getPositionData(gps)
        #time.sleep(.1)
    except KeyboardInterrupt:
        running = False
        gps.close()
        print ("Application closed!")
    except:
        # You should do some error handling here...
        print ("Application error!")