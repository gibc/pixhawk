#from unittest.main import _TestRunner


icao = str(0xA50720) # for N423DS verified with SkyBeacon phone app
callsign = 'N423DS'
DEBUG = True
AdsbAltLimit = 6 # k ft + or -
AdsbDistanceLimit = 5 # miles
AdsbAltThreat = 1 # k ft + or -
AdsbDistanceThreat = 2 # miles
MockAirPlane = True
MockHeading = -1 # -1 for no mock