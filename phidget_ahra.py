import pyglet
from pyglet import clock
from pyglet import shapes
from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Spatial import *
import time
import math
from threading import Lock, Thread
lock = Lock()

roll_x = 0
pitch_y = 0
yaw_z = 0
cnt = 0

def setyaw(yaw):
    global yaw_z
    lock.acquire()
    yaw_z = yaw
    lock.release()
    
def getyaw():
    global yaw_z
    lock.acquire()
    yaw = yaw_z
    lock.release()
    return yaw

def yaw2deg(yaw):
    yawd = math.degrees(yaw)
    if yawd < 0:
        yawd = 360+yawd
    return int(yawd)

from math import sin, cos, radians
def rotate_point(point, angle, center_point=(0, 0)):
    """Rotates a point around center_point(origin by default)
    Angle is in degrees.
    Rotation is counter-clockwise
    """
    angle_rad = radians(angle % 360)
    # Shift the point so that center_point becomes the origin
    new_point = (point[0] - center_point[0], point[1] - center_point[1])
    new_point = (new_point[0] * cos(angle_rad) - new_point[1] * sin(angle_rad),
                 new_point[0] * sin(angle_rad) + new_point[1] * cos(angle_rad))
    # Reverse the shifting we have done
    new_point = (new_point[0] + center_point[0], new_point[1] + center_point[1])
    return new_point

#def rotate_surface(insurface, angle):
#    return pygame.transform.rotate(insurface, angle)

def ahars():
    #accelerometer0 = Accelerometer()
    spatial0 = Spatial()
    
    spatial0.setOnAlgorithmDataHandler(onAlgorithmData)
    #accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
    spatial0.setOnSpatialDataHandler(onSpatialData)

    #accelerometer0.openWaitForAttachment(5000)
    spatial0.openWaitForAttachment(5000)
    spatial0.zeroGyro()

def onAccelerationChange(self, acceleration, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onSpatialData(self, acceleration, angularRate, magneticField, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")
	
def onAlgorithmData(self, quaternion, timestamp):
    print("Quaternion: " + str(quaternion))
    print("Timestamp " + str(timestamp))
    ea = euler_from_quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    print("roll " + str(ea[0]))
    print("pith " + str(ea[1]))
    print("yaw " + str(ea[2]))
    #circle.undraw()
    #circle.draw(theWindow)
    #theWindow.flush()

def euler_from_quaternion(x, y, z, w):
    print('call euler_from_quaternion')
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    global roll_x
    global pitch_y
    global yaw_z
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    setyaw(yaw)
    #print('yaw_z1', yaw_z)
     
    #pygame.time.delay(100)
    #draw_game()
     
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw) # in radians
 


 
window = pyglet.window.Window(500,500)
window1 = pyglet.window.Window(700,700)
#batch = pyglet.graphics.Batch()
N_image = pyglet.image.load('/home/pi/Downloads/N_img.jpg')
N_image.anchor_x = 10
N_image.anchor_y = 10
N_sprite = pyglet.sprite.Sprite(N_image, x=150, y=150)

E_image = pyglet.image.load('/home/pi/Downloads/E_img.jpg')
E_image.anchor_x = 10
E_image.anchor_y = 10
E_sprite = pyglet.sprite.Sprite(E_image, x=150, y=150)
# width of line
width = 2
line1 = shapes.Line(700/2, 700/2, 700/2, 600, width, color = (50, 225, 30))

label = pyglet.text.Label('Hello, world!',
                          font_size=36,
                          x=window.width // 2,
                          y=window.height // 2,
                          anchor_x='center',
                          anchor_y='center')
label1 = pyglet.text.Label('Hello, window 1',
                          font_size=36,
                          x=window1.width // 2,
                          y=window1.height // 2,
                          anchor_x='center',
                          anchor_y='center')


def on_draw():
    window.clear()
    label.draw()
    yaw = yaw2deg((getyaw()))
    N_sprite.rotation = yaw
    N_sprite.draw()

 
def on_draw1():
    window1.clear()
    yaw = getyaw()
    yaw = yaw2deg(yaw)
    label1.text = str(yaw)    
    label1.draw()
    pt = rotate_point((700/2, 600), -yaw, center_point=(700/2, 700/2))
    line1.position = (700/2, 700/2, pt[0], pt[1])
    line1.draw()
    
    N_sprite.update(x=pt[0], y=pt[1], rotation=yaw, scale=None, scale_x=None, scale_y=None)
    N_sprite.draw()
    
    #batch.draw()
    #yaw1 = yaw2deg(getyaw())
    #if yaw > 360:
    #    yaw = 360-yaw
    #pt = rotate_point((700/2, 600), 45, center_point=(700/2, 700/2))
    #E_sprite.update(x=pt[0], y=pt[1], rotation=yaw+90, scale=None, scale_x=None, scale_y=None)
    #E_sprite.draw()

def update(dt):
    x=0
    #print("dt: ", dt)
    
    

window.on_draw = on_draw
window1.on_draw = on_draw1
 
pyglet.clock.schedule_interval(update, .1)

ahars()

pyglet.app.run()



 


 
