import pix_hawk_msg
import pyglet
from pyglet import clock
from pyglet import shapes
from math import sin, cos, radians, fmod
import os

#add a comment to test commit to repo again

pyglet.options['debug_gl']= False

pitch_batch = pyglet.graphics.Batch()
clip_batch = pyglet.graphics.Batch()


gline60= 0
gline45= 0
gline30= 0
gline15= 0
gline0= 0
gline60n= 0
gline45n= 0
gline30n= 0
gline15n= 0

def ex_stop(e):
    print("\033c", end="")
    print("***************************************************")
    print(e)
    print("***************************************************")
    quit()

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

def make_roll_tick(angle, length, start_x, start_y, center_x, center_y):
    global pitch_batch
    global gline60
    global gline45
    global gline30
    global gline15
    global gline0
    global gline60n
    global gline45n
    global gline30n
    global gline15n
    pt = rotate_point((start_x, start_y), angle, center_point=(center_x, center_y))
    pta = rotate_point((start_x, start_y-length), angle, center_point=(center_x, center_y))
    if angle == 60:
        gline60 = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == 45:
        gline45 = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == 30:
        gline30 = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == 15:
        gline15 = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == 0:
        gline0 = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
        
    if angle == -60:
        gline60n = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == -45:
        gline45n = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == -30:
        gline30n = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)
    if angle == -15:
        gline15n = pyglet.shapes.Line(pt[0], pt[1], pta[0], pta[1], width, color = (255,255,255), batch=pitch_batch)

    #gline=shapes.Line(500, 500, 1000, 1000, 4, color = (255,255,255), batch=pitch_batch)
    if angle > 0:
        roll_label_ = pyglet.text.Label(str(angle),font_size=36/2, x=pt[0], y=pt[1], anchor_x='right',anchor_y='bottom', batch=pitch_batch)
    elif angle == 0:
        roll_label_ = pyglet.text.Label(str(angle),font_size=36/2, x=pt[0], y=pt[1]-3, anchor_x='center',anchor_y='bottom', batch=pitch_batch)
    else:
        angle = abs(angle)
        oll_label_ = pyglet.text.Label(str(angle),font_size=36/2, x=pt[0], y=pt[1], anchor_x='left',anchor_y='bottom', batch=pitch_batch)

def get_tick_angle(origin, tick_number, tick_interval):
    #print('origin', origin)
    #print('tick_number', tick_number)
    #print('tick_interval', tick_interval)
    #dist = fmod(origin, tick_interval) # dist to tick 1
    #print("math mod", fmod(origin, tick_interval))
    tick1_pos = round_down(origin, tick_interval)
    tick1_pos = tick1_pos+tick_interval
    #print('origin', origin)
    #print('tick1_pos', tick1_pos)
    
    tick_number = tick_number - 1
    tick_n_pos = get_sum_right(tick1_pos, tick_interval*tick_number)
    #print('tick_n_pos', tick_n_pos)
    return tick_n_pos

from math import floor
def round_down(num, divisor):
    return floor(num / divisor) * divisor

def get_sum_right(a1, a2):
    #print('get_sum_right', a1, a2)
    sum = a1+a2
    if sum < 360:
        return sum
    else:
        return sum - 360

def angle_dif_right(a1, a2):
    if a2 > a1:
        return a2 - a1
    else:
        to360 = 360 - a1
        return to360 + a2
    
def get_90_origin(a1):
    if a1 > 90:
        return a1 - 90
    else:
        return 360+(a1-90)
    
def get_heading_str(heading):
    if heading == 0:
        return '<N>'
    if heading == 360:
        return '<N>'
    if heading == 90:
        return '<E>'
    if heading == 180:
        return '<S>'
    if heading == 270:
        return '<W>'
    else:
        return str(heading)+'°'
    
def get_heading_color(heading):
    ordinal_color = (255,255,0,255)
    if heading == 0:
        return ordinal_color
    if heading == 360:
        return ordinal_color
    if heading == 90:
        return ordinal_color
    if heading == 180:
        return ordinal_color
    if heading == 270:
        return ordinal_color
    else:
        return (255,255,255,255)
    
def draw_pitch_ticks(tick_list, rotate, center, interval, length, pitch):
    idx = 0
    offset = len(tick_list)*interval
    offset = center[1] + offset/2
    offset = offset+pitch
    for line in tick_list:
        pt1 = rotate_point((center[0]-length/2, offset-interval*idx), rotate, center)
        pt2 = rotate_point((center[0]+length/2, offset-interval*idx), rotate, center)
        line.position = pt2[0], pt2[1], pt1[0], pt1[1]
        #line.color = (0,0,0)
        line.draw()
        idx = idx+1
        if idx == len(tick_list)/2:
            idx = idx+1
    
    
        
try:
    msgthd = pix_hawk_msg.mavlinkmsg()
    msgthd.start()
    ahdata = pix_hawk_msg.aharsData(-1,-1,-1)

    #window = pyglet.window.Window(fullscreen=True)
    window = pyglet.window.Window(700,700)
    center_x=window.width / 2
    center_y=window.height / 2
    
    width = 2
    pitch_scale = center_y / 90.0
    pitch_5_y = int(pitch_scale * 5)
    
    #pitch_batch = pyglet.graphics.Batch()
    pitch_tick_list = []
    pitch_tick_list.append(shapes.Line(center_x-30, center_y+pitch_5_y , center_x+30, center_y+pitch_5_y , width, color = (255,255,255)))
    pitch_tick_list.append(shapes.Line(center_x-30, center_y+2*pitch_5_y , center_x+30, center_y+2*pitch_5_y , width, color = (255,255,255)))
    pitch_tick_list.append(shapes.Line(center_x-30, center_y+3*pitch_5_y , center_x+30, center_y+3*pitch_5_y , width, color = (255,255,255)))
    
    pitch_tick_list.append(shapes.Line(center_x-30, center_y-pitch_5_y , center_x+30, center_y-pitch_5_y , width, color = (255,255,255)))
    pitch_tick_list.append(shapes.Line(center_x-30, center_y-2*pitch_5_y , center_x+30, center_y-2*pitch_5_y , width, color = (255,255,255)))
    pitch_tick_list.append(shapes.Line(center_x-30, center_y-3*pitch_5_y , center_x+30, center_y-3*pitch_5_y , width, color = (255,255,255)))
   
    noise_line1 = shapes.Line(center_x-100, center_y , center_x-25, center_y, 2*width, color = (0,255,100), batch=pitch_batch)
    noise_line2 = shapes.Line(center_x+25, center_y , center_x+100, center_y, 2*width, color = (0,255,100), batch=pitch_batch)
    
    noise_line3 = shapes.Line(center_x-25, center_y , center_x, center_y-15, 2*width, color = (0,255,100), batch=pitch_batch)
    noise_line4 = shapes.Line(center_x+25, center_y , center_x, center_y-15, 2*width, color = (0,255,100), batch=pitch_batch)
    
    pitch_label = pyglet.text.Label('pitch: ',
                          font_size=36/2,
                          x=center_x+105,
                          y=window.height // 2,
                          anchor_x='left',
                          anchor_y='center')
        
    N_image = pyglet.image.load('/home/pi/Downloads/N_img.jpg')
    N_image.anchor_x = 10
    N_image.anchor_y = 10
    N_sprite = pyglet.sprite.Sprite(N_image, x=150, y=150)

    E_image = pyglet.image.load('/home/pi/Downloads/E_img.jpg')
    E_image.anchor_x = 10
    E_image.anchor_y = 10
    E_sprite = pyglet.sprite.Sprite(E_image, x=150, y=150)
# width of line
    

#batch = pyglet.graphics.Batch()
    rect_ht = 2000
    rect_wd = 1500
    top_rect = shapes.Rectangle(center_x, center_y, rect_wd, rect_ht,  color = (0, 0, 225))
    top_rect.anchor_x = rect_wd/2
    top_rect.anchor_y = 0

    bot_rect = shapes.Rectangle(center_x, center_y, rect_wd, rect_ht,  color = (139,69,19))
    bot_rect.anchor_x = rect_wd/2
    bot_rect.anchor_y = rect_ht

    ah_win_wd = int((window.width-window.height)/2)

    left_clip_rect = shapes.Rectangle(0, 0, ah_win_wd, window.height, color = (0, 0, 0), batch=clip_batch)
    rgt_clip_rect = shapes.Rectangle(window.width-ah_win_wd, 0, ah_win_wd, window.height, color = (0, 0, 0), batch=clip_batch)

    line1 = shapes.Line(700/2, 700/2, 700/2, 600, width, color = (50, 225, 30))
    horz_line = shapes.Line(0, 500/2, 500, 500/2, width, color = (50, 225, 30))
    
    compass_height = 100
    compass_width = window.width-2*ah_win_wd
    roll_top = window.height - compass_height
    tick_length = 40
    arc_radius = 300
    arc_center = roll_top - arc_radius
    
    compass_rect = shapes.BorderedRectangle(center_x, roll_top+30,  compass_width, compass_height-30, border=10, color = (0, 0, 255),
                                            border_color = (255,255,255), batch=pitch_batch)
    compass_rect.anchor_x = compass_rect.width/2
    
    make_roll_tick(60, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(45, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(30, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(15, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(0, tick_length, center_x, roll_top, center_x, arc_center)
    
    make_roll_tick(-60, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(-45, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(-30, tick_length, center_x, roll_top, center_x, arc_center)
    make_roll_tick(-15, tick_length, center_x, roll_top, center_x, arc_center)
    
      
    roll_arc = shapes.Arc(center_x, arc_center, arc_radius, angle=radians(120), start_angle=radians(30), batch=pitch_batch)
    roll_line = shapes.Line(center_x, window.height-320, center_x, window.height-20, 6, color = (0,255,100))
    #roll_triangle = shapes.Triangle(center_x, roll_top, center_x-20, roll_top-20, center_x+20, roll_top-20, color = (0,255,100))
    roll_label = pyglet.text.Label(' roll: ',
                          font_size=36/2,
                          x=center_x,
                          y=window.height-200,
                          anchor_x='center',
                          anchor_y='top')

    
    heading_tick = 30
    heading_scale = compass_rect.width/180
    heading_center_x = compass_rect.x
    heading_center_y = compass_rect.y+compass_rect.height/2
    
    #heading_label_rect = shapes.Rectangle(compass_rect.x, heading_center_y, 60, 40,  color = (0,0,0))
    heading_label_rect = shapes.BorderedRectangle(compass_rect.x, heading_center_y, 60, 40, border=8, color = (0, 0, 0),
                                            border_color = (255,255,255))
    heading_label_rect.anchor_x = heading_label_rect.width/2
    heading_label_rect.anchor_y = heading_label_rect.height/2
    
    heading_label = pyglet.text.Label('heading: ',
                          font_size=36/2,
                          x=compass_rect.x,
                          y=heading_center_y,
                          anchor_y='center', anchor_x='center')
                          #batch=pitch_batch)
    
    heading_label_1 = pyglet.text.Label('45',
                          font_size=36/2,
                          x=heading_center_x + 45*heading_scale,
                          y=compass_rect.y+compass_rect.height/2,
                          anchor_y='center', anchor_x='center')
    heading_label_2 = pyglet.text.Label('45',
                          font_size=36/2,
                          x=heading_center_x + 45*heading_scale,
                          y=compass_rect.y+compass_rect.height/2,
                          anchor_y='center', anchor_x='center')
    heading_label_3 = pyglet.text.Label('45',
                          font_size=36/2,
                          x=heading_center_x + 45*heading_scale,
                          y=compass_rect.y+compass_rect.height/2,
                          anchor_y='center', anchor_x='center')
    heading_label_4 = pyglet.text.Label('45',
                          font_size=36/2,
                          x=heading_center_x + 45*heading_scale,
                          y=compass_rect.y+compass_rect.height/2,
                          anchor_y='center', anchor_x='center')
    heading_label_5 = pyglet.text.Label('45',
                          font_size=36/2,
                          x=heading_center_x + 45*heading_scale,
                          y=compass_rect.y+compass_rect.height/2,
                          anchor_y='center', anchor_x='center')
    heading_label_6 = pyglet.text.Label('45',
                          font_size=36/2,
                          x=heading_center_x + 45*heading_scale,
                          y=compass_rect.y+compass_rect.height/2,
                          anchor_y='center', anchor_x='center')
    
    #fps_display = pyglet.clock.ClockDisplay()
    
    #draw_back = True
    
except Exception as e:
    ex_stop(e)



def on_draw():
    global ahdata
    global draw_back
    try:
        #if draw_back:
        window.clear()
        #print("draw background")
        #pitch_batch.draw()
            #draw_back=False
        
        print(pyglet.clock.get_fps())
    
        
        ahdata = msgthd.getAharsData(ahdata)
        
        
        pitch_y = ahdata.pitch*pitch_scale
        
        ### NOTE: this could be a single, two color image
        # rotated and x postion set in one draw call
        top_rect.rotation = -ahdata.roll
        top_rect.position = (center_x, center_y-pitch_y)
        top_rect.draw()
    
        bot_rect.rotation = -ahdata.roll
        bot_rect.position = (center_x, center_y-pitch_y)
        bot_rect.draw()

        roll_label.text = str(round(abs(ahdata.roll), 2)) 
        roll_label.draw()
    
        pitch_label.text = str(round(ahdata.pitch, 2)) 
        pitch_label.draw()
        
        
        draw_pitch_ticks(pitch_tick_list, ahdata.roll, (center_x, center_y), pitch_5_y, 50, -pitch_y)
        pitch_batch.draw()
           
        pt1 = rotate_point((center_x, roll_top), -ahdata.roll, center_point=(center_x, arc_center))
        pt2 = rotate_point((center_x, roll_top-100), -ahdata.roll, center_point=(center_x, arc_center))
        roll_line.position = pt2[0], pt2[1], pt1[0], pt1[1]
        roll_line.draw()
        
        #heading_label_rect.draw()
        #heading_label.text = str(round(abs(ahdata.heading)))
        #heading_label.draw()
        
        heading_origin = get_90_origin(ahdata.heading)
                
        nxt = get_tick_angle(heading_origin, 1, 30)
        heading_label_1.text = get_heading_str(nxt)
        org_offset = angle_dif_right(heading_origin, nxt)
        heading_label_1.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
        heading_label_1.color = get_heading_color(nxt)
        heading_label_1.draw()
        
        nxt = get_tick_angle(heading_origin, 2, 30)
        heading_label_2.text = get_heading_str(nxt)
        org_offset = angle_dif_right(heading_origin, nxt)
        heading_label_2.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
        heading_label_2.color = get_heading_color(nxt)
        heading_label_2.draw()
        
        nxt = get_tick_angle(heading_origin, 3, 30)
        heading_label_3.text = get_heading_str(nxt)
        org_offset = angle_dif_right(heading_origin, nxt)
        heading_label_3.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
        heading_label_3.color = get_heading_color(nxt)
        heading_label_3.draw()
        
        nxt = get_tick_angle(heading_origin, 4, 30)
        heading_label_4.text = get_heading_str(nxt)
        org_offset = angle_dif_right(heading_origin, nxt)
        heading_label_4.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
        heading_label_4.color = get_heading_color(nxt)
        heading_label_4.draw()
        
        nxt = get_tick_angle(heading_origin, 5, 30)
        heading_label_5.text = get_heading_str(nxt)
        org_offset = angle_dif_right(heading_origin, nxt)
        heading_label_5.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
        heading_label_5.color = get_heading_color(nxt)
        heading_label_5.draw()
        
        nxt = get_tick_angle(heading_origin, 6, 30)
        heading_label_6.text = get_heading_str(nxt)
        org_offset = angle_dif_right(heading_origin, nxt)
        heading_label_6.x = int(compass_rect.x - compass_rect.width/2) + int(heading_scale*org_offset)
        heading_label_6.color = get_heading_color(nxt)
        heading_label_6.draw()
        
        heading_label_rect.draw()
        heading_label.text = get_heading_str(round(abs(ahdata.heading)))
        heading_label.draw()
        
        
        clip_batch.draw()
        left_clip_rect.draw()
        rgt_clip_rect.draw()
        
        
        
    except Exception as e:
        ex_stop(e)
 
@window.event
def on_key_press(symbol, modifiers):
    pyglet.image.get_buffer_manager().get_color_buffer().save('screenshot1.png')   
    
def on_draw1():
    global ahdata
    window1.clear()
    fps_display.draw()
    #ahdata = pix_hawk_msg.aharsData(-1,-1,-1)
    ahdata = msgthd.getAharsData(ahdata)
    yaw = ahdata.heading
    #yaw = getyaw()
    #yaw = yaw2deg(yaw)
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
    print("dt: ", dt)
    
    

window.on_draw = on_draw
#window1.on_draw = on_draw1
 
pyglet.clock.schedule_interval(update, .1)

#ahars()

pyglet.app.run()

