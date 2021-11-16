from math import radians
import os
import numpy as np
import math
from pix_hawk_util import KeyBoard, Math
import matplotlib.pyplot as plt
import traceback

class CompassOps():
    def __init__(self, param_file):
        self.home_dir = '/home/pi/PhidgetInsurments/'
        self.param_file = self.home_dir + param_file
        #self.data_file = self.home_dir + data_file
        
        self.e_angle = 0
        self.e_width = 0
        self.e_height = 0
        self.e_center_x = 0
        self.e_center_y = 0
        self.xmag_list = []
        self.ymag_list = []
        self.zmag_list = []
        self.xacc_list = []
        self.yacc_list = []
        self.zacc_list = []
        self.xmag_array = None
        self.ymag_array = None
        self.zmag_array = None
        self.xacc_array = None
        self.yacc_array = None
        self.zacc_array = None
        self.old_xmag_average = 0
        self.old_ymag_average = 0


        self.get_params()

    def get_params(self):

        if os.path.isfile(self.param_file) :
            data_file = open(self.param_file, 'r')
            data = data_file.readlines()
            for line in data:
                item = line.split()
                if item[0].strip(' ,') == 'center_x':
                    self.e_center_x = float(item[1].strip(' ,'))
                if item[0].strip(' ,') == 'center_y':
                    self.e_center_y = float(item[1].strip(' ,'))
                if item[0].strip(' ,') == 'width':
                    self.e_width = float(item[1].strip(' ,'))
                if item[0].strip(' ,') == 'height':
                    self.e_height = float(item[1].strip(' ,'))
                if item[0].strip(' ,') == 'angle':
                    self.e_angle = float(item[1].strip(' ,'))
            print('loaded params from {0}'.format(self.param_file))
        else: 
            raise Exception('compass parameter file not found')

        """elif os.path.isfile(self.data_file) :

            data_file = open(self.data_file, 'r')
            data = data_file.readlines()
            items = None
            self.xmag_list = []
            self.ymag_list = []
            self.zmag_list = []
            for line in data:
                items = line.split()
                self.xmag_list.append(float(items[1].strip(' ,')))
                self.ymag_list.append(float(items[2].strip(' ,')))
                self.zmag_list.append(float(items[3].strip(' ,')))

                self.xacc_list.append(float(items[4].strip(' ,')))
                self.yacc_list.append(float(items[5].strip(' ,')))
                self.zacc_list.append(float(items[6].strip(' ,')))
            
            self.xmag_array = np.array(self.xmag_list)
            self.ymag_array = np.array(self.ymag_list)
            self.zmag_array = np.array(self.zmag_list)
            
            self.xacc_array = np.array(self.xacc_list)
            self.yacc_array = np.array(self.yacc_list)
            self.zacc_array = np.array(self.zacc_list)
            self.estimate_ellipe(self.xmag_array, self.ymag_array)
            
        else:
            raise Exception('compass data files not found')"""
    
    def _get_heading(self, xmag, ymag, zmag, pitch, roll):
        
        #TBD: add un tilt, add declination
        xmag = self.xmag_average(xmag, 10)
        ymag = self.ymag_average(ymag, 10)

        #un_tilt_pt = self.un_tilt_mag(xmag,ymag,zmag, pitch, roll)
        un_tilt_pt = self.berryl_tilt(xmag,ymag,zmag, pitch, roll, 2)
        xmag = un_tilt_pt[0]
        ymag = un_tilt_pt[1]

        fpt = self.fix_point((xmag,ymag))
        xmag = fpt[0]
        ymag = fpt[1]

        heading = self.mag2heading(fpt[0], fpt[1])
        #print('heading {0} xmag {1} ymag {2} zmag {3} pitch {4} roll {5}'.format(int(heading), xmag,ymag,zmag,pitch,roll))
        print('heading {0} xmag {1} ymag {2} '.format(int(heading), int(xmag),int(ymag),zmag,pitch,roll))
        return heading

    def mag2heading(self, xmag, ymag):

        

        heading = math.degrees(math.atan(xmag/ymag))
        if ymag > 0:
            heading = 90 - heading
        else:
            heading = 270 - heading
        if ymag == 0:
            if xmag < 0:
                heading = 180
            else:
                heading = 0
        heading = heading
        if heading > 360:
            heading = heading - 360

        heading -= 90
        if heading < 0:
            heading = 360 + heading
        
        #declination = 4.75
        heading -= 8.75

        return heading

    def xmag_average(self, new_val, n):
        xmag_average = self.old_xmag_average * (n-1)/n + new_val/n
        self.old_xmag_average = xmag_average
        return xmag_average

    def ymag_average(self, new_val, n):
        ymag_average = self.old_ymag_average * (n-1)/n + new_val/n
        self.old_ymag_average = ymag_average
        return ymag_average

    def fix_point(self, point):
        f_point_x = point[0] - self.e_center_x
        f_point_y = point[1] - self.e_center_y
        rpt = Math.rotate_point((f_point_x,f_point_y), self.e_angle)

        radius = (self.e_width + self.e_height) / 2

        pol_pt = Math.cart2pol(rpt[0],rpt[1])
        mag = pol_pt[0]
        ang = pol_pt[1]
        delta = radius - mag

        mag = mag+delta
        npt = Math.pol2cart(mag, ang)

        return (npt[0], npt[1])
    
    def estimate_ellipe(self, x, y):
        from ellipse_fit import LsqEllipse

        try:
            fitter = LsqEllipse()
            narra = np.vstack((x,y)).T

            fitter.fit(narra)
            param = fitter.as_parameters()
            #center, width, height, phi
            print('center x ', param[0][0])
            print('center y ', param[0][1])
            self.e_center_x = param[0][0]
            self.e_center_y = param[0][1]

            center = param[0]

            print('width ', param[1])
            ht = param[1]
            self.e_width = param[1]

            print('height ', param[2])
            wd = param[2]
            self.e_height = param[2]  # was bug, set to wd

            print('angle ', np.degrees(param[3]))
            angle = np.degrees(param[3])
            self.e_angle = angle
            
            epts = fitter.return_fit(50)
            fX = epts[:,0:1]
            fY = epts[:,1:]

            x1 = center[0]-wd
            x2 = center[0]+wd
            y1 = center[1]
            y2 = center[1]

            r_line = Math.rotate_line((x1,y1), (x2,y2), -angle, center_point=center)
            r_pt1 = r_line[0]
            r_pt2 = r_line[1]

            x3 = center[0]
            x4 = center[0]
            y3 = center[1]-ht
            y4 = center[1]+ht

            r_line_ = Math.rotate_line((x3,y3), (x4,y4), -angle, center_point=center)
            r_pt1_ = r_line_[0]
            r_pt2_ = r_line_[1]

            

            plt.plot(fX, fY, color=('r'))
            plt.scatter(x, y, color=('g'))
            plt.scatter(center[0], center[1], color=('b'))
            
            plt.plot([r_pt1[0],r_pt2[0]],  [r_pt1[1],r_pt2[1]], color= ('m'))
            plt.plot([r_pt1_[0],r_pt2_[0]],  [r_pt1_[1],r_pt2_[1]], color= ('m'))
            
            plt.show()

            fix_ptsx = []
            fix_ptsy = []
            heading = []
            for point in narra:
                fpt = self.fix_point(point)
                fix_ptsx.append(fpt[0])
                fix_ptsy.append(fpt[1])
                heading.append(self.mag2heading(fpt[0], fpt[1]))


            plt.scatter(fix_ptsx, fix_ptsy, color=('b'))
            plt.show()

            #xvals = list(range(0, len(heading)))
            
            #plt.plot(xvals, heading, color=('g'))
            #plt.show()

            self.polar_plot(heading, 'corrected heading')

            #write param file
            param_file = open(self.param_file, 'w')
            param_file.write('center_x, ' + str(self.e_center_x) +'\n')
            param_file.write('center_y, ' + str(self.e_center_y) +'\n')    
            param_file.write('width, ' + str(self.e_width) +'\n')   
            param_file.write('height, ' + str(self.e_height) +'\n')   
            param_file.write('angle, ' +str(self.e_angle) +'\n') 
            param_file.close()

            return


        except Exception:
            traceback.print_exc()

    def berryl_tilt(self, MAGx, MAGy, MAGz, pitch, roll, BerryIMUversion =1 ):

        #Calculate the new tilt compensated values
        #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
        #This needs to be taken into consideration when performing the calculations

        #X compensation
        print('pitch {0} roll {1}'.format(pitch, roll))
        
        pitch = radians(pitch)
        """NOTE: had to invert sign of roll to make this work"""
        roll = -radians(roll)

        if(BerryIMUversion == 1 or BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

        #Y compensation
        if(BerryIMUversion == 1 or BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

        return (magXcomp, magYcomp)
        
    def un_tilt_mag(self, magx, magy, magz, pitch, roll):

    
        #print('roll ', self.roll)
        #print('pitch ', self.pitch)
        pitch_rad = radians(pitch)
        roll_rad = radians(roll)
        magx_corrected = magx * math.cos(pitch_rad) + magz * math.sin(pitch_rad)

        """NOTE: had to invert sign of roll to make this work"""
        magy_corrected = magx * math.sin(-roll_rad) * math.sin(pitch_rad) +  \
                    magy * math.cos(-roll_rad) - \
                    magz * math.sin(-roll_rad) * math.cos(pitch_rad)
        
        """magy_corrected = magx * math.sin(self.pitch_rad)  +  \
                    magy * math.sin(self.roll_rad)*math.sin(self.pitch_rad) - \
                    magz * math.cos(self.roll_rad) * math.sin(self.pitch_rad)"""

        return (magx_corrected, magy_corrected)
    
    def polar_plot(self, ang_list, title):
        
        #r = np.arange(0, 2, 0.01)
        r = [1.75] * len(ang_list)
        #theta = 2 * np.pi * r
        theta = []
        for ang in ang_list:
            theta.append(math.radians(ang))

        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        ax.plot(theta, r)
        ax.set_rmax(2)
        ax.set_rticks([0.5, 1, 1.5, 2])  # Less radial ticks
        ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
        ax.grid(True)

        ax.set_title(title, va='bottom')

        plt.show()

    def save_cal_params(self, data_file, param_file):

        data_file = open(data_file, 'r')
        data = data_file.readlines()
        items = None
        self.xmag_list = []
        self.ymag_list = []
        self.zmag_list = []
        for line in data:
            items = line.split()
            self.xmag_list.append(float(items[1].strip(' ,')))
            self.ymag_list.append(float(items[2].strip(' ,')))
            self.zmag_list.append(float(items[3].strip(' ,')))

            self.xacc_list.append(float(items[4].strip(' ,')))
            self.yacc_list.append(float(items[5].strip(' ,')))
            self.zacc_list.append(float(items[6].strip(' ,')))
        
        self.xmag_array = np.array(self.xmag_list)
        self.ymag_array = np.array(self.ymag_list)
        self.zmag_array = np.array(self.zmag_list)
        
        self.xacc_array = np.array(self.xacc_list)
        self.yacc_array = np.array(self.yacc_list)
        self.zacc_array = np.array(self.zacc_list)
        
        narra = np.vstack((self.xmag_array, self.ymag_array)).T

        self.estimate_ellipe(self.xmag_array, self.ymag_array)

        #write param file
        param_file = open(param_file, 'w')
        param_file.write('center_x, ' + str(self.e_center_x) +'\n')
        param_file.write('center_y, ' + str(self.e_center_y) +'\n')    
        param_file.write('width, ' + str(self.e_width) +'\n')   
        param_file.write('height, ' + str(self.e_height) +'\n')   
        param_file.write('angle, ' +str(self.e_angle) +'\n') 
        param_file.close()

    def test(self, test_file):

        data_file = open(test_file, 'r')
        data = data_file.readlines()
        items = None
        self.xmag_list = []
        self.ymag_list = []
        self.zmag_list = []
        for line in data:
            items = line.split()
            self.xmag_list.append(float(items[1].strip(' ,')))
            self.ymag_list.append(float(items[2].strip(' ,')))
            self.zmag_list.append(float(items[3].strip(' ,')))

            self.xacc_list.append(float(items[4].strip(' ,')))
            self.yacc_list.append(float(items[5].strip(' ,')))
            self.zacc_list.append(float(items[6].strip(' ,')))
        
        self.xmag_array = np.array(self.xmag_list)
        self.ymag_array = np.array(self.ymag_list)
        self.zmag_array = np.array(self.zmag_list)
        
        self.xacc_array = np.array(self.xacc_list)
        self.yacc_array = np.array(self.yacc_list)
        self.zacc_array = np.array(self.zacc_list)
        
        narra = np.vstack((self.xmag_array, self.ymag_array)).T

        self.estimate_ellipe(self.xmag_array, self.ymag_array)


if __name__ == '__main__':
    cps = CompassOps('mag_params/home_param.txt')
    #cps.test('/home/pi/PhidgetInsurments/mag_data/mag_data_4.txt') 
    #cps.save_cal_params('/home/pi/PhidgetInsurments/mag_data/cal_data.txt', '/home/pi/PhidgetInsurments/mag_params/new_keik_params.txt')   