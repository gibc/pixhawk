#from numpy.core.numeric import roll
from math import radians

from numpy.core.records import array
from pix_hawk_msg import mavlinkmsg
from pix_hawk_msg import aharsData
from pix_hawk_util import KeyBoard, Math
from threading import Thread, Lock
import time
import os
import numpy as np
print(np.__version__)
print(np.__path__)   
import matplotlib.pyplot as plt
import traceback
from pix_hawk_util import Math
import math

#import matplotlib
#import numpy as np
#import skimage
# curses

class Compass():
    
    def __init__(self):
        self.msg_thread = mavlinkmsg.get_instance()
        self.ahd = aharsData()
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
        self.key_board = KeyBoard.get_instance()

        self.e_angle = 0
        self.e_width = 0
        self.e_height = 0
        self.e_center_x = 0
        self.e_center_y = 0
        
    def get_data_loop(self):
        while self.get_mag_data():
            pass


    def get_mag_data(self):
        print('\npress any key to START compass data collection.')
        #key = self.key_board.wait_key(30)
        key = self.key_board.wait_key(-1)
        if key == None:
            return False
        if key == '\n':
            return False

        count = 0
        print('\npress any key to STOP compass data collection.')
        while count < 10000  and self.key_board.get_key() == None:
            count += 1
            self.ahd = self.msg_thread.getAharsData(self.ahd)
            print('\ncount ', count)
            print('xmag ', self.ahd.xmag)
            if not self.ahd.xmag == -1 and not self.ahd.ymag == -1 and not self.ahd.zmag == -1:
                self.xmag_list.append(self.ahd.xmag)
                self.ymag_list.append(self.ahd.ymag)
                self.zmag_list.append(self.ahd.zmag)
                self.xacc_list.append(self.ahd.xacc)
                self.yacc_list.append(self.ahd.yacc)
                self.zacc_list.append(self.ahd.zacc)
            
            time.sleep(.1)
        
        print('len xmag list', len(self.xmag_list))
        print('len ymag list', len(self.ymag_list))
        print('len zmag list', len(self.zmag_list))

        print('len xacc list', len(self.xacc_list))
        print('len yacc list', len(self.yacc_list))
        print('len zacc list', len(self.zacc_list))

        fn = self.get_file_name()

        #data_file = open('mag_data.txt', 'w')
        data_file = open(fn, 'w')
        for i in range(0, len(self.xmag_list)):
            data_file.write(
                'V, '+
                str(self.xmag_list[i]) + ', ' + 
                str(self.ymag_list[i]) + ', ' + 
                str(self.zmag_list[i]) + ', ' +
                str(self.xacc_list[i]) + ', ' + 
                str(self.yacc_list[i]) + ', ' + 
                str(self.zacc_list[i]) + '\n'
                
                )
        data_file.close()
        return True
    
    def is_file(self, fn):
        
        if os.path.isfile(fn) :
            #print('Could not open input file ' + fn)
            return True
        return False
        #try:
            #f = open(fn,'w')
            #lines = f.readlines()
            #f.close()
            #return False
        #except:
            #print ('Could not open input file ' + fn)
        #return rc
    
    def get_file_name(self):
        fn = '/home/pi/PhidgetInsurments/mag_data/mag_data_0.txt'
        file_num = 0
        while self.is_file(fn):
            file_num += 1 
            fn = '/home/pi/PhidgetInsurments/mag_data/mag_data_' + str(file_num) + '.txt'
        return fn
    
    def make_file_name(self, file_num):
        fn = '/home/pi/PhidgetInsurments/mag_data/mag_data_' + str(file_num) + '.txt'
        return fn


    def chart_data(self):
        file_num = 0
        while True:
            fn = self.make_file_name(file_num)
            if not self.is_file(fn):
                break
            
            data_file = open(fn, 'r')
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

            #self.time_serise_chart('xmag', self.xmag_list)
            #self.time_serise_chart('ymag',self.ymag_list)
            #self.time_serise_chart('zmag',self.ymag_list)
            self.estimate_ellipe(self.xmag_array, self.ymag_array)
        
            #self.ellipsoid_iterate()

            #self.scatter_plot('xmag vs ymag', self.xmag_list, self.ymag_list)
            #self.scatter_plot('xmag vs zmag', self.xmag_list, self.zmag_list)
            #self.scatter_plot('ymag vs zmag', self.ymag_list, self.zmag_list)


            data_file.close()
            file_num += 1
            

    def time_serise_chart(self, title, data):
        x = list(range(0,len(data)))
        
        plt.plot(x,data)
        plt.title(title)
        plt.show()

    def scatter_plot(self, name, xdata, ydata):
        plt.scatter(xdata, ydata)
        plt.title(name)
        plt.show()
    
    #gib - get centers
    def estimateCenter3D(self):

        # Slice off the component arrays
        #xx=arr[:,0]
        #yy=arr[:,1]
        #zz=arr[:,2]
        xx = self.xmag_list
        yy = self.ymag_list
        zz = self.zmag_list
   
        #average point is centered sufficiently with well sampled data
        center=np.array([np.mean(xx),np.mean(yy),np.mean(zz)])
        print('xmag center ', center[0])
        print('ymag center ', center[1])
        print('zmag center ', center[2])
      
        #Center the samples
        xc=xx-center[0]
        yc=yy-center[1]
        zc=zz-center[2]
   
        # Calculate distance from center for each point 
        rc = np.sqrt(xc*xc + yc*yc + zc*zc)
        # Take the average
        radius = np.mean(rc)
        print('radius ', radius)
   
        std = np.std(rc)
        print('std ', std)
      
        return (center,radius,std)

    def normalize3(self, xyz):
        x=xyz[:,0]
        y=xyz[:,1]
        z=xyz[:,2]
        #x=xyz[0]
        #y=xyz[1]
        #z=xyz[2]
        rarr = np.sqrt(x*x + y*y + z*z)
        ravg=np.mean(rarr)
        xyzn=xyz/ravg
        return (xyzn,ravg)
    
    def mgDot(self, mag,acc):
        ll=len(mag)
        mdg=np.zeros(ll)
        for ix in range(ll):
            mdg[ix]=np.dot(mag[ix],acc[ix].T)
        # print mdg   
        avgMdg=np.mean(mdg)
        stdMdg=np.std(mdg)
        # print avgMdg,stdMdg/avgMdg
        return (avgMdg,stdMdg)

    def applyParams12(self,xyz,params):
        # first three of twelve parameters are the x,y,z offsets
        ofs=params[0:3]
        # next nine are the components of the 3x3 transformation matrix
        mat=np.reshape(params[3:12],(3,3))
        # subtract ofs
        xyzcentered=xyz-ofs
 
        xyzout=np.dot(mat,xyzcentered.T).T
   
        return xyzout

    def magDotAccErr(self,mag,acc,mdg,params):
        #offset and transformation matrix from parameters
        ofs=params[0:3]
        mat=np.reshape(params[3:12],(3,3))
        #subtract offset, then apply transformation matrix
        mc=mag-ofs
        mm=np.dot(mat,mc)
        #calculate dot product from corrected mags
        mdg1=np.dot(mm,acc)
        err=mdg-mdg1
        return err
    
    def analyticPartialRow(self, mag,acc,target,params):
        err0=self.magDotAccErr(mag,acc,target,params)
        # ll=len(params)
        slopeArr=np.zeros(12)
        slopeArr[0]=  -(params[3]*acc[0] + params[ 4]*acc[1] + params[ 5]*acc[2])
        slopeArr[1]=  -(params[6]*acc[0] + params[ 7]*acc[1] + params[ 8]*acc[2])
        slopeArr[2]=  -(params[9]*acc[0] + params[10]*acc[1] + params[11]*acc[2])
   
        slopeArr[ 3]= (mag[0]-params[0])*acc[0]
        slopeArr[ 4]= (mag[1]-params[1])*acc[0]
        slopeArr[ 5]= (mag[2]-params[2])*acc[0]
   
        slopeArr[ 6]= (mag[0]-params[0])*acc[1]
        slopeArr[ 7]= (mag[1]-params[1])*acc[1]
        slopeArr[ 8]= (mag[2]-params[2])*acc[1]
   
        slopeArr[ 9]= (mag[0]-params[0])*acc[2]
        slopeArr[10]= (mag[1]-params[1])*acc[2]
        slopeArr[11]= (mag[2]-params[2])*acc[2]
   
        return (err0,slopeArr)

    def param9toOfsMat(self, params):
        ofs=params[0:3]
        mat=np.zeros(shape=(3,3))
   
        mat[0,0]=params[3]
        mat[1,1]=params[4]
        mat[2,2]=params[5]

        mat[0,1]=params[6]
        mat[0,2]=params[7]
        mat[1,2]=params[8]

        mat[1,0]=params[6]
        mat[2,0]=params[7]
        mat[2,1]=params[8]
        # print ofs,mat
        return (ofs,mat)

    def radiusErr(self,mag,target,params):
        #offset and transformation matrix from parameters
        (ofs,mat)=self.param9toOfsMat(params)
   
        #subtract offset, then apply transformation matrix
        mc=mag-ofs
        mm=np.dot(mat,mc)

        radius = np.sqrt(mm[0]*mm[0] +mm[1]*mm[1] + mm[2]*mm[2] )
        err=target-radius
        return err

    def magDotAccErr(self, mag,acc,mdg,params):
        #offset and transformation matrix from parameters
        ofs=params[0:3]
        mat=np.reshape(params[3:12],(3,3))
        #subtract offset, then apply transformation matrix
        mc=mag-ofs
        mm=np.dot(mat,mc)
        #calculate dot product from corrected mags
        mdg1=np.dot(mm,acc)
        err=mdg-mdg1
        return err    

    def errorEstimate(self, magN,accN,target,params):
        err2sum=0
        nsamp=len(magN)
        for ix in range(nsamp):
            err=self.magDotAccErr(magN[ix],accN[ix],target,params)
            err2sum += err*err
        # print "%10.6f" % (err)
        sigma=np.sqrt(err2sum/nsamp)  
        return sigma
    
    def errorEstimateSymmetric(self, mag,target,params):
        err2sum=0
        nsamp=len(mag)
        for ix in range(nsamp):
            err=self.radiusErr(mag[ix],target,params)
            err2sum += err*err
            # print "%10.6f" % (err)
        sigma=np.sqrt(err2sum/nsamp)  
        return sigma  
    
    def plotmag(self, name, mag):
        xmagl=[]
        ymagl=[]
        
        for item in mag:
            xmagl.append(item[0])
            ymagl.append(item[1])
        self.scatter_plot(name, xmagl, ymagl)

    # =============================
    #gib - get cal vals
    def ellipsoid_iterate(self):

        #mag = [self.xmag_array, self.ymag_array, self.zmag_array]
        #accel = [self.xacc_array, self.yacc_array, self.zacc_array]
        mag = np.array([self.xmag_array, self.ymag_array, self.zmag_array])
        accel = np.array([self.xacc_array, self.yacc_array, self.zacc_array])
        mag = np.transpose(mag)
        accel = np.transpose(accel)
        verbose = 1
      
        # magCorrected=copy.deepcopy(mag)
        # Obtain an estimate of the center and radius
        # For well distributed samples, the average of all points is sufficient
   
        (centerE,magR,magSTD)=self.estimateCenter3D()
   
        #Work with normalized data
        magScaled=mag/magR
        centerScaled = centerE/magR
   
        (accNorm,accR)=self.normalize3(accel)
   
        params=np.zeros(12)
        #Use the estimated offsets, but our transformation matrix is unity
        params[0:3]=centerScaled
        mat=np.eye(3)
        params[3:12]=np.reshape(mat,(1,9))

        #initial dot based on centered mag, scaled with average radius
        magCorrected=self.applyParams12(magScaled,params)
        self.plotmag('inital', magCorrected)
        

        (avgDot,stdDot)=self.mgDot(magCorrected,accNorm)

        nSamples=len(magScaled)

        sigma = self.errorEstimate(magScaled,accNorm,avgDot,params)
        if verbose: print ('Initial Sigma',sigma)
   
        # pre allocate the data.  We do not actually need the entire
        # D matrix ( a nSamples x 12 matrix ) if we calculate DTD (a 12x12 matrix) within the sample loop
        # Also DTE (dimension 12) can be calculated on the fly. 
  
        D=np.zeros([nSamples,12])
        E=np.zeros(nSamples)
   
        #If numeric derivatives are used, this step size works with normalized data.
        step=np.ones(12)  
        step/=5000
   
        #Fixed number of iterations for testing.  In production you check for convergence
   
        iterations=7 #global Number of iterations 
        nLoops=iterations
        mag_save = None
        for iloop in range(nLoops):
            # Numeric or analytic partials each give the same answer
            for ix in range(nSamples):
            # (f0,pdiff)=numericPartialRow(magScaled[ix],accNorm[ix],avgDot,params,step,1)
                (f0,pdiff)=self.analyticPartialRow(magScaled[ix],accNorm[ix],avgDot,params)
                E[ix]=f0
                D[ix]=pdiff
            #Use the pseudo-inverse   
            DT=D.T 
            DTD=np.dot(DT,D)
            DTE=np.dot(DT,E)
            invDTD=np.linalg.inv(DTD)
            deltas=np.dot(invDTD,DTE)

            #negative sign because of the way we defined the derivatives
            p2=params + deltas
      
      
            sigma = self.errorEstimate(magScaled,accNorm,avgDot,p2)
      
            # add some checking here on the behavior of sigma from loop to loop
            # if satisfied, use the new set of parameters for the next iteration

            params=p2
      
            # recalculste gain (magR) and target dot product
            # not strictly required, the symmetric algorithm does not renormalice each loop
      
            magCorrected=self.applyParams12(magScaled,params)
            mag_save = magCorrected
            #self.plotmag('loop ' + str(iloop), mag_save)
            

            (mc,mcR)=self.normalize3(magCorrected)
            (avgDot,stdDot)=self.mgDot(mc,accNorm)
            magR *= mcR
            magScaled=mag/magR
      
            if verbose: 
                 print ('iloop ',iloop,'sigma',sigma)
                 #print ('iloop',iloop)
        
        self.plotmag('final', mag_save)
   
        return (params,magR)
 
# =============================

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
        return heading
    
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
            self.e_height = param[1]

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

            

            return


        except Exception:
            traceback.print_exc()

        try:

            narra = np.vstack((x,y)).T
            _X = narra[:,0:1]
            _Y = narra[:,1:]


            alpha = 5
            beta = 3
            N = 500
            DIM = 2

            np.random.seed(2)

    # Generate random points on the unit circle by sampling uniform angles
            theta = np.random.uniform(0, 2*np.pi, (N,1))
            eps_noise = 0.2 * np.random.normal(size=[N,1])
            circle = np.hstack([np.cos(theta), np.sin(theta)])

    # Stretch and rotate circle to an ellipse with random linear tranformation
            B = np.random.randint(-3, 3, (DIM, DIM))
            noisy_ellipse = circle.dot(B) + eps_noise

    # Extract x coords and y coords of the ellipse as column vectors
            X = noisy_ellipse[:,0:1]
            Y = noisy_ellipse[:,1:]
            X = _X
            Y = _Y

    # Formulate and solve the least squares problem ||Ax - b ||^2
            A = np.hstack([X**2, X * Y, Y**2, X, Y])
            b = np.ones_like(X)
            x = np.linalg.lstsq(A, b)[0].squeeze()

    # Print the equation of the ellipse in standard form
            print('The ellipse is given by {0:.3}x^2 + {1:.3}xy+{2:.3}y^2+{3:.3}x+{4:.3}y = 1'.format(x[0], x[1],x[2],x[3],x[4]))

    # Plot the noisy data
            plt.scatter(X, Y, label='Data Points')

    # Plot the original ellipse from which the data was generated
            phi = np.linspace(0, 2*np.pi, 1000).reshape((1000,1))
            c = np.hstack([np.cos(phi), np.sin(phi)])
            ground_truth_ellipse = c.dot(B)
            plt.plot(ground_truth_ellipse[:,0], ground_truth_ellipse[:,1], 'k--', label='Generating Ellipse')

    # Plot the least squares ellipse
            x_coord = np.linspace(-5,5,300)
            y_coord = np.linspace(-5,5,300)
            X_coord, Y_coord = np.meshgrid(x_coord, y_coord)
            Z_coord = x[0] * X_coord ** 2 + x[1] * X_coord * Y_coord + x[2] * Y_coord**2 + x[3] * X_coord + x[4] * Y_coord
            plt.contour(X_coord, Y_coord, Z_coord, levels=[1], colors=('r'), linewidths=2, label='eq points')

            plt.legend()
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()

        except Exception:
            traceback.print_exc()

class CompassOps():
    def __init__(self, param_file, data_file):
        self.home_dir = '/home/pi/PhidgetInsurments/'
        self.param_file = self.home_dir + param_file
        self.data_file = self.home_dir + data_file
        
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

        elif os.path.isfile(self.data_file) :

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
            raise Exception('compass data files not found')
    
    def get_heading(self, xmag, ymag, zmag, pitch, roll):
        #TBD: add un tilt, add declination
        un_tilt_pt = self.un_tilt_mag(xmag,ymag,zmag, pitch, roll)
        fpt = self.fix_point((xmag,ymag))

        heading = self.mag2heading(fpt[0], fpt[1])
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
        return heading

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
    
    def test(self, data_file):

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

        narra = np.vstack((self.xmag_array, self.ymag_array)).T

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
            self.e_height = param[1]

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

        


if __name__ == '__main__':
    cps = Compass()  
    # cps.get_mag_data()
    cps.get_data_loop()
    #cps.chart_data()
    
    
    mavlinkmsg.put_instance()
    if not mavlinkmsg._run_thread:
        cps.msg_thread.join()
    
    KeyBoard.stop()
    if not KeyBoard._run_thread:
        Thread.join(cps.key_board)
    exit(0)      