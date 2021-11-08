#from numpy.core.numeric import roll
from pix_hawk_msg import mavlinkmsg
from pix_hawk_msg import aharsData
from pix_hawk_util import KeyBoard
import time
# curses

class Compass():
    def __init__(self):
        self.msg_thread = mavlinkmsg.get_instance()
        self.ahd = aharsData()
        self.xmag_list = []
        self.ymag_list = []
        self.zmag_list = []
        self.key_board = KeyBoard.get_instance()
        

    def get_mag_data(self):
        count = 0
        #while count < 100 and self.key_board.get_key() == None:
        while count < 100 and self.key_board.get_key() == None:
            count += 1
            self.ahd = self.msg_thread.getAharsData(self.ahd)
            print('count ', count)
            print('xmag ', self.ahd.xmag)
            if not self.ahd.xmag == -1 and not self.ahd.ymag == -1 and not self.ahd.zmag == -1:
                self.xmag_list.append(self.ahd.xmag)
                self.ymag_list.append(self.ahd.ymag)
                self.zmag_list.append(self.ahd.zmag)
            
            time.sleep(.1)
        
        print('len xmag list', len(self.xmag_list))
        print('len ymag list', len(self.ymag_list))
        print('len zmag list', len(self.zmag_list))

        data_file = open('mag_data.txt', 'w')
        for i in range(0, len(self.xmag_list)):
            data_file.write(str(self.xmag_list[i]) +', '+ 
                str(self.ymag_list[i]) + ', ' + 
                str(self.zmag_list[i]) + '\n')
        data_file.close()


if __name__ == '__main__':
    cps = Compass()
    cps.get_mag_data()
    #akey = input('press any key to continue')