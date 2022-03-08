
from os import mkfifo
from numpy import asmatrix
import serial
from threading import Lock, Thread
import subprocess
from asyncio.subprocess import PIPE
import time
from pathlib import Path

magic = [0x0a, 0xb0, 0xcd, 0xe0]
class Radio():
    def __init__(self, con_str):
        self.con_str = con_str
        self.run_thread = True
        self.ser = None
        self.r2f_pid = None
        self.snd_pipe = None
        self.rec_pipe = None
        self.radio_thread = Thread(target = self.read_target)
        
        #self.radio_thread.start()
        #self.ser = serial.Serial('/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0', baudrate=2000000, timeout=5)

    def mkpipe(self):
        amode = 0o777
        try:
            mkfifo('/tmp/send_radio', mode=amode)
        except:
            pass

        try:
            mkfifo('/tmp/receive_radio', mode=amode)
            return True
        except:
            return True
        
    def connect(self):
        #self.ser = serial.Serial(self.con_str, baudrate=2000000, timeout=5)
        self.ser = open("/home/pi/PhidgetInsurments/mag_dataadsb_log.txt", "rb")

    def radio2frame(self):
        self.r2f_pid = subprocess.Popen("./radio2frame", stdin=PIPE)
        self.snd_pipe = open('/tmp/send_radio', 'w')
        self.rec_pipe =  open('/tmp/receive_radio', 'r')
        
            
        """ astr = ""
        for i in range(200):
            #h = '0x{:02x}'.format(i)
            h = '{:02x}'.format(i)
            astr += h
        l = len(astr)
        #astr += "\r\n\0"

        while True:
            b = self.pipe.write(astr)
            self.pipe.flush()
            time.sleep(5) """

    def readByte(self, ser):
        while self.run_thread:
            ln = ser.read(1)
            if len(ln) == 0:
                print('timeout\n')
            else:
                byte = ord(ln)
                #print(byte)
                return (byte,True)
        return (0,False)

    def read_target(self):
        print('started radio thread')
        cnt = 0
        while self.run_thread:
        
            byte, ret = self.readByte(self.ser)
            if not ret:
                continue
            if byte == magic[cnt]:
                cnt += 1
                if cnt >= 4:
                    cnt = 0
                    print('got magic\n')
                    lob, ret = self.readByte(self.ser)
                    if not ret:
                        continue
                    hib, ret = self.readByte(self.ser)
                    if not ret:
                        continue

                    msgLen = int(lob) + int(hib<<8) + 5
                    if msgLen > 200:
                        continue

                    print('msglen: {0}\n'.format(msgLen))

                    msg = []
                    msg.append(msgLen)
                    for i in range(msgLen):
                        byte, ret = self.readByte(self.ser)
                        msg.append(byte)
                        
                    extra = 200 - len(msg)
                    for i in range(extra):
                        msg.append(0)

                    astr = ""
                    for i in range(200):
                        h = '{:02x}'.format(msg[i])
                        astr += h


                    self.snd_pipe.write(astr)
                    self.snd_pipe.flush()

                    
                    #vs = self.rec_pipe.read()
                    vs = self.rec_pipe.readline()
                    print(vs)
                    
                    #self.r2f_pid.stdin.write(d)
                    #self.r2f_pid.stdin.flush()
                    cnt = 0
            else:
                cnt = 0

        print('stoped radio thread')
    
    def close(self):
        self.run_thread = False

if __name__ == '__main__':
    # unit test code
    rdo = Radio('/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0')
    if not rdo.mkpipe():
        print('make pipe failed\n')
    rdo.connect()
    rdo.radio2frame()
    rdo.radio_thread.start()
    #time.sleep(30)
    #rdo.close()