
import re
from threading import Thread, Lock, Timer
import time
import termios
import sys
import select
import tty


class KeyBoard(Thread):
    _instance = None
    _instance_count = 0
    _run_thread = True
    def __init__(self):
        Thread.__init__(self)
    
        self.key_buf = None
        self.keylock = Lock()
        self.time_out = False
    
    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            cls._instance = KeyBoard()
            cls._instance.start()

        cls._instance_count += 1    
        return cls._instance

    def run(self):

        print("started KeyBoard thread")

        while KeyBoard._run_thread:
        
            #key = input()
            #key = getch.getche()
            key = self.getch()
            #print('keyboard got key ', key)
            self.set_key(key)
        print('end kbd thread')

    @classmethod
    def stop(cls):
        cls._instance_count -= 1
        if cls._instance_count == 0:
            cls._run_thread = False
            cls._instance = None
        
    def get_key(self):
        with self.keylock:
            key = self.key_buf
            self.key_buf = None
            return key
    
    def wait_key(self, timeout = 10):
        self.time_out = False
        timer = Timer(timeout, self.end_fun)
        timer.start()
        #key = None
        while self.time_out == False:
            print('.', end='', flush=True)
            key = self.get_key()
            if key != None:
                timer.cancel()
                return key
            time.sleep(.05)
        return None
    
    # call by timer
    def end_fun(self):
        self.time_out = True

    def set_key(self, key):
        with self.keylock:
            self.key_buf = key

    def getch(self):
        
        def _getch():
            ch = None
            fd = sys.stdin.fileno()    
            old_settings = termios.tcgetattr(fd)
            try:
                #tty.setraw(fd)
                tty.setcbreak(fd)
                if select.select([sys.stdin,],[],[], 0.5)[0]:
                    #print('sys.stdin.read(1)')
                    ch = sys.stdin.read(1)
                else:
                    ch = None
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                #termios.tcsetattr(fd, termios.TCSANOW, old_settings)
            #sys.stdout.write(ch)   
            #print(ch)
            return ch
            
        return _getch()


if __name__ == '__main__':
    kbd = KeyBoard.get_instance()
    
    key = kbd.wait_key(timeout=30)
    print('wait kye ', key)
    i= 0
    while i in range(0, 20):
        key = kbd.get_key()
        if key != None:
            print(key)
        time.sleep(.1)
        i += 1
    KeyBoard.stop()

    
    """while True:
        key = kbd.get_key()
        if key != None:
            print('got key: ', key)
        key = kbd.wait_key(timeout=3)
        if key == None:
            print('\nwait key time out')
        else:
            print('\ngot wait key: ', key)
        time.sleep(.1)"""
    