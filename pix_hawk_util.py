
from threading import Thread, Lock
import time
#from typing_extensions import final
#import getch

class KeyBoard(Thread):
    _instance = None
    def __init__(self):
        Thread.__init__(self)
    
        self.run_thread = True
        self.key_buf = None
        self.keylock = Lock()
    
    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            _instance = KeyBoard()
            _instance.start()
        return _instance

    def run(self):

        print("started KeyBoard thread")

        while self.run_thread:
        
            #key = input()
            #key = getch.getche()
            key = self.getch()
            print('keyboard got key ', key)
            self.set_key(key)

    def get_key(self):
        with self.keylock:
            key = self.key_buf
            self.key_buf = None
            return key

    def set_key(self, key):
        with self.keylock:
            self.key_buf = key

    def getch(self):
        import termios
        import sys, tty
        def _getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                #tty.setraw(fd)
                tty.setcbreak(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                #termios.tcsetattr(fd, termios.TCSANOW, old_settings)
                
            return ch
        return _getch()


if __name__ == '__main__':
    kbd = KeyBoard.get_instance()
    while True:
        key = kbd.get_key()
        if key != None:
            print('got key: ', key)
        time.sleep(.1)
    