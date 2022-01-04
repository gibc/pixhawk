
from re import S
from threading import Thread, Lock
import pyaudio
import numpy as np
from numpy import arange, float32



import time


class WaveData():
    def __init__(self):
        self.channels = 1
        self.freq = 660
        self.chunk = 1024
        self.rate = int(48000)
        self.last_chunck = 0
        self.volume = .5
        #self.data = self.get_chunk()
        

    def sine(self):
        length = self.chunk
        factor = float32(self.freq)*2*np.pi/self.rate
        this_chunk = np.arange(length)+self.last_chunck
        self.last_chunck = this_chunk[-1]
        return np.sin(this_chunk*factor)

    def get_chunk(self):
        data = self.sine()
        return data * self.volume

    def set_volume(self, volume):
        if volume > 1:
            volume = 1
        if volume < 0:
            volume = 0
        self.volume = volume
    
    def call_back(self, in_data, frame_count, time_info, status):
        chunk = self.get_chunk() * .35 # .2
        data = chunk.astype(np.float32).tobytes()
        return (data, pyaudio.paContinue)
        
        


class SoundThread(Thread):

    _instance = None
    _instance_count = 0
    _run_thread = True

    def __init__(self):
        Thread.__init__(self)
        self.wave_data = WaveData()
        self.p = pyaudio.PyAudio()
        #device_count = self.p.get_device_info_by_index(2)
        self.stream = self.p.open(format=pyaudio.paFloat32,
                         channels=1,
                         rate=self.wave_data.rate,
                         output=True,
                         stream_callback = self.wave_data.call_back,
                         output_device_index=2
                         )

    def start_tone(self, volume):
        if self.stream.is_active():
            self.wave_data.set_volume(volume)
            return
        else:
            self.wave_data.set_volume(volume)
            self.stream.start_stream()


    def stop_tone(self):
        if self.stream.is_active():
            self.stream.stop_stream()

    def run(self):
        print('SoundThread started')
        while SoundThread._run_thread:
            time.sleep(.5)
        print('SoundThread terminated')

    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            cls._instance = SoundThread()
            cls._run_thread = True
            cls._instance.start()
        cls._instance_count += 1
        return cls._instance
    
    @classmethod
    def put_instance(cls):
        cls._instance_count -= 1
        if cls._instance_count <= 0:
            cls._run_thread = False
            cls._instance = None
        

if __name__ == '__main__':
    st = SoundThread.get_instance()
    st.start_tone(.1)
    time.sleep(3)
    st.stop_tone()
    time.sleep(1)
    st.start_tone(1)
    time.sleep(3)
    SoundThread.put_instance()