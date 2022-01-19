

from re import S
from threading import Thread, Lock
import pyaudio
import numpy as np
from numpy import arange, float32
import threading



import time


class WaveData():
    def __init__(self):
        self.channels = 1
        self.freq = 660
        self.chunk = 1024
        self.rate = int(48000)
        self.last_chunck = 0
        self.volume = .5
        self.chunck_list = []
        self.adj_freq()
        self.static_chunk = self.get_chunk()
        
        
    def adj_freq(self):
        sec_per_sample = 1/self.rate
        chunk_preiod = self.chunk * sec_per_sample
        freq_period = 1/self.freq
        cycles_per_chunk = chunk_preiod / freq_period
        error = cycles_per_chunk % int(cycles_per_chunk)
        cycles_per_chunk = int(cycles_per_chunk)
        
        freq_period2 = self.chunk / cycles_per_chunk * sec_per_sample
        adj_freq = 1/freq_period2
        self.freq = adj_freq
        #while error > .0001:
        #    freq_period = 1/self.freq
        #    cycles_per_chunk = chunk_preiod / freq_period
        #    error = cycles_per_chunk % cycles_per_chunk

    def sine(self):
        length = self.chunk
        factor = float32(self.freq)*2*np.pi/self.rate
        this_chunk = np.arange(length)+self.last_chunck #numbers 0 to 1023 and add last chunk to each element
        self.last_chunck = this_chunk[-1] #save last element of chunck
        #print(self.last_chunck)
        return np.sin(this_chunk*factor)

    def get_chunk(self):
        data = self.sine()
        return data * self.volume

    def get_static_chunk(self):
        return self.static_chunk * self.volume


    def set_volume(self, volume):
        if volume > 1:
            volume = 1
        if volume < 0:
            volume = 0
        self.volume = volume
    
    def call_back(self, in_data, frame_count, time_info, status):
        #chunk = self.get_chunk() * .35 # .2
        chunk = self.get_static_chunk() * .35
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
        self.stream.stop_stream()
        self.beep_on = False
        self.tone_mode = False
        self.beep_volume = 0
        self.beep_on_secs = 0
        self.beep_off_secs = 0
        self.lock = Lock()

    def start_tone(self, volume):
        print('*************************** start tone')
        with self.lock:
            if self.stream.is_active():
                self.wave_data.set_volume(volume)
                return
            else:
                self.wave_data.set_volume(volume)
                self.stream.start_stream()

                if not self.stream.is_active():
                    print('audio stream start failed')
                    raise Exception('audio stream start failed')
                


    def stop_tone(self):
        print('*************************** stop tone')
        with self.lock:
            if self.stream.is_active():
                self.stream.stop_stream()

    
    def set_tone_mode(self, mode):
        with self.lock:
            self.tone_mode = mode

    def get_tone_mode(self):
        with self.lock:
            return self.tone_mode

    
    def start_beep(self, volume, on_secs, off_secs):
        #print('start_beep+++++++++++++++++++++++++')
        with self.lock:
            self.beep_volume = volume
            self.beep_on_secs = on_secs
            self.beeb_off_secs = off_secs
            self.beep_on = True

    def set_beep(self, volume, on_secs, off_secs):
        #print('set_beep+++++++++++++++++++++++++')
        with self.lock:
            self.beep_volume = volume
            self.beep_on_secs = on_secs
            self.beeb_off_secs = off_secs

    def get_beep_on_secs(self):
        with self.lock:
            return self.beep_on_secs

    def get_beep_off_secs(self):
        with self.lock:
            return self.beep_off_secs

        
                

    def stop_beep(self):
        with self.lock:
            self.beep_on = False
    
    def get_beep_on(self):
        with self.lock:
            return self.beep_on

        

    def run(self):
        print('SoundThread started')
        while SoundThread._run_thread:
            #print('sound thread running++++++++++++++++++++++++++')
            if self.get_beep_on() and not self.get_tone_mode():
                self.start_tone(self.beep_volume)
                time.sleep(self.get_beep_on_secs())
                if self.get_beep_on() and not self.get_tone_mode():
                    self.stop_tone()
                    time.sleep(self.get_beep_off_secs())
            else:
                time.sleep(.5)
                if not self.get_beep_on() and not self.get_tone_mode():
                    self.stop_tone()
                
        print('SoundThread terminated')

    @classmethod
    def get_instance(cls):
        if cls._instance == None:
            cls._instance = SoundThread()
            cls._run_thread = True
            #cls._instance.start()
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
    
    st.start_beep(.3, .25, .15)
    time.sleep(2)

    st.set_tone_mode(True)
    st.start_tone(.1)
    time.sleep(3)
    st.stop_tone()
    st.set_tone_mode(False)
    time.sleep(2)
    st.stop_beep()


    SoundThread.put_instance()