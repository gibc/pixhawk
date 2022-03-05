
import decimal
#Play a fixed frequency sound.
#from __future__ import division
import math
import os
import subprocess
import sys
import time
import tty
from asyncio.subprocess import PIPE
from encodings import utf_8
from re import I, M

import matplotlib.pyplot as plt
import numpy as np
import pyaudio
import serial
import simpleaudio as sa
from numpy import arange

magic = [0x0a, 0xb0, 0xcd, 0xe0]

p = subprocess.Popen("./radio2frame", stdin=PIPE)
#p = subprocess.Popen("cat", stdin=PIPE)
#while(True):
    #d = bytearray("some test dat", 'utf_8')
    #d = bytes([1,2,3,4,5])
    #p.communicate(input=d)[0]
    #p.stdin.write(d)
    #p.stdin.flush()
    #time.sleep(1)

def readByte(ser):
    ln = ser.read(1)
    if len(ln) == 0:
        print('timeout\n')
    byte = ord(ln)
    return byte


ser = serial.Serial('/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0', baudrate=2000000, timeout=5)
#ser.open()
cnt = 0
while True:
    #ln = ser.read(1)
    #if len(ln) == 0:
        #print('timeout\n')
        #continue
    #byte = ord(ln)
    byte = readByte(ser)
    if byte == magic[cnt]:
        cnt += 1
        if cnt >= 4:
            cnt = 0
            print('got magic\n')
            lob = readByte(ser)
            hib = readByte(ser)
            msgLen = int(lob) + int(hib<<8) + 5
            print('msglen: {0}\n'.format(msgLen))

            #d = bytearray('got magic\n', 'utf_8')
            #p.stdin.write(d)
            #p.stdin.flush()
    else:
        cnt = 0
    #print(byte)
    
    #print(ln)


filename = '/home/pi/Downloads/beep-07a.wav'
wav_obj = sa.WaveObject.from_wave_file(filename)
wav_obj.play()
quit()

strattime = 0
endtime = 5
sampelrate = 48000
time_ = np.arange(strattime, endtime, 1/sampelrate)
theta = 0
freq = 440
amp = .25
sinewave = amp * np.sin(2 * np.pi * freq * time_ + theta)
plt.plot(sinewave)
plt.show()

# calculate note frequencies
A_freq = 440
Csh_freq = A_freq * 2 ** (4 / 12)
E_freq = A_freq * 2 ** (7 / 12)

# get timesteps for each sample, T is note duration in seconds
sample_rate = int(48000)
T = 0.25
#t = np.linspace(0, T, T * sample_rate, False)
#t = float_range(0, T * sample_rate, T)
t = arange(0, T * sample_rate, T)
#t = np.array(t)

# generate sine wave notes
A_note = np.sin(A_freq * t * 2 * np.pi)
sinewave = amp * np.sin(2 * np.pi * 1000 * time_)


Csh_note = np.sin(Csh_freq * t * 2 * np.pi)
E_note = np.sin(E_freq * t * 2 * np.pi)

# concatenate notes
audio = np.hstack((A_note, Csh_note, E_note))
# normalize to 16-bit range
audio *= 32767 / np.max(np.abs(audio))
# convert to 16-bit data
audio = audio.astype(np.int16)

data = sinewave.astype(np.float32).tobytes()
def call_back(in_data, fm_cnt, time_info, status_flag):
    print('fm_cnt: ', fm_cnt)
    return(data, pyaudio.paContinue)
    

p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paFloat32,
                         channels=1,
                         rate=sample_rate,
                         output=True,
                         output_device_index=2,
                         frames_per_buffer=16*2048,
                         stream_callback=call_back
                         )
# Assuming you have a numpy array called samples
#data = sinewave.astype(np.float32).tobytes()
#stream.write(data)
stream.start_stream()

time.sleep(2)

stream.stop_stream()

stream.close()
p.terminate()


# start playback
#play_obj = sa.play_buffer(audio, 1, 2, sample_rate)

# wait for playback to finish before exiting
#play_obj.wait_done()



import wave

filename = '/home/pi/Downloads/beep-07a.wav'

chunk = 1024
wf = wave.open(filename, 'rb')

p = pyaudio.PyAudio()

info = p.get_device_count()

stream = p.open(format = p.get_format_from_width(wf.getsampwidth()),
           channels=wf.getnchannels(),
           rate=wf.getframerate(),
           output = True)

data = wf.readframes(chunk)

while data != b'':
    stream.write(data)
    data = wf.readframes(chunk)

stream.close()
p.terminate()

"""import time

import traceback

try:

    pysine.sine(444, duration=1)
    time.sleep(2)
except Exception:
    traceback.print_exc()"""

