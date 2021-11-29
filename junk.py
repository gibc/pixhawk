import os
import subprocess
import sys, tty


import pyaudio


import numpy as np
from numpy import arange

import simpleaudio as sa
import decimal

#Play a fixed frequency sound.
#from __future__ import division
import math
import pyaudio
import matplotlib.pyplot as plt

strattime = 0
endtime = 5
sampelrate = 48000
time = np.arange(strattime, endtime, 1/sampelrate)
theta = 0
freq = 440
amp = .25
sinewave = amp * np.sin(2 * np.pi * freq * time + theta)
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
sinewave = amp * np.sin(2 * np.pi * 1000 * time)


Csh_note = np.sin(Csh_freq * t * 2 * np.pi)
E_note = np.sin(E_freq * t * 2 * np.pi)

# concatenate notes
audio = np.hstack((A_note, Csh_note, E_note))
# normalize to 16-bit range
audio *= 32767 / np.max(np.abs(audio))
# convert to 16-bit data
audio = audio.astype(np.int16)

p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paFloat32,
                         channels=1,
                         rate=sample_rate,
                         output=True,
                         output_device_index=2
                         )
# Assuming you have a numpy array called samples
data = sinewave.astype(np.float32).tobytes()
stream.write(data)
stream.close()
p.terminate()

# start playback
#play_obj = sa.play_buffer(audio, 1, 2, sample_rate)

# wait for playback to finish before exiting
#play_obj.wait_done()



import wave

filename = '/home/pi/piano2.wav'

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

