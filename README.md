# Group-4
Software

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)

## 🧰 Installation

This project relies on several Python libraries:

```python
import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import numpy as np
import serial
import time
```
These libraries are used for several important functions. Sounddesign is used for recording your microphone. Vosk is used for recognizing the commands. Numpy is used for changing arrays efficiently. Pyserial is used for sending the command to the Arduino. You can download all the non-standard neccessary libraries with this line of code in you command prompt, don't forget to run command prompt as administrator.

```cmd
pip install sounddevice vosk scipy numpy pyserial
```

To get the bluetooth connection there is COM5 and COM6 available. sometimes it might not connect and you have to change to the other COM.
Then it is needed to download a language model from their site [(Alphacephei, n.d.)](https://alphacephei.com/vosk/models). Then put this folder somewhere close to the python file and edit the MODEL_PATH to the corrosponding path.
```python
btSerial = serial.Serial('COM5', baudrate=9600)#change to COM5 or COM6 if not working
time.sleep(5)
# Pad naar het gedownloade model (pas dit aan naar jouw locatie)
MODEL_PATH = "languages\\vosk-model-small-en-us-0.15"
```

Then the model path is put in to the model. For the audio a queue is started. A noise profile is made. and the sample and blocksize are made.
```python
# Laad het Vosk-model
model = Model(MODEL_PATH)

# Queue voor audiostream
audio_queue = queue.Queue()
noise_fft_profile = None

SAMPLERATE = 16000
BLOCKSIZE = 16000  # Smaller for real-time chunks
text = ""
```
