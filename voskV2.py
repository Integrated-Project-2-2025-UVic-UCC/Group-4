import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
#import vosk
from scipy.signal import butter, lfilter
import numpy as np
import serial
import time

#change 'COM5' to your HC-05 port(e.g., '/dev/rfcomm0' linux maxos)
#device manager -> ports(com&LPT) -> look for the port
btSerial = serial.Serial('COM5')
time.sleep(2)
# Pad naar het gedownloade model (pas dit aan naar jouw locatie)
MODEL_PATH = "languages\\vosk-model-small-en-us-0.15"

# Laad het Vosk-model
model = Model(MODEL_PATH)

# Queue voor audiostream
audio_queue = queue.Queue()

def highpassFilter(data, cutoff=100, fs=16000, order=5):
    nyquist=0.5*fs
    normalCutoff = cutoff / nyquist
    b, a = butter(order, normalCutoff, btype='high', analog=False)
    return lfilter(b, a, data)


def callback(indata, frames, time, status):
    """ Callback-functie die audio toevoegt aan de queue. """
    if status:
        print(status, flush=True)
    audioData = np.frombuffer(indata, dtype=np.int16)
    filterdData = highpassFilter(indata, dtype=np.int16)

def sendMessageToArduino(command):
    btSerial.write(command.encode())
    print("sent: ", command)

# Start audiostream
with sd.RawInputStream(samplerate=16000, blocksize=32000, dtype='int16',
                       channels=1, callback=callback):
    print("Luisteren naar commando's... (Zeg 'stop' om te stoppen)")
    
    while True:
        data = audio_queue.get()

        recognizer = KaldiRecognizer(model, 16000)
        recognizer.SetWords(False)  # Voorkomt volledige transcripties
        recognizer.SetGrammar('["left", "right", "start", "stop", "forward", "look", "[unk]"]')

        recognizer.AcceptWaveform(data)
        result = json.loads(recognizer.PartialResult())
        text = result.get("partial", "")
        if text:
            print(f"Herkenning: {text}")

        # Controleer op specifieke commando's
        if "start" in text:
            print("Commando: START gedetecteerd!")
            sendMessageToArduino("start")

            #function()moveRobot
        if "stop" in text:
            print("Commando: STOP gedetecteerd! Programma beëindigen...")
            sendMessageToArduino("stop")
            #function()moveRobot
            break
        if "left" in text:
            print("Commando: LINKS gedetecteerd!")
            sendMessageToArduino("left")
            #function()moveRobot
        if "right" in text:
            print("Commando: RECHTS gedetecteerd!")
            sendMessageToArduino("right")
            #function()moveRobot
        if "forward" in text:
            print("Commando: FORWARD gedetecteerd!")
            sendMessageToArduino("forward")
            #function()moveRobot
        if "look" in text:
            print("Commando: LOOK gedetecteerd!")
            sendMessageToArduino("look")
            #function()moveRobot
        #while not audio_queue.empty():
        #    audio_queue.get()