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
btSerial = serial.Serial('COM5', baudrate=9600)#change to COM5 or COM6 if not working
time.sleep(2)
# Pad naar het gedownloade model (pas dit aan naar jouw locatie)
MODEL_PATH = "languages\\vosk-model-small-en-us-0.15"

# Laad het Vosk-model
model = Model(MODEL_PATH)

# Queue voor audiostream
audio_queue = queue.Queue()
noise_fft_profile = None

SAMPLERATE = 16000
BLOCKSIZE = 32000  # Smaller for real-time chunks
text = ""


def estimate_noise(audio_fft, noise_fft):
    noise_magnitude = np.abs(noise_fft)
    audio_magnitude = np.abs(audio_fft)
    clean_magnitude = np.maximum(audio_magnitude - noise_magnitude, 0)
    phase = np.angle(audio_fft)
    return clean_magnitude * np.exp(1j * phase)

def record_noise_profile(seconds=3):
    print("Opname van ruisprofiel...")
    noise = sd.rec(int(seconds * SAMPLERATE), samplerate=SAMPLERATE, channels=1, dtype='int16')
    sd.wait()
    noise = noise.flatten()
    noise_fft = np.fft.fft(noise, n=BLOCKSIZE)
    return noise_fft

def callback(indata, frames, time, status):
    if status:
        print(status)
    chunk = np.frombuffer(indata, dtype=np.int16)
    chunk_fft = np.fft.fft(chunk, n=BLOCKSIZE)
    filtered_fft = estimate_noise(chunk_fft, noise_fft_profile)
    filtered_chunk = np.fft.ifft(filtered_fft).real.astype(np.int16)
    audio_queue.put(filtered_chunk.tobytes())


def sendMessageToArduino(command):
    btSerial.write((command+ "\n").encode())
    print("sent: ", command)
    

# === Start Script ===
noise_fft_profile = record_noise_profile()

recognizer = KaldiRecognizer(model, 16000)
recognizer.SetWords(False)  # Voorkomt volledige transcripties
recognizer.SetGrammar('["left", "right", "start", "stop", "forward", "fork", "reverse", "[unk]"]')
lastWord = ""
# Start audiostream
with sd.RawInputStream(samplerate=16000, blocksize=32000, dtype='int16',
                       channels=1, callback=callback):
    print("Luisteren naar commando's... (Zeg 'stop' om te stoppen)")
    while True:
        data = audio_queue.get()

        recognizer.AcceptWaveform(data)
        result = json.loads(recognizer.PartialResult())
        text = result.get("partial", "")#of text = result.get("text", "").strip()
        if text:
            #lastWord = text.split()[-1]
            print(f"Herkenning: {text}")#full text
            text = text.replace("[unk]", "")#unk removed
            text = ' '.join(text.split())#spaces removed
            print(f"Herkenning: {text}")#only commands shown
            words = text.split()
            

            if words:
             text = words[-1]
            #check if previous command was the same one if this one has already been send. 
            #if the command has already been used one then skip it unless the previous command was the same.
            #with an itteration that does +=1, check for the amount of times the command has been said and then check if that there is a new command
            # i+=1 if new command or reset to 1;
            #then check i amount of words from the back;
            #if the same then do another of that command;
            #if not the same then it is still at the same command;
            else:
                text = ""

            print("Only last word:", text)
        print("new text:", text)
        # Controleer op specifieke commando's
        if "start" in text:
            print("Commando: START gedetecteerd!")
            sendMessageToArduino("start")

            #function()moveRobot
        if "stop"  in text:
            print("Commando: STOP gedetecteerd! Programma beëindigen...")
            sendMessageToArduino("stop")
            print("Commando: STOP gedetecteerd! Programma beëindigen...")
            #function()moveRobot
            
        if "left"  in text:
            print("Commando: LINKS gedetecteerd!")
            sendMessageToArduino("left")
            print("Commando: LINKS gedetecteerd!")
            #function()moveRobot
        if "right"  in text:
            print("Commando: RECHTS gedetecteerd!")
            sendMessageToArduino("right")
            print("Commando: RECHTS gedetecteerd!")
            #function()moveRobot
        if "forward"  in text:
            print("Commando: FORWARD gedetecteerd!")
            sendMessageToArduino("forward")
            print("Commando: FORWARD gedetecteerd!")
        if "reverse"  in text:
            print("Commando: reverse gedetecteerd!")
            sendMessageToArduino("reverse")
            print("Commando: reverse gedetecteerd!")
            #function()moveRobot

            #function()moveRobot
        #while not audio_queue.empty():
        #    audio_queue.get()