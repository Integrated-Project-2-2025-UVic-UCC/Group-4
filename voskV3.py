import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import numpy as np
import serial
import time

#change 'COM5' to your HC-05 port(e.g., '/dev/rfcomm0' linux maxos)
#device manager -> ports(com&LPT) -> look for the port
btSerial = serial.Serial('COM5', baudrate=9600)#change to COM5 or COM6 if not working
time.sleep(5)
# Pad naar het gedownloade model (pas dit aan naar jouw locatie)
MODEL_PATH = "languages\\vosk-model-small-en-us-0.15"

# Laad het Vosk-model
model = Model(MODEL_PATH)

# Queue voor audiostream
audio_queue = queue.Queue()
noise_fft_profile = None

SAMPLERATE = 16000
BLOCKSIZE = 16000  # Smaller for real-time chunks
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


def sendMessageToArduino(command, isRepeated, amountOfMessages, newWord):
    
    if(newWord):
        amount = 1#amountOfMessagesSend
        btSerial.write((command+ "\n").encode())
        print("sent: ", command)
    elif(amountOfMessages==0):
        
        amount = 1
    elif(amountOfMessages>=1):
        amount = 0
    return amount
    

# === Start Script ===
noise_fft_profile = record_noise_profile()

recognizer = KaldiRecognizer(model, 16000)
recognizer.SetWords(False)  # Voorkomt volledige transcripties
recognizer.SetGrammar('["left", "right", "start", "stop", "forward", "fork", "reverse", "[unk]"]')
lastWord = ""


amountOfMessagesSend = 0
amountOfTimesUsed = 0
IsRepeated = False
NewWord = False
AmountToAdd = 0
PreviousLength = 0

# Start audiostream
with sd.RawInputStream(samplerate=16000, blocksize=16000, dtype='int16',
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
            words.reverse()
            amountOfTimesUsed = 0 
            if words:
                text = words[0]
                length = len(words)
                if PreviousLength < length:
                    NewWord = True
                    AmountToAdd = -1
                elif PreviousLength == length:
                    NewWord = False
                PreviousLength = length
            else:
                text = ""
            print(f"Herkenning: {words}")#only commands shown
            print("Only last word:", text)
        if(NewWord):
            # Controleer op specifieke commando's
            if "start" in text:
                print("Commando: START gedetecteerd!")
                AmountToAdd = sendMessageToArduino("start", IsRepeated, amountOfMessagesSend, NewWord)

                #function()moveRobot
            if "stop"  in text:
                print("Commando: STOP gedetecteerd! Programma beëindigen...")
                AmountToAdd = sendMessageToArduino("stop", IsRepeated, amountOfMessagesSend, NewWord)
                print("Commando: STOP gedetecteerd! Programma beëindigen...")
                #function()moveRobot
            if "left"  in text:
                print("Commando: LINKS gedetecteerd!")
                AmountToAdd = sendMessageToArduino("left", IsRepeated, amountOfMessagesSend, NewWord)
                print("Commando: LINKS gedetecteerd!")
                #function()moveRobot
            if "right"  in text:
                print("Commando: RECHTS gedetecteerd!")
                AmountToAdd = sendMessageToArduino("right", IsRepeated, amountOfMessagesSend, NewWord)
                print("Commando: RECHTS gedetecteerd!")
                #function()moveRobot
            if "forward"  in text:
                print("Commando: FORWARD gedetecteerd!")
                AmountToAdd = sendMessageToArduino("forward", IsRepeated, amountOfMessagesSend, NewWord)
                print("Commando: FORWARD gedetecteerd!")
            if "reverse"  in text:
                print("Commando: reverse gedetecteerd!")
                AmountToAdd = sendMessageToArduino("reverse", IsRepeated, amountOfMessagesSend, NewWord)
                print("Commando: reverse gedetecteerd!")
            if "fork"  in text:
                print("Commando: fork gedetecteerd!")
                AmountToAdd = sendMessageToArduino("fork", IsRepeated, amountOfMessagesSend, NewWord)
                print("Commando: fork gedetecteerd!")
                #function()moveRobot
            if AmountToAdd == 1:
                amountOfMessagesSend += AmountToAdd
            elif AmountToAdd == -1:
                amountOfMessagesSend = 0
                AmountToAdd == 0
                #function()moveRobot
            #while not audio_queue.empty():
            #    audio_queue.get()
        