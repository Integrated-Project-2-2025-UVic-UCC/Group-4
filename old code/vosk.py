import json
import queue
import wave
import sys
from vosk import Model, KaldiRecognizer

modelPath = "talen\\vosk-model-small-en-us-0.15"
audioFile = "opname.wav"

model = Model(modelPath)
recognizer = KaldiRecognizer(model, 16000)

audioPath = sys.argv[1]
with wave.open(audioPath,"rb") as wf:
    if wf.getnchannels() != 1 or wf.getsampwidth() !=2 or wf.getframerate() != 16000:
        print("fout: bestand moet mono, 16-bit PCM, 16kHz zijn ")
        sys.exit(1)

    print("verwerken van ", audioPath)

    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            print("herkenning ", result.get("text", ""))
            
    finalResult = json.loads(recognizer.FinalResult())
    print("\nVOlledige herkenning: ", finalResult.get("text", ""))






#recognizer.SetWords(True)
#
#audioQueue = queue.Queue()
#
#def callback(indata, frames, time, status):
#    if status:
#        print(status, flush=True)
#    audioQueue.put(bytes(indata))
#
#with sd.RawInputStream(samplerate=16000)