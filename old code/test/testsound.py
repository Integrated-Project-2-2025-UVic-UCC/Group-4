import sounddevice as sd
import numpy as np
import wave

RATE = 16000
CHANNELS = 1
RECORD_SECONDS = 5
OUTPUT_FILE = "test_sounddevice.wav"

print("Opnemen met Sounddevice...")
recording = sd.rec(int(RECORD_SECONDS * RATE), samplerate=RATE, channels=CHANNELS, dtype=np.int16)
sd.wait()

# Opslaan als .wav
with wave.open(OUTPUT_FILE, "wb") as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(2)  # 16-bit audio
    wf.setframerate(RATE)
    wf.writeframes(recording.tobytes())

print(f"Opname opgeslagen als {OUTPUT_FILE}")
