import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write

def estimate_noise(audio_fft, noise_fft):
    """
    Schat de ruis en trekt deze af van het originele signaal in het frequentiedomein.
    :param audio_fft: FFT van de originele audio
    :param noise_fft: FFT van het ruisprofiel
    :return: Gefilterde FFT van de audio
    """
    noise_magnitude = np.abs(noise_fft)
    audio_magnitude = np.abs(audio_fft)
    
    # Trek ruisamplitude af (zorgt ervoor dat het niet negatief wordt)
    clean_magnitude = np.maximum(audio_magnitude - noise_magnitude, 0)
    
    # Behoud fase van originele signaal
    phase = np.angle(audio_fft)
    return clean_magnitude * np.exp(1j * phase)

# Instellingen
DURATION_NOISE = 5  # Ruisprofiel (seconden)
DURATION_SPEECH = 5  # Spraakopname (seconden)
SAMPLERATE = 16000  # Sample rate in Hz

# Stap 1: Neem ruisprofiel op
print("Neem omgevingsgeluid op...")
noise_audio = sd.rec(int(DURATION_NOISE * SAMPLERATE), samplerate=SAMPLERATE, channels=1, dtype='int16')
sd.wait()
noise_audio = noise_audio.flatten()

# Stap 2: Neem spraak op
print("Neem spraak op...")
speech_audio = sd.rec(int(DURATION_SPEECH * SAMPLERATE), samplerate=SAMPLERATE, channels=1, dtype='int16')
sd.wait()
speech_audio = speech_audio.flatten()
print("done recording")

# Stap 3: Bereken FFT
noise_fft = np.fft.fft(noise_audio)
speech_fft = np.fft.fft(speech_audio)

# Stap 4: Trek ruis af
filtered_fft = estimate_noise(speech_fft, noise_fft)

# Stap 5: Inverse FFT terug naar audio
filtered_audio = np.fft.ifft(filtered_fft).real
filtered_audio = np.int16(filtered_audio)  # Zet terug naar integer formaat

# Speel de gefilterde audio af
print("Afspelen met ruis...")
sd.play(speech_audio, SAMPLERATE)
sd.wait()

print("Afspelen zonder ruis...")
sd.play(filtered_audio, SAMPLERATE)
sd.wait()

# Optioneel: Opslaan als WAV-bestand
write("speech_audio.wav", SAMPLERATE, speech_audio)
write("filtered_audio.wav", SAMPLERATE, filtered_audio)


