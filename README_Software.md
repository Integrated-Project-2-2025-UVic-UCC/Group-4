# Group-4 software


## Table of Contents
- [Python](#python)
  - [Python installation](#Python-installation)
  - [Python full code explanation](#Python-full-code-explanation)
  - [Python usage](#Python-usage)
- [Arduino](#Arduino)
  - [Arduino installation](#Arduino-installation)
  - [Arduino full code explanation](#Arduino-full-code-explanation)
  - [Arduino usage](#Arduino-usage)

## Python
### Python installation

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
These libraries are used for several important functions. Sounddesign is used for recording your microphone. Vosk is used for recognizing the commands. Numpy is used for changing arrays efficiently. Pyserial is used for sending the command to the Arduino. You can download all the non-standard neccessary libraries with this line of code in you command prompt, don't forget to run "command prompt" as administrator.

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

### Python full code explaination

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

For a more accurate voice recognition a noise filter function has to be implemented. How it works is it listens for 3 seconds when the program is started and then it will remember the noice so it can remove it later.
```python
def record_noise_profile(seconds=3):
    print("Recording noice profile...(be silent)")
    noise = sd.rec(int(seconds * SAMPLERATE), samplerate=SAMPLERATE, channels=1, dtype='int16')
    sd.wait()
    noise = noise.flatten()
    noise_fft = np.fft.fft(noise, n=BLOCKSIZE)
    return noise_fft
```
This function removes the noice from the actual commands.
```python
def estimate_noise(audio_fft, noise_fft):
    noise_magnitude = np.abs(noise_fft)
    audio_magnitude = np.abs(audio_fft)
    clean_magnitude = np.maximum(audio_magnitude - noise_magnitude, 0)
    phase = np.angle(audio_fft)
    return clean_magnitude * np.exp(1j * phase)
```

THe callback gets a chunk of data from the microphone and the amount of frames and how long it took. Then it calls the previous function estimate_noise() to filter out the noice and then it returns the filtered data.
```python
def callback(indata, frames, time, status):
    if status:
        print(status)
    chunk = np.frombuffer(indata, dtype=np.int16)
    chunk_fft = np.fft.fft(chunk, n=BLOCKSIZE)
    filtered_fft = estimate_noise(chunk_fft, noise_fft_profile)
    filtered_chunk = np.fft.ifft(filtered_fft).real.astype(np.int16)
    audio_queue.put(filtered_chunk.tobytes())
```

This function is called when it recognizes a command and in this function the command is being send to the bluetooth module connected to the arduino.
```python
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
```
After all the functions are defined. First the noice function is called to get the noice profile. Then the Vosk model is made and the command list is made. and the variables are initalised.
```python
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
```
Then it starts the audiostream where it listens for 1 second. then it puts the audio in the queue. And gets a result from it and puts it in an array. If it is a command it will store that command otherwise  it will give an unknown in the array. Then the unkowns and the unnecessary spaces are removed. Then it will reverse it so the last heard command is in the front.
```python
# Start audiostream
with sd.RawInputStream(samplerate=16000, blocksize=16000, dtype='int16',
                       channels=1, callback=callback):
    print("Listening to commands... (Say 'Stop' to stop)")
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
```
Then it needs to be checked if there is a new word in the array by checking the length of the array then it will move on. otherwise it will return to listening.
```python
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
```
Then if there is a new command in the array it will check which command it is. And call the function where it sends that command to the bluetooth module.
```python
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
```

### Python usage

Start up the arduino. Then run the python file. Then follow the Serial instructions on the terminal of the laptop. If file doesn't run that means there is a bluetooth issue. Check if the arduino is powered and if the bluetooth module is blinking rapidly. That means it is avalable for connecting. If it is blinking slowly that means it is connected. If it doesn't blink at all then it isn't connected properly. If it is blinking rapidly and the python file has an error. Change the COM, COM5 and COM6 available.
```python
btSerial = serial.Serial('COM5', baudrate=9600)#change to COM5 or COM6 if not working
```

## Arduino

### Arduino installation

For the arduino it has two libraries. In the code below there are also some comments on how the libraries are configurated.
```cpp
#include <SoftwareSerial.h>

// Include the AccelStepper Library//https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
#include <AccelStepper.h>

// Define step constant
#define MotorInterfaceType 4

SoftwareSerial BTSerial(2,10); // RX | TX // or (10,11)
String  receivedText = "";


/* -----------------------------------------------------|
 * |-----------------|------------|------------|
 * | function        | Channel A  | Channel B  |
 * | Direction       | Digital 12 | Digital 13 |             
 * | Speed (PWM)     | Digital 3  | Digital 11 |
 * | Brake           | Digital 9  | Digital 8  |
 * | Current Sensing | Analog 0 ` | Analog 1   | not necessary for basic use
 * |-----------------|------------|------------|
 * https://www.instructables.com/Arduino-Motor-Shield-Tutorial/

  *---------------------------------------------------*

```
In the Arduino IDE there has to be only library installed, which is the AccelStepper. Go to tools>manage libraries...>Search on AccelStepper>install AccelStepper by Mike McCauley.
After that you can upload it to the arduino.
Then the use the interface 4 so it uses 4 wire configuration for the fork motor.
Then the bluetooth connection module is made with the two connections for RX and TX.
Then for the four weels the pins that are used in the motor board are shown in comments.


### Arduino full code explaination
```cpp
int pinMotorLeft = 13;
int pinBrakeLeft = 8;
int pinSpeedLeft = 11;


int pinMotorRight = 12;
int pinBrakeRight = 9;
int pinSpeedRight = 3;

int leftMotorArray[3] = {pinMotorLeft, pinBrakeLeft, pinSpeedLeft};
int rightMotorArray[3] = {pinMotorRight, pinBrakeRight, pinSpeedRight};
int speedMotor = 200;

// Creates an instance
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper myStepper(MotorInterfaceType, 7, 5, 6, 4);
int directionFork = 1;
float UpDown = false;
```
Here are all of the motors are defined.

In the code below there are two functions shown one with going forward and one with standing still. These functions show how the weels are powered.
The first one in the array sets the direction, the second one is for the brake and third and final one is for the speed. There are other functions like left and right but those ones are the same except for a few parameters.
```cpp
void forward() {

  digitalWrite(leftMotorArray[0], HIGH); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], speedMotor);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], HIGH);  //Establishes forward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], speedMotor);    //Spins the motor on Channel B at half speed
}

void still() {
  digitalWrite(leftMotorArray[0], LOW); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], HIGH);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], 0);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], LOW);  //Establishes backward direction of Channel B
  digitalWrite(rightMotorArray[1], HIGH);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], 0);    //Spins the motor on Channel B at half speed
}
```

In the setup the serial monitor and the bluetooth baud rate are set. Then the motor pins are set as an output. Lastly the fork speed and acceleration is defined.
```cpp
void setup() {
  Serial.begin(9600); //for serial monitor
  BTSerial.begin(9600); //hc-05 default baud rate

  
  for(int i = 0; i < 3; i++) {
    pinMode(leftMotorArray[i], OUTPUT);
    pinMode(rightMotorArray[i], OUTPUT);
  }
  myStepper.setMaxSpeed(400.0);       // Max safe speed
  myStepper.setAcceleration(200.0);   // Reasonable acceleration
  Serial.println(myStepper.currentPosition());

  Serial.println("setup done");
}
```

In the loop there is checked if there is a command sent. If so then there the characters are added letter for letter and when the entire command is put in the variable then it will print the final command.
```cpp
void loop() {

  if(BTSerial.available()){
    Serial.println("Start reading");
    char character = BTSerial.read();
  
    if(character == '\n' || character == '\r'){
      receivedText.trim();
      Serial.print("Final string: >");
      Serial.print(receivedText);
      Serial.println("<");
```

Then it will compare the command to the commands in the if statemants and when it found its match then it will turn the motors accordingly for 1 second and then stop again.
```cpp
if (receivedText=="right"){
        right();
        Serial.println("right");
        delay(1000);
        still();
      }
      if (receivedText=="forward"){
        forward();
        Serial.println("forward");
        delay(1000);
        still();
      }

```
For the fork it is different. It will first check if it is up or down. When it is the first time for the fork command it will assume it is in the down position. Then it will see "UpDown" as false. So then it will stay in while loop until it is at the Up position. In the while loop it will check if it is still and if it is then it will set the next place at 2048. If it has reached that position then it will exit the loop. If it is not yet at that place then it will keep running the fork. For going down it is the exact opposite.
```cpp
if (receivedText=="fork"){
    Serial.print("Fork");
    if (!UpDown){
      while(!UpDown){
        if (myStepper.distanceToGo() == 0) {
          myStepper.moveTo(2048); 
          if (myStepper.currentPosition() == 2048) {
          UpDown = true;
          }
        }
        Serial.println(myStepper.currentPosition());
        myStepper.run(); 
      }
    }
    else if (UpDown){
      while(UpDown){
        if (myStepper.distanceToGo() == 0) {
          myStepper.moveTo(0); 
          if (myStepper.currentPosition() == 0) {
          UpDown = false;
          }
        }
        Serial.println(myStepper.currentPosition());
        myStepper.run(); 
      }
    }
  }
```
And after that it will reset the command and go back to the top of the loop.
### Arduino usage
To use this code to your preference, what you can do is change the speed or change the delays to move longer or shorter. 
