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
 */
//-------------------------------------------------------------------------------------------------------------------------------------------
/*
Motor=forward- backwards pin
brake=brake pin
Speed=Speed pin
*/
int pinMotorLeft = 13;
int pinBrakeLeft = 8;
int pinSpeedLeft = 11;


int pinMotorRight = 12;
int pinBrakeRight = 9;
int pinSpeedRight = 3;

int leftMotorArray[3] = {pinMotorLeft, pinBrakeLeft, pinSpeedLeft};
int rightMotorArray[3] = {pinMotorRight, pinBrakeRight, pinSpeedRight};
int speedMotor = 120;

// Creates an instance
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper myStepper(MotorInterfaceType, 7, 5, 6, 4);
int directionFork = 1;
float UpDown = false;



void reverse() {
  digitalWrite(leftMotorArray[0], LOW); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], speedMotor);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], LOW);  //Establishes forward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], speedMotor);    //Spins the motor on Channel B at half speed
}
  
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

void right() {
  
  digitalWrite(leftMotorArray[0], HIGH); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], speedMotor+20);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], LOW);  //Establishes backward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], speedMotor+20);    //Spins the motor on Channel B at half speed
}

void left() {
  digitalWrite(leftMotorArray[0], LOW); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], speedMotor+20);//speedMotorTurn+40);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], HIGH);  //Establishes backward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], speedMotor+20);//speedMotorTurn+50);    //Spins the motor on Channel B at half speed
}
void fastleft() {
  digitalWrite(leftMotorArray[0], LOW); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], 200);//speedMotorTurn+40);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], LOW);  //Establishes backward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], 200);//speedMotorTurn+50);    //Spins the motor on Channel B at half speed
}

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
  
void loop() {

  if(BTSerial.available()){
    Serial.println("Start reading");
    char character = BTSerial.read();
  
    
    if(character == '\n' || character == '\r'){
      receivedText.trim();
      Serial.print("Final string: >");
      Serial.print(receivedText);
      Serial.println("<");
      if (receivedText=="stop"){
        still();
      }
      if (receivedText=="start"){
        
      }
      if (receivedText=="left"){
        left();
        Serial.println("left");
        delay(1000);
        still();
        
      }
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
      if (receivedText=="reverse"){
        reverse();
        Serial.println("reverse"); 
        delay(1000);
        still();
      }
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
      receivedText = "";
      Serial.println("reset");
    }
    else{
      receivedText += character;
      
    }
    
  }
  else{
    Serial.println("no serial available");
  }
 

}