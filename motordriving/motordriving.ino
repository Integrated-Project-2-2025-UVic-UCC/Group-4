#include <SoftwareSerial.h>

// Include the AccelStepper Library//https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/
#include <AccelStepper.h>

// Define step constant
#define MotorInterfaceType 4

SoftwareSerial BTSerial(2,3); // RX | TX // or (10,11)
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

// Creates an instance
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper myStepper(MotorInterfaceType, 7, 5, 6, 4);

 */


//-------------------------------------------------------------------------------------------------------------------------------------------
/*
Motor=voor- achteruit pin
brake=rem pin
Speed=snelheid pin
*/
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

//Ultrasonic sensor
#define echoPin 37 // attach pin D12 Arduino to pin Echo of HC-SR04
#define trigPin 36 //attach pin D13 Arduino to pin Trig of HC-SR04


void reverse() {
  digitalWrite(leftMotorArray[0], LOW); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], speedMotor);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], HIGH);  //Establishes forward direction of Channel B
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
  
  digitalWrite(leftMotorArray[0], LOW); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], 150);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], HIGH);  //Establishes backward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], 130);    //Spins the motor on Channel B at half speed
}

void right() {
  digitalWrite(leftMotorArray[0], HIGH); //Establishes forward direction of Channel A
  digitalWrite(leftMotorArray[1], LOW);   //Disengage the Brake for Channel A
  analogWrite(leftMotorArray[2], 120);//speedMotorTurn+40);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(rightMotorArray[0], LOW);  //Establishes backward direction of Channel B
  digitalWrite(rightMotorArray[1], LOW);   //Disengage the Brake for Channel B
  analogWrite(rightMotorArray[2], 190);//speedMotorTurn+50);    //Spins the motor on Channel B at half speed
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
double Sonic_Sensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH); 
  double distance_cm = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.println(distance_cm);
  return distance_cm;
}
void setup() {
  Serial.begin(9600); //for serial monitor
  BTSerial.begin(9600); //hc-05 default baud rate
  pinMode(10, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT 
  
  for(int i = 0; i < 3; i++) {
    pinMode(leftMotorArray[i], OUTPUT);
    pinMode(rightMotorArray[i], OUTPUT);
  }
  
  forward();
  delay(1000);
  still();

  myStepper.setMaxSpeed(1000.0);
	myStepper.setAcceleration(50.0);
	myStepper.setSpeed(200);
	myStepper.moveTo(2038);

  Serial.println("vfev");
}
  
void loop() {

  if(BTSerial.available()){
    
    char character = BTSerial.read();
  
    
    if(character == '\n'){
      receivedText.trim();
      Serial.print("Final string: >");
      Serial.print(receivedText);
      Serial.println("<");
      if (receivedText=="stop"){
        still();
      if (receivedText=="start"){
        
      }
      if (receivedText=="left"){
        left();
        digitalWrite(10,HIGH);
        
      }
      if (receivedText=="right"){
        right();
        digitalWrite(10,HIGH);
      }
      if (receivedText=="forward"){
        forward();
        digitalWrite(10,HIGH);
      }
      if (receivedText=="backwards"){
        reverse();
        digitalWrite(10,HIGH);
      }
      if (receivedText=="look"){
        digitalWrite(10,HIGH);
        
      }
      
        
      }
      if (receivedText=="fork"){
        myStepper.move(4324*directionFork);//in steps
        directionFork *= -1;
      
      	
      }
      receivedText = "";
    }
    else{
      receivedText += character;
      
    }
    
  }
 

}