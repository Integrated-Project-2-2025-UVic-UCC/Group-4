#include <AccelStepper.h>

#define MotorInterfaceType 4  // 4-pin interface

// Create stepper instance: IN1-IN3-IN2-IN4
AccelStepper myStepper(MotorInterfaceType, 7, 5, 6, 4);

void setup() {
  Serial.begin(9600);

  // Set safe values for this stepper motor
  myStepper.setMaxSpeed(400.0);       // Max safe speed
  myStepper.setAcceleration(200.0);   // Reasonable acceleration

  // Start at position 0
  myStepper.setCurrentPosition(0);    

  // Move to positive position
  myStepper.moveTo(2048);             // One full revolution
}

void loop() {
  // Print current position for debugging
  Serial.println(myStepper.currentPosition());

  // If the motor reached its destination...
  if (myStepper.distanceToGo() == 0) {
    // If we're at 0, go to 2048
    if (myStepper.currentPosition() == 0) {
      myStepper.moveTo(2048);
    }
    // If we're at 2048, go back to 0
    else {
      myStepper.moveTo(0);
    }
  }

  // Keep stepping
  myStepper.run();
}
