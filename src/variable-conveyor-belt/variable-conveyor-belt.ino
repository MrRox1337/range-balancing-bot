#include <AccelStepper.h>

const int dirPin = 5;   
const int stepPin = 6;  
const int potPin = A3;  
const int minSpeed = 200;
const int maxSpeed = 1000; 

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setSpeed(800);  // Initial speed (this will be overwritten)
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potPin);  // Read potentiometer value

  // // Map potentiometer value to desired speed range
  long stepSpeed = map(potValue, 0, 1023, minSpeed, maxSpeed);

  // Dynamically set the speed of the stepper motor based on potentiometer value
  stepper.setSpeed(stepSpeed);

  // Run the stepper motor at the set speed
  stepper.runSpeed();
}
