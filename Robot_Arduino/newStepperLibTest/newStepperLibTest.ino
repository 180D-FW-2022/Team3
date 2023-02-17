// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.
 
#include <AccelStepper.h>
#include <MultiStepper.h>
 
#define SET 1
#define RESET 0

//MOTOR 1: RED ARM
#define MOTOR_1_ENABLE 2
#define MOTOR_1_DIR 3
#define MOTOR_1_STEP 4

#define MOTOR_2_ENABLE 5
#define MOTOR_2_DIR 6
#define MOTOR_2_STEP 7

#define MOTOR_3_ENABLE 8 
#define MOTOR_3_DIR 9
#define MOTOR_3_STEP 10

#define MOTOR_4_ENABLE 11
#define MOTOR_4_DIR 12
#define MOTOR_4_STEP 13
//END

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR_1_STEP, MOTOR_1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR_2_STEP, MOTOR_2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR_3_STEP, MOTOR_3_DIR);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR_4_STEP, MOTOR_4_DIR);
 
// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;
 
void setup() {
  Serial.begin(9600);
  portSetup();
  setMotorTorqueAll(1);
  // Configure each stepper
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(1200);
  
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(1200);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(1200);
  
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(1200);
 
  stepper3.move(10000); 
  stepper2.move(10000); 
  stepper1.move(10000); 
  stepper4.move(10000); 

  // Then give them to MultiStepper to manage
  // steppers.addStepper(stepper1);
  // steppers.addStepper(stepper2);
  // steppers.addStepper(stepper3);
  // steppers.addStepper(stepper4);
}
 


void portSetup(){ //SETUP INPUT/OUTPUT pins.
  
  pinMode(MOTOR_1_ENABLE, OUTPUT);
  pinMode(MOTOR_2_ENABLE, OUTPUT);
  pinMode(MOTOR_3_ENABLE, OUTPUT);
  pinMode(MOTOR_4_ENABLE, OUTPUT);

  pinMode(MOTOR_1_DIR, OUTPUT);
  pinMode(MOTOR_2_DIR, OUTPUT);
  pinMode(MOTOR_3_DIR, OUTPUT);
  pinMode(MOTOR_4_DIR, OUTPUT);

  pinMode(MOTOR_1_STEP, OUTPUT);
  pinMode(MOTOR_2_STEP, OUTPUT);
  pinMode(MOTOR_3_STEP, OUTPUT);
  pinMode(MOTOR_4_STEP, OUTPUT);
  
  }

void setMotorTorque(int motorID, bool setEnabled){
  if(motorID == 1){
      if(setEnabled){
      digitalWrite(MOTOR_1_ENABLE, RESET);
      }else{
      digitalWrite(MOTOR_1_ENABLE, SET);
      }
  }else if(motorID == 2){
      if(setEnabled){
      digitalWrite(MOTOR_2_ENABLE, RESET);
      }else{
      digitalWrite(MOTOR_2_ENABLE, SET);
      }
  }else if(motorID == 3){
      if(setEnabled){
      digitalWrite(MOTOR_3_ENABLE, RESET);
      }else{
      digitalWrite(MOTOR_3_ENABLE, SET);
      }
  }else if(motorID == 4){
     if(setEnabled){
     digitalWrite(MOTOR_4_ENABLE, RESET);
     }else{
     digitalWrite(MOTOR_4_ENABLE, SET);
     }
  }
  }

void setMotorTorqueAll(bool setEnabled){
  setMotorTorque(1, setEnabled);
  setMotorTorque(2, setEnabled);
  setMotorTorque(3, setEnabled);
  setMotorTorque(4, setEnabled);
  }
int count = 0;
void loop() {
 if(count == 10000){
  stepper1.stop();
  stepper2.stop();
  stepper3.stop();
  stepper4.stop();
    }
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  count++;


}