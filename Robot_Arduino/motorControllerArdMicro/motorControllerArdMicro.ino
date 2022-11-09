//********** INCLUDES *************
#include <TimerOne.h>
//END

//********* PIN DEFINITIONS ************

#define SET 1
#define RESET 0


//COLORS
//ENABLE PIN: RED   /  TURQUISE / GREY
//DIR PIN   : ORANGE / BLUE
//STEP PIN  : YELLOW / PURPLE

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

//******* GLOBAL VARIABLES **********
int motorStepsArr[4] = {0,0,0,0}; //number of steps left for each motor.
bool motorEnableArr[4] = {0,0,0,0}; //motor enable memory.

void setup() {
  portSetup();
  setMotorTorqueAll(1);
  Timer1.initialize(10000); //every 10ms run interrupt
  Timer1.attachInterrupt(interruptHandler);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void interruptHandler(void){ 
  
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
      digitalWrite(MOTOR_1_ENABLE, SET);
      }else{
      digitalWrite(MOTOR_1_ENABLE, RESET);
      }
  }else if(motorID == 2){
      if(setEnabled){
      digitalWrite(MOTOR_2_ENABLE, SET);
      }else{
      digitalWrite(MOTOR_2_ENABLE, RESET);
      }
  }else if(motorID == 3){
      if(setEnabled){
      digitalWrite(MOTOR_3_ENABLE, SET);
      }else{
      digitalWrite(MOTOR_3_ENABLE, RESET);
      }
  }else if(motorID == 4){
     if(setEnabled){
     digitalWrite(MOTOR_4_ENABLE, SET);
     }else{
     digitalWrite(MOTOR_4_ENABLE, RESET);
     }
  }
  }

void setMotorTorqueAll(bool setEnabled){
  setMotorTorque(1, setEnabled);
  setMotorTorque(2, setEnabled);
  setMotorTorque(3, setEnabled);
  setMotorTorque(4, setEnabled);
  }

void stepMotor(int motorID){
  if(motorID == 1){
    digitalWrite(MOTOR_1_STEP, SET);
    delayMicroseconds(100);
    digitalWrite(MOTOR_1_STEP, RESET);
  }else if(motorID == 2){
    digitalWrite(MOTOR_2_STEP, SET);
    delayMicroseconds(100);
    digitalWrite(MOTOR_2_STEP, RESET);
  }else if(motorID == 3){ 
    digitalWrite(MOTOR_3_STEP, SET);
    delayMicroseconds(100);
    digitalWrite(MOTOR_3_STEP, RESET);
  }else if(motorID == 4){
    digitalWrite(MOTOR_4_STEP, SET);
    delayMicroseconds(100);
    digitalWrite(MOTOR_4_STEP, RESET);
  }
  }
