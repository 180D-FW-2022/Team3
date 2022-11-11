//********** INCLUDES *************
#include <TimerOne.h>
//END

//********* PIN DEFINITIONS ************

#define SET 1
#define RESET 0

//584mm wheel-to-wheel --> 1834.69mm circumference
//120mm wheel diamter --> 377mm circumference
//1036 steps per revoltion
//0.3638996139mm per step.
//5041.748 steps per rotation 360 degrees
//~ 14 steps per degree

double movLin_mmStep = 0.3638996139; //mm per step
double movLin_stepMM = 2.7480106101; //steps per mm
double rot_stepDeg = 14.005; //steps per degree

//COLORS
//ENABLE PIN: RED   /  TURQUISE / GREY
//DIR PIN   : ORANGE / BLUE
//STEP PIN  : YELLOW / PURPLE

//ENABLE : HIGH DISABLE
//DIR    : LOW CLOCKWISE


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

#define clockwise  true
#define counterCW  false
//******* GLOBAL VARIABLES **********
int motorStepsArr[4] = {0,0,0,0}; //number of steps left for each motor.

void setup() {
  portSetup();
  setMotorTorqueAll(1);
  Timer1.initialize(10000); //every 10ms run interrupt
  Timer1.attachInterrupt(interruptHandler);
  Serial.begin(9600);
  delay(3000);

}

void loop() {
  int angle = waitForSerialAngle_Blocking();
  double a_toPass = abs(angle);
  if(angle != -1){
    if(angle>0){
    rotateRobot(a_toPass, clockwise, 500);
    }else{
      rotateRobot(a_toPass, counterCW, 500);
      }
    }
  delay(2000);
  
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

void stepMotor(int motorID, int spdDel){
  if(motorID == 1){
    digitalWrite(MOTOR_1_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_1_STEP, RESET);
  }else if(motorID == 2){
    digitalWrite(MOTOR_2_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_2_STEP, RESET);
  }else if(motorID == 3){ 
    digitalWrite(MOTOR_3_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_3_STEP, RESET);
  }else if(motorID == 4){
    digitalWrite(MOTOR_4_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_4_STEP, RESET);
  }
  }

void setMotorDir(int motorID, bool cw){
if(motorID == 1){
    if(cw){
    digitalWrite(MOTOR_1_DIR, RESET);
    }else{
    digitalWrite(MOTOR_1_DIR, SET);
    }
}else if(motorID == 2){
    if(cw){
    digitalWrite(MOTOR_2_DIR, RESET);
    }else{
    digitalWrite(MOTOR_2_DIR, SET);
    }
}else if(motorID == 3){
    if(cw){
    digitalWrite(MOTOR_3_DIR, RESET);
    }else{
    digitalWrite(MOTOR_3_DIR, SET);
    }
}else if(motorID == 4){
   if(cw){
   digitalWrite(MOTOR_4_DIR, RESET);
   }else{
   digitalWrite(MOTOR_4_DIR, SET);
   }
}
}

void rotateRobot(double deg, bool cw, int spd){
  int steps = int(rot_stepDeg * deg);
  
  setMotorDir(1, cw);
  setMotorDir(2, cw);
  setMotorDir(3, cw);
  setMotorDir(4, cw);
  
  for(int i = 0; i < steps; i++){
  stepMotor(1, spd);
  stepMotor(2, spd);
  stepMotor(3, spd);
  stepMotor(4, spd);
  }
  }


// **moveRobot Notes**
//dirColor : RBGW, Red Black Green White arm direciton.
// R = 1, B = 2 ....
void moveRobot(double distanceMM, int dirColor, int spd){ 
  int steps = int(movLin_stepMM * distanceMM);
  
  if(dirColor == 1 || dirColor == 2){ //Red or Black
    setMotorDir(1, clockwise); //for black dir
    setMotorDir(2, counterCW); //for red dir
    setMotorDir(3, counterCW); //for black dor
    setMotorDir(4, clockwise); //for red dir
    }else{
    setMotorDir(1, counterCW);
    setMotorDir(2, clockwise);
    setMotorDir(3, clockwise);
    setMotorDir(4, counterCW);
      }

    if(dirColor == 1 || dirColor == 3){
      for(int i = 0; i < steps; i++){
        stepMotor(2, spd);
        stepMotor(4, spd);
        }
     }else{
      for(int i = 0; i < steps; i++){
        stepMotor(1, spd);
        stepMotor(3, spd);
        }
      }
}



//******TESTING AND IN DEVELOPMENT

int waitForSerialAngle_Blocking(){
    if (Serial.available()) {
    char c = Serial.read();
    char d = 0;
     char e = 0;
    if(c == 'd'){ //continue
      if(Serial.available() > 0){
    d = Serial.read();
      }else{
        while(!Serial.available()){}
      d = Serial.read();
      }
      if(Serial.available() > 0){
    e = Serial.read();
      }else{
        while(!Serial.available()){}
      e = Serial.read();
      }

     int r = (d << 8) | (e);
     Serial.write(d);
     Serial.write(e);
     return r;
      }else{ 
    while (Serial.available() > 0) { //empty buffer
     Serial.read();  
    } 
  }
  }
  return -1;
  }
