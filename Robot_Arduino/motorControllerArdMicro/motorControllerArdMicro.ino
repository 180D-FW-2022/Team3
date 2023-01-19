//********** INCLUDES *************
#include <TimerOne.h>
//END

//#define DEBUG_SER
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


#define COMM_TIMEOUT 100
//COLORS
//ENABLE PIN: RED   /  TURQUISE / GREY
//DIR PIN   : ORANGE / BLUE
//STEP PIN  : YELLOW / PURPLE

//ENABLE : HIGH DISABLE
//DIR    : LOW CLOCKWISE

#define SPD_linear 800
#define SPD_rotate 400



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
  Timer1.initialize(10000); //every 10ms run interrupt
  Timer1.attachInterrupt(interruptHandler);
  Serial.begin(115200);
  delay(3000);

}

void loop() {
  String command = checkForSerialAngleDist();
  if(command != "-1"){
    setMotorTorqueAll(1);
    if(command[0] == 'd'){
      command.remove(0,1);
      int angle = command.toInt();
      double a_toPass = abs(angle);
      if(angle != -1){
        if(angle>0){ 
          rotateRobot(a_toPass, clockwise, SPD_rotate);
          delay(200);
          sendDone();
        }else{
          rotateRobot(a_toPass, counterCW, SPD_rotate);
          delay(200);
          sendDone();
        }
        }
      }else if(command[0] == 'r' || command[0] == 'b' || command[0] == 'g' || command[0] == 'w'){
        int armDir = 1;
        if(command[0] == 'r'){
          armDir = 1;
          }else if(command[0] == 'b'){
          armDir = 2;
          }else if(command[0] == 'g'){
          armDir = 3;
          }else if(command[0] == 'w'){
          armDir = 4;
          }
        command.remove(0,1);
        int distance = command.toInt();
        double d_toPass = abs(distance)*10.0;
        moveRobot(d_toPass, armDir, SPD_linear);
        delay(50);
        sendDone();
        }
        setMotorTorqueAll(0);
    }
    delay(100);
    command = "";
  //delay(2000);

#ifdef DEBUG_SER

if(Serial.available()>0){
  String incomingString = Serial.readString();
  int toTurn = incomingString.toInt();
  Serial.println(toTurn);
  if(toTurn > 0){
  moveRobot(toTurn, 1, SPD);
  }else{
    toTurn = -1*toTurn;
    moveRobot(toTurn, 3, SPD);
    }
  }

#endif
  
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
    //delayMicroseconds(spdDel);
  }else if(motorID == 2){
    digitalWrite(MOTOR_2_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_2_STEP, RESET);
    //delayMicroseconds(spdDel);
  }else if(motorID == 3){ 
    digitalWrite(MOTOR_3_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_3_STEP, RESET);
    //delayMicroseconds(spdDel);
  }else if(motorID == 4){
    digitalWrite(MOTOR_4_STEP, SET);
    delayMicroseconds(spdDel);
    digitalWrite(MOTOR_4_STEP, RESET);
   // delayMicroseconds(spdDel);
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

String checkForSerialAngleDist(){
  int timeout = 0;
    if (Serial.available()) {
    char c = Serial.read();
    char d = 0;
     char e = 0;
    if(c == 'd'){ //degreeMove
      if(Serial.available() > 0){
    d = Serial.read();
      }else{
        timeout = 0;
        while(!Serial.available() && timeout <= COMM_TIMEOUT){
          delay(1);
          timeout++;
          }
        if(timeout >= COMM_TIMEOUT){
          return "-1";
          }
      d = Serial.read();
      }
      if(Serial.available() > 0){
    e = Serial.read();
      }else{
         timeout = 0;
        while(!Serial.available() && timeout <= COMM_TIMEOUT){
          delay(1);
          timeout++;
          }
        if(timeout >= COMM_TIMEOUT){
          return "-1";
          }
      e = Serial.read();
      }
        while (Serial.available() > 0) { //empty buffer
        Serial.read();  
        delay(1);
        } 
     int r = (d << 8) | (e);
     Serial.write(d);
     Serial.write(e);
     return ("d"+String(r));
      }else if(c == 'r' || c == 'b' || c == 'g' || c == 'w' ){ //linearMove
      if(Serial.available() > 0){
    d = Serial.read();
      }else{
          timeout = 0;
        while(!Serial.available() && timeout <= COMM_TIMEOUT){
          delay(1);
          timeout++;
          }
        if(timeout >= COMM_TIMEOUT){
          return "-1";
          }
      d = Serial.read();
      }
      if(Serial.available() > 0){
    e = Serial.read();
      }else{
         timeout = 0;
        while(!Serial.available() && timeout <= COMM_TIMEOUT){
          delay(1);
          timeout++;
          }
        if(timeout >= COMM_TIMEOUT){
          return "-1";
          }
      e = Serial.read();
      }
      while (Serial.available() > 0) { //empty buffer
     Serial.read();  
     delay(1);
    } 
     int r = (d << 8) | e;
     Serial.write(d);
     Serial.write(e);
     return (String(c)+String(r));
      }
      }else{ 
    while (Serial.available() > 0) { //empty buffer
     Serial.read();  
     delay(1);
    } 
  }
  return "-1";
  }

void sendDone(){
  Serial.write('a');
  delay(10);
  Serial.write('a');
  }

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
      }else if(c != 'm'){ 
    while (Serial.available() > 0) { //empty buffer
     Serial.read();  
    } 
  }
  }
  return -1;
  }

int waitForSerialDist_Blocking(){
    if (Serial.available()) {
    char c = Serial.read();
    char d = 0;
     char e = 0;
    if(c == 'm'){ //continue
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
      }else if(c != 'd'){ 
    while (Serial.available() > 0) { //empty buffer
     Serial.read();  
    } 
  }
  }
  return -1;
  }
