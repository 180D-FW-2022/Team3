//********** INCLUDES *************
#include <TimerOne.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
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

//622.3mm wheel to wheel --> 1955mm circumference
//120mm wheel diamter --> 377mm circumference
//1036 steps per revolution
//0.3638996139mm per step.
//5372.37 steps per rotation 360 degrees
//~14.92 steps per degree


double movLin_mmStep = 0.3638996139; //mm per step
double movLin_stepMM = 2.7480106101; //steps per mm
double rot_stepDeg = 16; //steps per degree

float bat_volt = 0.0;

int distance = 0;


#define batReadTime 1000
#define COMM_TIMEOUT 100
//COLORS
//ENABLE PIN: RED   /  TURQUISE / GREY
//DIR PIN   : ORANGE / BLUE
//STEP PIN  : YELLOW / PURPLE

//ENABLE : HIGH DISABLE
//DIR    : LOW CLOCKWISE
#define stepMargin 200L
#define stepMarginRot 200

#define SPD_1 2000
#define SPD_2 1500
#define SPD_3 1100
#define SPD_linear 500

#define SPD_rot_1 1600
#define SPD_rot_2 1000
#define SPD_rotate 600



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

unsigned long last_volt_time = 0;

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, A5, A4, U8X8_PIN_NONE); //OLED setup

void setup() {
  //setMotorTorqueAll(0);
  analogReference(EXTERNAL);
  u8g2.begin();
  portSetup();
  Timer1.initialize(500000); //every 500ms run interrupt
  Timer1.attachInterrupt(interruptHandler);
  Serial.begin(115200);
  delay(3000);

}

void loop() {
  if(millis()-last_volt_time > (unsigned long)batReadTime){
    last_volt_time = millis();
    readAndPrintVoltage();
  }

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
  moveRobot(toTurn, 1, SPD_linear);
  }else{
    toTurn = -1*toTurn;
    moveRobot(toTurn, 3, SPD_linear);
    }
  }

#endif
  
}

void readAndPrintVoltage(void){
  bat_volt = (analogRead(A3)/1023.0)*(30.5)*(1.0); //last value is adjustment factor for ref voltage
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,10, "Total Voltage:");
  u8g2.drawStr(0,25, (String(bat_volt, 3)+"V").c_str());
  u8g2.sendBuffer();
}

void interruptHandler(void){  //Periodic information logging 
  Serial.write('p');
  Serial.write((byte) (int(bat_volt*100)>>8));
  Serial.write((byte) (int(bat_volt*100)));
  Serial.write((byte) ((distance)>8));
  Serial.write((byte) (distance));
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
  if(steps<stepMarginRot){
  for(int i = 0; i < steps; i++){
  stepMotor(1, spd);
  stepMotor(2, spd);
  stepMotor(3, spd);
  stepMotor(4, spd);
  }
  }else{

    for(int i = 0; i < 50; i++){
  stepMotor(1, SPD_rot_1);
  stepMotor(2, SPD_rot_1);
  stepMotor(3, SPD_rot_1);
  stepMotor(4, SPD_rot_1);
  }
  for(int i = 0; i < 50; i++){
  stepMotor(1, SPD_rot_2);
  stepMotor(2, SPD_rot_2);
  stepMotor(3, SPD_rot_2);
  stepMotor(4, SPD_rot_2);
  }
  for(int i = 0; i < (steps-200); i++){
  stepMotor(1, spd);
  stepMotor(2, spd);
  stepMotor(3, spd);
  stepMotor(4, spd);
  }
  for(int i = 0; i < 50; i++){
  stepMotor(1, SPD_rot_2);
  stepMotor(2, SPD_rot_2);
  stepMotor(3, SPD_rot_2);
  stepMotor(4, SPD_rot_2);
  }
  for(int i = 0; i < 50; i++){
  stepMotor(1, SPD_rot_1);
  stepMotor(2, SPD_rot_1);
  stepMotor(3, SPD_rot_1);
  stepMotor(4, SPD_rot_1);
  }
    
  }
}

// **moveRobot Notes**
//dirColor : RBGW, Red Black Green White arm direciton.
// R = 1, B = 2 ....
void moveRobot(double distanceMM, int dirColor, int spd){ 
  distance = 0;
  long int steps = long(movLin_stepMM * double(distanceMM));
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
      if(steps<stepMargin){
      for(int i = 0; i < steps; i++){
        stepMotor(2, SPD_1);
        stepMotor(4, SPD_1);
        }
      }else{
        for(int i = 0; i < 50; i++){
        stepMotor(2, SPD_1);
        stepMotor(4, SPD_1);
        }
        for(int i = 0; i < 30; i++){
        stepMotor(2, SPD_2);
        stepMotor(4, SPD_2);
        }
        for(int i = 0; i < 20; i++){
        stepMotor(2, SPD_3);
        stepMotor(4, SPD_3);
        }
        bool isHalt = 0;
        for(long int j = 0L; j < (steps-200L); j++){
          if(Serial.available() > 0){
          char d = Serial.read();
          if(d == 'x'){
            isHalt = 1;
            Serial.write('s');
            break;
            break;
          }
        }
          if(isHalt == 0){
            distance = (((steps-100L)-j)*movLin_mmStep)/10;
         stepMotor(2, spd);
         stepMotor(4, spd);
          }else{
            break;
          }
        
        }
        if(!isHalt){
        for(int i = 0; i < 20; i++){
        stepMotor(2, SPD_3);
        stepMotor(4, SPD_3);
        }
        for(int i = 0; i < 30; i++){
        stepMotor(2, SPD_2);
        stepMotor(4, SPD_2);
        }
        for(int i = 0; i < 50; i++){
        stepMotor(2, SPD_1);
        stepMotor(4, SPD_1);
        }
        }
      } 
     }else{
       if(steps<stepMargin){
      for(int i = 0; i < steps; i++){
        stepMotor(1, SPD_1);
        stepMotor(3, SPD_1);
        }
      }else{
        for(int i = 0; i < 50; i++){
        stepMotor(1, SPD_1);
        stepMotor(3, SPD_1);
        }
        for(int i = 0; i < 30; i++){
        stepMotor(1, SPD_2);
        stepMotor(3, SPD_2);
        }
        for(int i = 0; i < 20; i++){
        stepMotor(1, SPD_3);
        stepMotor(3, SPD_3);
        }
        bool isHalt = 0;
        for(long j = 0L; j < steps-(200L); j++){
          if(Serial.available() > 0){
            char d = Serial.read();
            if(d == 'x'){
              isHalt = 1;
              Serial.write('s');
              break;
              break;
            }
          }
          if(isHalt == 0){
          distance = (((steps-100L)-j)*movLin_mmStep)/10;
        stepMotor(1, spd);
        stepMotor(3, spd);
          }else{
            break;
            }
        }
        if(!isHalt){
        for(int i = 0; i < 20; i++){
        stepMotor(1, SPD_3);
        stepMotor(3, SPD_3);
        }
        for(int i = 0; i < 30; i++){
        stepMotor(1, SPD_2);
        stepMotor(3, SPD_2);
        }
        for(int i = 0; i < 50; i++){
        stepMotor(1, SPD_1);
        stepMotor(3, SPD_1);
        }
      }
      }
      }
}



//******TESTING AND IN DEVELOPMENT

String checkForSerialAngleDist(){
  int timeout = 0;
    if (Serial.available()) {
    char c = Serial.read();
    byte d = 0;
    byte e = 0;
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
     int r = ( d<<8 ) | e;
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
