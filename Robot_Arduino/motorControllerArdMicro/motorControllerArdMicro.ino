//********** INCLUDES *************
#include <TimerOne.h>
#include <U8g2lib.h>
#include <AccelStepper.h>

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
double rot_stepDeg = 15.1; //steps per degree

float bat_volt = 0.0;

int distance_report = 0;


#define batReadTime 1000
#define posReadTime 50

#define COMM_TIMEOUT 100
//COLORS
//ENABLE PIN: RED   /  TURQUISE / GREY
//DIR PIN   : ORANGE / BLUE
//STEP PIN  : YELLOW / PURPLE

//ENABLE : HIGH DISABLE
//DIR    : LOW CLOCKWISE
#define stepMargin 200L
#define stepMarginRot 200

#define SPD_linear 3000

#define SPD_rotate 800

#define TORQUE_OFF_OBSTACLE 1000



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
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR_1_STEP, MOTOR_1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR_2_STEP, MOTOR_2_DIR);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR_3_STEP, MOTOR_3_DIR);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR_4_STEP, MOTOR_4_DIR);

unsigned long last_volt_time = 0;
unsigned long last_pos_time = 0;
unsigned long torque_off_time = 0;
int needTorqueOff = 0;
int isInMotion = 0;
int report_dist_type = 0;
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, A5, A4, U8X8_PIN_NONE); //OLED setup

void setup() {
  portSetup();
  setMotorTorqueAll(0);

  Serial.begin(115200);
  delay(300);
  while(!Serial.available() > 0);
  Serial.write("m");
  delay(100);

  analogReference(EXTERNAL);
  u8g2.begin();

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(1200);
  
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(1200);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(1200);
  
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(1200);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);

}
void loop() {

  if(millis()-last_volt_time > (unsigned long)batReadTime && isInMotion == 0){
    last_volt_time = millis();
    readAndPrintVoltage();
  }
  if(millis()-last_pos_time > (unsigned long)posReadTime){
    last_pos_time = millis();
    sendPositionInfo();
  }

  if(isInMotion == 0){
  String command = checkForSerialAngleDist();
  if(command != "-1"){
    stepper1.setAcceleration(1200);
    stepper2.setAcceleration(1200);
    stepper3.setAcceleration(1200);
    stepper4.setAcceleration(1200);
    setMotorTorqueAll(1);
    needTorqueOff = 0;
    if(command[0] == 'd'){
      command.remove(0,1);
      int angle = command.toInt();
      double a_toPass = abs(angle);
      if(angle != -1){
        if(angle>0){ 
          rotateRobot(a_toPass, clockwise, SPD_rotate);
        }else{
          rotateRobot(a_toPass, counterCW, SPD_rotate);
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
        }
    }
  }else if(Serial.available() > 0){
    char d = Serial.read();
    if(d == 'x'){
      stepper1.setAcceleration(10000);
      stepper2.setAcceleration(10000);
      stepper3.setAcceleration(10000);
      stepper4.setAcceleration(10000);
      stepper1.stop();
      stepper2.stop();
      stepper3.stop();
      stepper4.stop();
      Serial.write('s');
      Serial.write('s');
      isInMotion = 0;
      torque_off_time = millis() + TORQUE_OFF_OBSTACLE;
      needTorqueOff = 1;
    }
  }
  //delay(2000);
  

if(isInMotion == 1 and checkAllSteppersStates() == 1){
  isInMotion = 0;
  sendDone();
  setMotorTorqueAll(0);
}else if((torque_off_time > millis() && isInMotion == 0 && needTorqueOff == 1)){
  setMotorTorqueAll(0);
  needTorqueOff = 0;
}

stepper1.run();
stepper2.run();
stepper3.run();
stepper4.run();

}

bool checkAllSteppersStates(){
  if(!stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning() && !stepper4.isRunning())
  {
    return 1;
  }
  return 0;
}

void readAndPrintVoltage(void){
  bat_volt = (analogRead(A3)/1023.0)*(30.5)*(1.0); //last value is adjustment factor for ref voltage
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,10, "Total Voltage:");
  u8g2.drawStr(0,25, (String(bat_volt, 3)+"V").c_str());
  u8g2.sendBuffer();
}

void sendPositionInfo(void){  //Periodic information logging 
  Serial.write('p');
  Serial.write((lowByte(int(bat_volt*100))));
  Serial.write((highByte(int(bat_volt*100))));
  if(report_dist_type == 1){
    distance_report = int(abs(stepper1.currentPosition())*movLin_mmStep)/10;
  }else if(report_dist_type == 2){
    distance_report = int(abs(stepper2.currentPosition())*movLin_mmStep)/10;
  }else{
    distance_report = 0;
  }
  Serial.write((lowByte(int(distance_report))));
  Serial.write((highByte(int(distance_report))));
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


void rotateRobot(double deg, bool cw, int spd){
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  isInMotion = 1;
  report_dist_type = 0;
  int steps = int(rot_stepDeg * deg);
  int dir = -1;
  if(cw == 0){
    dir = 1;
  }
  stepper1.setMaxSpeed(spd);
  stepper2.setMaxSpeed(spd);
  stepper3.setMaxSpeed(spd);
  stepper4.setMaxSpeed(spd);
  stepper1.move(dir*steps);
  stepper2.move(dir*steps);
  stepper3.move(dir*steps);
  stepper4.move(dir*steps);

}
// **moveRobot Notes**
//dirColor : RBGW, Red Black Green White arm direciton.
// R = 1, B = 2 ....
void moveRobot(double distanceMM, int dirColor, int spd){ 
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper1.setMaxSpeed(spd);
  stepper2.setMaxSpeed(spd);
  stepper3.setMaxSpeed(spd);
  stepper4.setMaxSpeed(spd);
  isInMotion = 1;
  distance_report = 0;
  long int steps = long(movLin_stepMM * double(distanceMM));
  int dir1 = 1;
  int dir2 = 1;
  int dir3 = 1;
  int dir4 = 1;
  if(dirColor == 1 || dirColor == 2){ //Red or Black
      dir1 = -1;
      dir2 = 1;
      dir3 = 1;
      dir4 = -1;
    }else{
      dir1 = 1;
      dir2 = -1;
      dir3 = -1;
      dir4 = 1;
      }

    if(dirColor == 1 || dirColor == 3){
      stepper2.move(dir2*steps);
      stepper4.move(dir4*steps);
      report_dist_type = 2;
     }else{
      stepper1.move(dir1*steps);
      stepper3.move(dir3*steps);
      report_dist_type = 1;
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
  report_dist_type = 0;
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
