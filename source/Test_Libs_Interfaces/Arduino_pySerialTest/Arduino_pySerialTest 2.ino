
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT); //built in LED
}

void loop() {
 int angle = waitForSerialAngle_Blocking();
}

int waitForSerialAngleDist(){
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
  }
