int ph1 = 7;                  // Control Board 
int en1 = 9;
int ph2 = 10;
int en2 = 11;
int sw = 13;
int input1 = A4;
int input2 = A3;
#include <SPI.h>

int mode2Speed = 200;                               
int motor1A = MOSI;             // Encoder Inputs
int motor1B = SCK;
int motor2A = 12;
int motor2B = 36;
volatile int currentPos1 = 1; // Encoder Variables
volatile int currentPos2 = 1100;
int encoderTotalCount = 2959; // 1000:1 motor: 986 * 3
//int encoderTotalCount = 1137;   // 380:1 motor
//int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,3};    // Leveling (1000:1 motor)
//int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,3,3,3,3,3};    // Leveling (380:1 motor)
//int sequenceArray[] = {5,5,5,5,5,5,5,5,5,6,1,1};            // Create mound
int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,1,1};     
bool onoff = HIGH;
volatile int flag1 = 0;
volatile int flag2 = 0;
volatile int targetEncoderCount1 = 0;
volatile int targetEncoderCount2 = 0;

/********** Setup ********/
void setup() {
    Serial.begin(9600);
    
    pinMode(ph1, OUTPUT); //Motor 1
    pinMode(ph2, OUTPUT); //Motor 2
    pinMode(sw, INPUT);
    pinMode (motor1A,INPUT);
    pinMode (motor1B,INPUT);
    pinMode (motor2A,INPUT);
    pinMode (motor2B,INPUT);
    //attachInterrupt(digitalPinToInterrupt(motor1A), motor1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(motor2A), motor2_ISR, RISING);
    
    delay(3000);
}
/********** Motor 1 ISR ********/
void motor1_ISR(){
   if (digitalRead(motor1B) == digitalRead(motor1A)) currentPos1--;
   else currentPos1++;
   currentPos1 = (currentPos1 + encoderTotalCount) % encoderTotalCount;
   if (currentPos1 == targetEncoderCount1){
      flag1 = 1;
   }
}
/********** Motor 2 ISR ********/
void motor2_ISR(){
   if (digitalRead(motor2B) == digitalRead(motor2A)) currentPos2--;
   else currentPos2++;
   currentPos2 = (currentPos2 + encoderTotalCount) % encoderTotalCount;
   if (currentPos2 == targetEncoderCount2){
      flag2 = 1;
   }
}
/********** Loop ********/
void loop() {
  delay(10);
  onoff = digitalRead(sw);
  if (onoff) {
    int motor1 = analogRead(input1);
    if (motor1 < 512) {
      motor1 = map(motor1, 512, 0, 0, 255);
      analogWrite(en1, motor1);
      digitalWrite(ph1, LOW);
    }
    else {
      motor1 = map(motor1, 512, 1024, 0, 255);
      analogWrite(en1, motor1);
      digitalWrite(ph1, HIGH);
    }
    int motor2 = analogRead(input2);
    if (motor2 < 512) {
      motor2 = map(motor2, 512, 0, 0, 255);
      analogWrite(en2, motor2);
      digitalWrite(ph2, LOW);
    }
    else {
      motor2 = map(motor2, 512, 1024, 0, 255);
      analogWrite(en2, motor2);
      digitalWrite(ph2, HIGH);
    }
    Serial.println(currentPos1);
  }
  else{
    Serial.println(sizeof(sequenceArray));
    for (int i=0;i<sizeof(sequenceArray);i++){
      if (sequenceArray[i] == 1){
        forward1();
      }
      else if (sequenceArray[i] == 2){
        backward1();
      }
      else if (sequenceArray[i] == 3){
        inward1();
      }
      else if (sequenceArray[i] == 4){
        outward1();
      }
      else if (sequenceArray[i] == 5){
        front_forward1();
      }
      else if (sequenceArray[i] == 6){
        back_forward1();
      }
      delay(1);
    }
  }
}
void forward1() {
  Serial.println("Forward");
  analogWrite(en2, mode2Speed);
  digitalWrite(ph2, LOW);
  delay(1);
  analogWrite(en1, mode2Speed);
  digitalWrite(ph1, LOW);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(1);
    if (flag2) {
      analogWrite(en2, 0);
      delay(1);
      analogWrite(en1, 0);
    }
//    if (flag1) {
//      analogWrite(en1, 0);
//    }
  }
  flag2 = 0;
  flag1 = 0;
 // delay(100);
}

void backward1() {
  Serial.println("Backward");
  analogWrite(en2, mode2Speed);
  digitalWrite(ph2, HIGH);
  delay(1);
  analogWrite(en1, mode2Speed);
  digitalWrite(ph1, HIGH);
  targetEncoderCount2 = currentPos2+1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(1);
    if (flag2) {
      analogWrite(en2, 0);
      delay(1);
      analogWrite(en1, 0);
    }
  }
  flag2 = 0;
  flag1 = 0;
 // delay(100);
}

void inward1() {
  Serial.println("Inward");
  analogWrite(en2, mode2Speed);
  digitalWrite(ph2, HIGH);
  delay(1);
  analogWrite(en1, mode2Speed);
  digitalWrite(ph1, LOW);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(1);
    if (flag2) {
      analogWrite(en2, 0);
      delay(1);
      analogWrite(en1, 0);
    }
  }
  flag2 = 0;
  flag1 = 0;
 // delay(100);
}

void outward1() {
  Serial.println("Outward");
  analogWrite(en2, mode2Speed);
  digitalWrite(ph2, LOW);
  delay(1);
  analogWrite(en1, mode2Speed);
  digitalWrite(ph1, HIGH);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(1);
    if (flag2) {
      analogWrite(en2, 0);
      delay(1);
      analogWrite(en1, 0);
    }
  }
  flag2 = 0;
  flag1 = 0;
 // delay(100);
}

void front_forward1() {
  Serial.println("Front forward");
  analogWrite(en2, mode2Speed);
  digitalWrite(ph2, HIGH);
  delay(1);
  analogWrite(en1, 0);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(1);
    if (flag2) {
      analogWrite(en2, 0);
      delay(1);
      analogWrite(en1, 0);
    }
  }
  flag2 = 0;
  flag1 = 0;
 // delay(100);
}


void back_forward1() {
  Serial.println("Front forward");
  analogWrite(en1, mode2Speed);
  digitalWrite(ph1, LOW);
  delay(1);
  analogWrite(en2, 0);
  delay(7000);
}
