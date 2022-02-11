#include <Wire.h>
#include <SPI.h>
#include "TimerInterrupt_Generic.h"

// Accelerometer adresses and variables
#define IIS328DQ_REG_CTRL_REG1      0x20  // Set mode, ODR and enabled axes
#define IIS328DQ_REG_CTRL_REG3      0x22  // Interrupt control register
#define IIS328DQ_REG_CTRL_REG4      0x23  // Set scale
#define IIS328DQ_REG_OUT_X_L        0x28  // X-axis acceleration data in 2's complement. LSB
#define IIS328DQ_REG_INT1_CFG       0X30  // Configuration interrupt 1
#define IIS328DQ_REG_INT1_THS       0X32  // Threshold interrupt 1
#define IIS328DQ_REG_INT1_DURATION  0X33
#define IIS328DQ_PM_NORMAL          0x20
#define IIS328DQ_ODR_100_CF_74      0x08
#define IIS328DQ_X_ENABLE           0x01  // Enable X-axis
#define IIS328DQ_Y_ENABLE           0x02  // Enable Y-axis
#define IIS328DQ_Z_ENABLE           0x04
#define IIS328DQ_FS_2               0x00  // +/- 2g
#define IIS328DQ_FS_4               0x10  // +/- 4g
#define IIS328DQ_FS_8               0x30  // +/- 8g
#define IIS328DQ_BDU_ENABLE         0X80
#define IIS328DQ_SLAVE_ADDR         0X19
#define MULTIPLE_BYTES_MASK         0X80

// Timer variables
#define TIMER0_INTERVAL_MS        100
#define TIMER0_DURATION_MS        200
volatile uint32_t preMillisTimer0 = 0;
SAMDTimer ITimer0(TIMER_TC3);
float tempTimer = 0;

// Accelerometer helper variables
byte X0,X1,Y0,Y1,Z0,Z1;
byte axis[6];
volatile double X_prec,Y_prec,Z_prec;
short X,Y,Z;
volatile float pitch=0;
volatile float roll=0;
float old_pitch_a=0;
float old_roll_a=0;
float alpha = 0.2;

// Target angle variables
volatile int targetAngleCycles = 0;
int targetAngleCyclesGoal = 5;
volatile int targetAngleReached = 0;
int targetAngle = 20;
int targetAngleTolerance = 2;

// Control Board pins 
int ph1 = 7;                  
int en1 = 9;
int ph2 = 10;
int en2 = 11;
int input1 = A4;
int input2 = A3;
int sw = 13;

// Control Board variables
bool onoff = HIGH;
int mode2Speed = 200;  

// Encoder pins            
int motor1A = MOSI;             
int motor1B = SCK;
int motor2A = 12;
int motor2B = 36;

// Encoder variables
volatile int currentPos1 = 1; 
volatile int currentPos2 = 1100;
int encoderTotalCount = 1137; //986 * 3;
volatile int flag1 = 0;
volatile int flag2 = 0;
volatile int targetEncoderCount1 = 0;
volatile int targetEncoderCount2 = 0;

// Sequences
//int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,3};    // Leveling
//int sequenceArray[] = {5,5,5,5,5,5,5,5,5,5,6,7,7};            // Create mound
//int sequenceArray[] = {5,5,5,5,5,5,5,5,7,7,7,7,1,1,1,1};            // Create mound
//int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,3,3};    // Leveling (380:1 motor)
//int sequenceArray[] = {3,3,3,3,1,1};
//int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,3,3};    // Leveling (380:1 motor)
int sequenceArray[] = {1,1,3,1,1,3,1,1,3,1,1,3,1,1,3};    

/********** Setup ********/
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(100);

    // Accelerometer interrupt setup
    if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0)){
      preMillisTimer0 = millis();
      Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(preMillisTimer0);
    }
    else
      Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

    // Accelerometer configuration registers
    uint8_t ctrl_reg1 = (IIS328DQ_ODR_100_CF_74 | IIS328DQ_PM_NORMAL | IIS328DQ_Z_ENABLE | IIS328DQ_Y_ENABLE | IIS328DQ_X_ENABLE);
    writeToAccel(IIS328DQ_REG_CTRL_REG1,ctrl_reg1);
    uint8_t ctrl_reg4 = IIS328DQ_FS_2 | IIS328DQ_BDU_ENABLE;      // Scale to +/- 2g and BDU
    writeToAccel(IIS328DQ_REG_CTRL_REG4,ctrl_reg4);
    writeToAccel(IIS328DQ_REG_CTRL_REG3,0x00);                    // Interrupt control
    writeToAccel(IIS328DQ_REG_INT1_THS,0x1A);                     // Scale threshold
    writeToAccel(IIS328DQ_REG_INT1_DURATION,0x64);                // Interrupt duration
    writeToAccel(IIS328DQ_REG_INT1_CFG,0x04);                     // ZLIE, YLIE

    // Inputs/outputs setup
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

    // Default motor states
    analogWrite(en2, mode2Speed);
    digitalWrite(ph2, LOW);
    delay(5);
    analogWrite(en1, mode2Speed);
    digitalWrite(ph1, HIGH);
}

/********** Accelerometer Interrupt ********/
void TimerHandler0(){
  // I2C Setup
  Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
  Wire.write(IIS328DQ_REG_OUT_X_L | MULTIPLE_BYTES_MASK);
  Wire.endTransmission();
  Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
  Wire.requestFrom(IIS328DQ_SLAVE_ADDR,6);
  for(int i=0; i<6; i++) {
      axis[i] = Wire.read();
  }
  Wire.endTransmission();

  // Raw accelerometer values
  X = (axis[1] << 8) | axis[0];
  Y = (axis[3] << 8) | axis[2];
  Z = (axis[5] << 8) | axis[4];
  X_prec = X * 0.981 / 16384;
  Y_prec = Y * 0.981 / 16384;
  Z_prec = Z * 0.981 / 16384;

  // Calculate pitch
  float pitch_a = 180 * atan2(X_prec*0.061, sqrt(Y_prec*0.061*Y_prec*0.061 + Z_prec*0.061*Z_prec*0.061))/M_PI;
  pitch = alpha * pitch_a + ( 1 - alpha) * old_pitch_a;           
  old_pitch_a = pitch;

  // Target Angle
  if (pitch > targetAngle-targetAngleTolerance && pitch < targetAngle+targetAngleTolerance){
    targetAngleCycles += 1;
  }
  else{
    targetAngleCycles = 0;
  }
  if (targetAngleCycles == targetAngleCyclesGoal){
    analogWrite(en2, 0);
    analogWrite(en1, 0);
    delay(1000);
  }
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
  // Read switch state
  onoff = digitalRead(sw);
  if (onoff) {
//    int motor1 = analogRead(input1);
//    if (motor1 < 512) {
//      motor1 = map(motor1, 512, 0, 0, 255);
//      analogWrite(en1, motor1);
//      digitalWrite(ph1, LOW);
//    }
//    else {
//      motor1 = map(motor1, 512, 1024, 0, 255);
//      analogWrite(en1, motor1);
//      digitalWrite(ph1, HIGH);
//    }
//    int motor2 = analogRead(input2);
//    if (motor2 < 512) {
//      motor2 = map(motor2, 512, 0, 0, 255);
//      analogWrite(en2, motor2);
//      digitalWrite(ph2, LOW);
//    }
//    else {
//      motor2 = map(motor2, 512, 1024, 0, 255);
//      analogWrite(en2, motor2);
//      digitalWrite(ph2, HIGH);
//    }
//    Serial.println(currentPos1);
  }
  else{
//    Serial.print(String(pitch) + " " + String(targetAngleCycles));
//    Serial.println();
//    if (targetAngleReached){
//      Serial.println("Target Angle Weached");
//    }
    for (int i=0;i<15;i++){                 //  Hard code this value for each different array 
      Serial.print(String(pitch) + " " + String(targetAngleCycles));
      if (targetAngleReached){
        analogWrite(en2, 0);
        analogWrite(en1, 0);
        break;
      }
      Serial.println(i); 
      delay(10);
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
        front_backward1();
      }
      else if (sequenceArray[i] == 6){
        back_forward1();
      }
      else if (sequenceArray[i] == 7){
        front_forward1();
      }
      delay(3);
    }
  }
}

/********** Write to Accel via I2C ********/
void writeToAccel(byte reg_address, byte value) {
    Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
    Wire.write(reg_address);
    Wire.write(value);
    Wire.endTransmission();
}

/********** Moving Functions ********/
void forward1() {
  Serial.println("Forward");
  analogWrite(en2, mode2Speed);
  digitalWrite(ph2, LOW);
  delay(1);
  analogWrite(en1, mode2Speed);
  digitalWrite(ph1, HIGH);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(3);
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
  digitalWrite(ph1, LOW);
  targetEncoderCount2 = currentPos2+1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(3);
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
  analogWrite(en2, 255);
  digitalWrite(ph2, HIGH);
  delay(1);
  analogWrite(en1, 255);
  digitalWrite(ph1, HIGH);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(3);
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
  digitalWrite(ph1, LOW);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(3);
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

void front_backward1() {
  Serial.println("Front backward");
  analogWrite(en2,255);
  digitalWrite(ph2, HIGH);
  delay(1);
  analogWrite(en1, 0);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(3);
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
  digitalWrite(ph2, LOW);
  delay(1);
  analogWrite(en1, 0);
  targetEncoderCount2 = currentPos2-1;
  targetEncoderCount1 = currentPos1-1;
  while (!flag2 && !flag1){
    //Serial.println(String(currentPos2) + " " + String(targetEncoderCount2));
    delay(3);
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
  digitalWrite(ph1, HIGH);
  delay(1);
  analogWrite(en2, 0);
  delay(7000);
}
