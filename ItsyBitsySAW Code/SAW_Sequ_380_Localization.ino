#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

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

//Localiztion
int zone;
float xCoordinate; // Localization output

// Accelerometer helper variables
byte X0,X1,Y0,Y1,Z0,Z1;
byte axis[6];
double X_prec,Y_prec,Z_prec;
short X,Y,Z;
float pitch=0;
float roll=0;
float old_pitch_a=0;
float old_roll_a=0;
float alpha = 0.2;

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
int sequenceArray[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,3,3};    // Leveling (380:1 motor)

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/********** Setup ********/
void setup() {
    Serial.begin(19200);
    Wire.begin();
    delay(100);
    if (tcs.begin()) {
    //Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1); // halt!
    }
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
    // Sequences
    //getAccelData();
    for (int i=0;i<15;i++){                 //  Hard code this value for each different array 
      Serial.println(i); 
      locate();
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

/** Localization with linear gradient **/
void locate() {
  float red, green, blue;
  float xPosMin;
  float xPosMax;
  tcs.getRGB(&red, &green, &blue);
  Serial.print("R:\t"); Serial.print(red); 
  Serial.print("\tG:\t"); Serial.print(green); 
  Serial.print("\tB:\t"); Serial.print(blue);
  if (red > blue && blue > green) {
    zone = 6;
    xPosMin = 6;
    xPosMax = 12.5;
    Serial.println("ZONE SIX");
  }
  else if (blue > red && red > green) {
    zone = 5;
    xPosMin = 12.5;
    xPosMax = 20;
    Serial.println("ZONE FIVE");
  }
  else if (blue > green && green > red) {
    zone = 4;
    xPosMin = 20;
    xPosMax = 48;
    Serial.println("ZONE FOUR");
  }
  else if (green > blue && blue > red) {
    zone = 3;
    xPosMin = 48;
    xPosMax = 54.5;
    Serial.println("ZONE THREE");
  }
  else if (green > red && red > blue) {
    zone = 2;
    xPosMin = 48;
    xPosMax = 65;
    Serial.println("ZONE TWO");
  }
  else if (red > green && green > blue) {
    zone = 1;
    xPosMin = 65;
    xPosMax = 84;
    Serial.println("ZONE ONE");
  }
  else {
    zone = -1;
    Serial.println("UNKNOWN ZONE");
  }
  float diffIndex = 9999999;
  while(xPosMin<xPosMax) {
    float RTemp = 179 - 8.16 * xPosMin + 0.13 * pow(xPosMin,2) - 0.000342*pow(xPosMin,3);
    float GTemp = 47.2 - 0.918 * xPosMin + 0.0933 * pow(xPosMin,2) - 0.00105*pow(xPosMin,3);
    float BTemp = 22.8 - 9.04 * xPosMin - 0.235 * pow(xPosMin,2) - 0.00155*pow(xPosMin,3);
    if ( (RTemp+GTemp+BTemp) < diffIndex) {
      diffIndex = RTemp+GTemp+BTemp;
      xCoordinate = xPosMin;
    }
    xPosMin = xPosMin + 0.5;
  }
  Serial.print("Current Position is: ");
  Serial.println(xCoordinate); 
}

void getAccelData(){
  // Accelerometer - I2C
  Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
  Wire.write(IIS328DQ_REG_OUT_X_L  | MULTIPLE_BYTES_MASK);
  Wire.endTransmission();
  Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
  Wire.requestFrom(IIS328DQ_SLAVE_ADDR,6);
  for(int i=0; i<6; i++) {
      axis[i] = Wire.read();
  }
  Wire.endTransmission();

  X = (axis[1] << 8) | axis[0];
  Y = (axis[3] << 8) | axis[2];
  Z = (axis[5] << 8) | axis[4];
  /* CALIBRATION:
      2^16 = 65536 (dinamic range quantification)
      +/- 2g = 4g   ->    65536 / 4 = 16384    */
  X_prec = X * 0.981 / 16384;
  Y_prec = Y * 0.981 / 16384;
  Z_prec = Z * 0.981 / 16384;

  float roll_a = 180 * atan2(Y_prec*0.061, sqrt(X_prec*0.061*X_prec*0.061 + Z_prec*0.061*Z_prec*0.061))/M_PI;  
  float pitch_a = 180 * atan2(X_prec*0.061, sqrt(Y_prec*0.061*Y_prec*0.061 + Z_prec*0.061*Z_prec*0.061))/M_PI;

  roll = alpha * roll_a + ( 1 - alpha) * old_roll_a; 
  pitch = alpha * pitch_a + ( 1 - alpha) * old_pitch_a;           
  old_pitch_a = pitch;
  old_roll_a = roll;

  Serial.print(" ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(" ");
  Serial.print(pitch);
  Serial.println();
                        
  delay(50);
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

/********** Write to Accel via I2C ********/
void writeToAccel(byte reg_address, byte value) {
    Wire.beginTransmission(IIS328DQ_SLAVE_ADDR);
    Wire.write(reg_address);
    Wire.write(value);
    Wire.endTransmission();
}
