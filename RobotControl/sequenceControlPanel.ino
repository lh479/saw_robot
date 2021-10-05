#include <IRremote.h>  // use the IRRemote.h
const int irReceiverPin = 2;  //the SIG of receiver module attach to pin2
IRrecv irrecv(irReceiverPin); //Creates a variable of type IRrecv
decode_results results;    // define resultsas 
//int selectPin = 2;
int ph1 = 7;
int en1 = 9;
int ph2 = 10;
int en2 = 11;
int sw = 13;
int input1 = A4;
int input2 = A3;
bool onoff = HIGH;
int mode2Speed = 255;

void setup() {
  Serial.begin(9600); 
  pinMode(ph1, OUTPUT); //Motor 1
  pinMode(ph2, OUTPUT); //Motor 2
  pinMode(sw, INPUT);
  pinMode(selectPin, INPUT);
  irrecv.enableIRIn();   // enable ir receiver module
}

void loop() {
  onoff = digitalRead(sw);

  if (onoff) {
    int motor1 = analogRead(input1);
    Serial.println(motor1);
    if (motor1 < 512) {
      motor1 = map(motor1, 512, 0, 0, 255);
      Serial.println("CCW out:");
      Serial.println(motor1);
      analogWrite(en1, motor1);
      digitalWrite(ph1, LOW);
    }
    else {
      motor1 = map(motor1, 512, 1024, 0, 255);
      Serial.println("CW out:");
      Serial.println(motor1);
      analogWrite(en1, motor1);
      digitalWrite(ph1, HIGH);
    }

    int motor2 = analogRead(input2);
    Serial.println(motor2);
    if (motor2 < 512) {
      motor2 = map(motor2, 512, 0, 0, 255);
      Serial.println("CCW out:");
      Serial.println(motor2);
      analogWrite(en2, motor2);
      digitalWrite(ph2, LOW);
    }
    else {
      motor2 = map(motor2, 512, 1024, 0, 255);
      Serial.println("CW out:");
      Serial.println(motor2);
      analogWrite(en2, motor2);
      digitalWrite(ph2, HIGH);
    }
  }
  else {
    if (irrecv.decode(&results)) //if the ir receiver module receiver data
    {
       int sequenceNum = results.value;
       switch (sequenceNum) {
        case 1:
          forward1()；
          break;
        case 2:
          backward1()；
          break;
        case 3:
          inward1();
          break:
        case 4:
          outward1();
          break;
        default:
          analogWrite(en1, 0);
          analogWrite(en2, 0);
          break;
       }
      irrecv.resume();
    }
    else {
      analogWrite(en1, 0);
      analogWrite(en2, 0);
    }
  }
}

void forward1() {
  digitalWrite(ph1, LOW);
  digitalWrite(ph2, HIGH);
  analogWrite(en1, mode2Speed);
  analogWrite(en2, mode2Speed);
  delay(900);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  delay(3000); 
}

void backward1() {
  digitalWrite(ph2, LOW);
  digitalWrite(ph1, HIGH);
  analogWrite(en1, mode2Speed);
  analogWrite(en2, mode2Speed);
  delay(900);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  delay(3000); 
}

void outward1() {
  digitalWrite(ph2, LOW);
  digitalWrite(ph1, LOW);
  analogWrite(en1, mode2Speed);
  analogWrite(en2, mode2Speed);
  delay(900);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  delay(3000); 
}

void inward1() {
  digitalWrite(ph2, HIGH);
  digitalWrite(ph1, HIGH);
  analogWrite(en1, mode2Speed);
  analogWrite(en2, mode2Speed);
  delay(900);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  delay(3000); 
}
