#define encoder0PinA 1
#define encoder0PinB 2
#define phase        3

int Speed;
int encoder0Pos = 0;
int i;
int previousTime;
double Setpoint;
double KP = 0.62;
double KI = 0;
double presentErr;
double errSum;
double Input;
double Output;
double Count = 0;
double iterationTime;

void setup() {
  Serial.begin(9600);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(phase, OUTPUT);
  Setpoint = 30;
 attachInterrupt(digitalPinToInterrupt(1),doEncode, RISING);
}

void loop() {
  if (i%100000 == 0) {
    double currentTime = (double)millis();
    iterationTime = (currentTime-previousTime)/1000;
    presentErr = Setpoint - Input;
    errSum += presentErr * iterationTime;
    if ((( KI * errSum) < -254) || ((KI *errSum) > 254))
    {
      errSum -= presentErr * Count;
    }
    Output = Output + KP * presentErr + KI * errSum;
    analogWrite(A4, Output);//motor spin, duty cycle (0-255)
    digitalWrite(phase, HIGH);//forward = HIGH, backward=LOW
    Serial.print("RPM:");
    Speed = ((60 * 1000 * 3.94 *  encoder0Pos/(12*379.17)) /(currentTime-previousTime));
    Serial.println(Speed);
    Input = abs(Speed);
    previousTime = (double)millis();
    encoder0Pos = 0;
     Serial.print(Input);
    Serial.print(" ");
    Serial.println(Output);
    Serial.print(" ");  
    Serial.println(Setpoint);
  }


  i++;
}

void doEncode()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
  {
  encoder0Pos++;
  }
  else
  {
  encoder0Pos--;
  }
}
