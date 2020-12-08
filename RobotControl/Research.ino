#define encoder0PinA 1
#define encoder0PinB 2
#define phase        3

int Speed;
int encoder0Pos = 0;
int i;
long previousTime;
long currentTime;

void setup() {
  Serial.begin(9600);
  pinMode(encoder0PinA, INPUT);
 pinMode(encoder0PinB, INPUT);
 pinMode(phase, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(1),doEncode, RISING);
}

void loop() {
  
  analogWrite(A4, 255);//motor spin, duty cycle (0-255)
  digitalWrite(phase, HIGH);//forward = HIGH, backward=LOW


    Serial.print("RPM:");
    currentTime = micros();
    //Serial.println(currentTime-previousTime);
    long Speed = ((60 * 1000000 * 3.94 *  encoder0Pos/(12*379.17)) /(currentTime-previousTime));
    Serial.println(Speed);
    previousTime = micros();
    encoder0Pos = 0;


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
