#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define FAILPIN 2
#define WORKPIN 3
#define DELAYTIME 1000
#define SERVODELAY 1000
#define SERVOMIN  2000 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  4096 // this is the 'maximum' pulse length count (out of 4096)
#define LED1 5
#define SERVO1 12
#define FIRE1 4
#define PINWHEEL 8
#define PINWHEELMAX 4096
#define FIREMAX 4096

int failState = 0;
int workState = 0;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(FAILPIN, INPUT);
  pinMode(WORKPIN, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
   pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //powerup();
  Serial.println("running");
}

void servoMove() {
  Serial.print("Servo to SERVOMAX");
  pwm.setPWM(SERVO1, 0, SERVOMAX);
  delay(SERVODELAY);
  Serial.print("Servo to SERVOMIN");
  pwm.setPWM(SERVO1, 0, SERVOMIN);
}

void pinWheelOn(){
  Serial.print("PinWheel on");
  pwm.setPWM(PINWHEEL, 0, PINWHEELMAX);
}

void fireOn(){
  Serial.print("FirePot on");
  pwm.setPWM(FIRE1, 0, FIREMAX);
}

void pinWheelOff(){
  Serial.print("PinWheel off");
  pwm.setPWM(PINWHEEL, 0, 0);
}

void fireOff(){
  Serial.print("FirePot off");
  pwm.setPWM(FIRE1, 0, 0);
}

void ledOn(){
  Serial.print("LED on");
  analogWrite(LED1, 255);
}

void ledSet(int ledVal){
  Serial.print("LED to value");
  analogWrite(LED1, ledVal);
}

void ledOff(){
  Serial.print("LED off");
  analogWrite(LED1, 0);
}

void failure() {
  ledOn();

  delay(DELAYTIME);

  pinWheelOn();

  delay(DELAYTIME);
  
  servoMove();
  delay(SERVODELAY);
  servoMove();
  delay(SERVODELAY);
  servoMove();
  delay(SERVODELAY);
  servoMove();

  pinWheelOff();
  
  fireOn();
  
  ledSet(78);
  delay(SERVODELAY);
  ledSet(200);
  delay(SERVODELAY);
  ledSet(56);
  servoMove();
  delay(SERVODELAY);
  ledSet(125);
  servoMove();
  delay(SERVODELAY);
  ledOff();
  fireOff();
}

void success() {
  ledOn();

  delay(DELAYTIME);

  pinWheelOn();

  delay(DELAYTIME);
  delay(DELAYTIME);
  pinWheelOff();

  ledOff();
  delay(SERVODELAY);
  ledOn();
  delay(SERVODELAY);
  ledOff();
  delay(SERVODELAY);
  ledOn();
  delay(SERVODELAY);
  ledOff();
  delay(SERVODELAY);
  ledOn();
  delay(SERVODELAY);
  ledOff();
}

void loop() {
  // put your main code here, to run repeatedly:
  failState = digitalRead(FAILPIN);
  workState = digitalRead(WORKPIN);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(SERVODELAY);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(SERVODELAY);  
  //failState=HIGH;
  if (failState == HIGH) {
    Serial.print("Fail");
    failure();
  }

  if (workState == HIGH) {
    Serial.print("Succeed");
    success();
  }
}
