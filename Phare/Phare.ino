#include <Arduino.h>
#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo

int motor_11 = 2;
int motor_12 = 3;
int motor_21 = 4;
int motor_22 = 5;
int motor_pwm1 = 12;
int motor_pwm2 = 11;
byte motor_speed = 100;
int b1 = 8;
int b2 = 10;
int trigger = 0;
float times = 0;
int triggerr = 0;
int b11 = 6;
int b22 = 7;
int on = false;
int i = 180;

void setup() {
  Serial.begin(9600);
  pinMode(motor_11, OUTPUT);
  pinMode(motor_12, OUTPUT);
  pinMode(motor_21, OUTPUT);
  pinMode(motor_22, OUTPUT);
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, OUTPUT);
  digitalWrite(b2, LOW);
  // pinMode(trigger, OUTPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);  // 一開始先置中90度
  pinMode(b11, INPUT_PULLUP);
  pinMode(b22, OUTPUT);
  digitalWrite(b22, LOW);
}

void loop() {
  // Serial.println("test");
  digitalWrite(motor_11, LOW);
  digitalWrite(motor_12, LOW);
  digitalWrite(motor_21, LOW);
  digitalWrite(motor_22, LOW);
  times = 0;
  /*if(!digitalRead(b1) && on) {
    // Serial.println("OK");
    trigger = 1;
    while(times <= 1.0) {
      // Serial.println(times);
      digitalWrite(motor_21, HIGH);
      digitalWrite(motor_22, LOW);
      analogWrite(motor_pwm2, motor_speed);
      digitalWrite(motor_11, HIGH);
      digitalWrite(motor_12, LOW);
      analogWrite(motor_pwm1, 255);
      delay(1500);
      times += 0.5;
    }
  }*/
  if(!digitalRead(b1)) {
    // Serial.println("OK");
    trigger = 1;
    while(times <= 1.0) {
      // Serial.println(times);
      digitalWrite(motor_21, HIGH);
      digitalWrite(motor_22, LOW);
      analogWrite(motor_pwm2, motor_speed);
      digitalWrite(motor_11, HIGH);
      digitalWrite(motor_12, LOW);
      analogWrite(motor_pwm1, 255);
      delay(1500);
      times += 0.5;
    }
  }
  if(trigger) {
    digitalWrite(motor_11, HIGH);
    digitalWrite(motor_12, LOW);
    analogWrite(motor_pwm1, 255);
  }
  if(!digitalRead(b11)) {
    triggerr = 1;
  }
  if(i>=0) {
    if(!triggerr) {
      Serial.println("0");
      myservo.write(i);
      delay(300);
    } else {
      Serial.println("1");
      myservo.write(90);
    }
    i-=1;
  }
}
