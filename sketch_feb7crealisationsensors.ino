void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
#include "sensor_control.h"

SensorControl::SensorControl(int irL, int irR, int irF, int trig, int echo) {
  irLeftPin = irL;
  irRightPin = irR;
  irFrontPin = irF;
  usTrigPin = trig;
  usEchoPin = echo;
}

void SensorControl::init() {
  pinMode(irLeftPin, INPUT);
  pinMode(irRightPin, INPUT);
  pinMode(irFrontPin, INPUT);
  pinMode(usTrigPin, OUTPUT);
  pinMode(usEchoPin, INPUT);
}

int SensorControl::getIRLeft() {
  return analogRead(irLeftPin);
}

int SensorControl::getIRRight() {
  return analogRead(irRightPin);
}

int SensorControl::getIRFront() {
  return analogRead(irFrontPin);
}

float SensorControl::getUltrasonicDistance() {
  digitalWrite(usTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(usTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(usTrigPin, LOW);
  
  long duration = pulseIn(usEchoPin, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  
  if (distance == 0 || distance > 200) {
    return 200.0; // Максимальная дистанция
  }
  
  return distance;
}