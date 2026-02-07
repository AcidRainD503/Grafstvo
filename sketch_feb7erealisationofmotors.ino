void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
#include "motor_control.h"

MotorControl::MotorControl(int l1, int l2, int r1, int r2) {
  pinL1 = l1;
  pinL2 = l2;
  pinR1 = r1;
  pinR2 = r2;
}

void MotorControl::init() {
  pinMode(pinL1, OUTPUT);
  pinMode(pinL2, OUTPUT);
  pinMode(pinR1, OUTPUT);
  pinMode(pinR2, OUTPUT);
  stop();
}

void MotorControl::setSpeeds(int leftSpeed, int rightSpeed) {
  // Левый мотор
  if (leftSpeed > 0) {
    analogWrite(pinL1, leftSpeed);
    analogWrite(pinL2, 0);
  } else if (leftSpeed < 0) {
    analogWrite(pinL1, 0);
    analogWrite(pinL2, -leftSpeed);
  } else {
    analogWrite(pinL1, 0);
    analogWrite(pinL2, 0);
  }
  
  // Правый мотор
  if (rightSpeed > 0) {
    analogWrite(pinR1, rightSpeed);
    analogWrite(pinR2, 0);
  } else if (rightSpeed < 0) {
    analogWrite(pinR1, 0);
    analogWrite(pinR2, -rightSpeed);
  } else {
    analogWrite(pinR1, 0);
    analogWrite(pinR2, 0);
  }
}

void MotorControl::moveForward(int leftSpeed, int rightSpeed) {
  setSpeeds(leftSpeed, rightSpeed);
}

void MotorControl::moveBackward(int leftSpeed, int rightSpeed) {
  setSpeeds(-leftSpeed, -rightSpeed);
}

void MotorControl::turnLeft(int speed) {
  setSpeeds(-speed, speed);
}

void MotorControl::turnRight(int speed) {
  setSpeeds(speed, -speed);
}

void MotorControl::stop() {
  setSpeeds(0, 0);
}