void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
  private:
    int pinL1, pinL2, pinR1, pinR2;
    
  public:
    MotorControl(int l1, int l2, int r1, int r2);
    void init();
    void setSpeeds(int leftSpeed, int rightSpeed);
    void moveForward(int leftSpeed, int rightSpeed);
    void moveBackward(int leftSpeed, int rightSpeed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();
};

#endif