void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include <Arduino.h>

class SensorControl {
  private:
    int irLeftPin, irRightPin, irFrontPin;
    int usTrigPin, usEchoPin;
    
  public:
    SensorControl(int irL, int irR, int irF, int trig, int echo);
    void init();
    int getIRLeft();
    int getIRRight();
    int getIRFront();
    float getUltrasonicDistance();
};

#endif