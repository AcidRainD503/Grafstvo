void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
#include <Wire.h>
#include <Servo.h>
#include "motor_control.h"
#include "sensor_control.h"

// Пины
#define MOTOR_L1 5
#define MOTOR_L2 6
#define MOTOR_R1 9
#define MOTOR_R2 10

#define SERVO_BASE 3
#define SERVO_ARM  11
#define SERVO_GRIP 12

#define IR_LEFT   A0
#define IR_RIGHT  A1
#define IR_FRONT  A2
#define US_TRIG   7
#define US_ECHO   8

// Объекты
MotorControl motors(MOTOR_L1, MOTOR_L2, MOTOR_R1, MOTOR_R2);
SensorControl sensors(IR_LEFT, IR_RIGHT, IR_FRONT, US_TRIG, US_ECHO);
Servo servoBase, servoArm, servoGrip;

// Состояние
enum State { IDLE, MOVING, GRABBING, RELEASING, AVOIDING };
State currentState = IDLE;

// Навигация
float currentX = 0;
float currentY = 0;
float currentAngle = 0;

// Цели
float targetX = 0;
float targetY = 0;

// Константы
const float WHEEL_DIAMETER = 6.5; // см
const float WHEEL_DISTANCE = 15.0; // см между колесами
const float TICKS_PER_REV = 20; // тиков энкодера на оборот

// Энкодеры
volatile int leftTicks = 0;
volatile int rightTicks = 0;

void setup() {
  Serial.begin(115200);
  
  // Инициализация
  motors.init();
  sensors.init();
  
  // Настройка сервоприводов
  servoBase.attach(SERVO_BASE);
  servoArm.attach(SERVO_ARM);
  servoGrip.attach(SERVO_GRIP);
  
  // Начальная позиция манипулятора
  servoBase.write(90);
  servoArm.write(30);
  servoGrip.write(90);
  
  // Настройка прерываний для энкодеров
  // (пины энкодеров должны поддерживать прерывания)
  attachInterrupt(digitalPinToInterrupt(2), countLeftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), countRightTick, CHANGE);
  
  Serial.println("ARDUINO_READY");
}

void loop() {
  // Чтение команд из Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Основной цикл управления
  switch (currentState) {
    case MOVING:
      updateNavigation();
      moveToTarget();
      checkObstacles();
      break;
      
    case GRABBING:
      performGrab();
      break;
      
    case RELEASING:
      performRelease();
      break;
      
    case AVOIDING:
      avoidObstacle();
      break;
      
    case IDLE:
      // Ничего не делаем, ждем команды
      break;
  }
  
  // Отправка данных о состоянии
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 500) {
    sendStatus();
    lastSendTime = millis();
  }
}

void processCommand(String command) {
  if (command.startsWith("MOVE_TO")) {
    // Команда: MOVE_TO X Y
    int space1 = command.indexOf(' ');
    int space2 = command.indexOf(' ', space1 + 1);
    
    if (space1 > 0 && space2 > 0) {
      targetX = command.substring(space1 + 1, space2).toFloat();
      targetY = command.substring(space2 + 1).toFloat();
      
      Serial.print("MOVING_TO ");
      Serial.print(targetX);
      Serial.print(" ");
      Serial.println(targetY);
      
      currentState = MOVING;
    }
  }
  else if (command == "STOP") {
    motors.stop();
    currentState = IDLE;
    Serial.println("STOPPED");
  }
  else if (command == "GRAB") {
    currentState = GRABBING;
    Serial.println("GRABBING_STARTED");
  }
  else if (command == "RELEASE") {
    currentState = RELEASING;
    Serial.println("RELEASING_STARTED");
  }
  else if (command == "PATROL") {
    // Режим патрулирования для поиска объектов
    patrolMode();
  }
  else if (command.startsWith("GET_")) {
    if (command == "GET_ULTRASONIC") {
      float distance = sensors.getUltrasonicDistance();
      Serial.print("ULTRASONIC ");
      Serial.println(distance);
    }
    else if (command == "GET_IR") {
      int leftIR = sensors.getIRLeft();
      int rightIR = sensors.getIRRight();
      Serial.print("IR ");
      Serial.print(leftIR);
      Serial.print(" ");
      Serial.println(rightIR);
    }
    else if (command == "GET_POSITION") {
      Serial.print("POSITION ");
      Serial.print(currentX);
      Serial.print(" ");
      Serial.println(currentY);
    }
  }
}
void updateNavigation() {
  // Обновление позиции по одометрии
  float leftDist = (leftTicks / TICKS_PER_REV) * PI * WHEEL_DIAMETER;
  float rightDist = (rightTicks / TICKS_PER_REV) * PI * WHEEL_DIAMETER;
  
  float deltaDist = (leftDist + rightDist) / 2;
  float deltaAngle = (rightDist - leftDist) / WHEEL_DISTANCE;
  
  currentX += deltaDist * cos(currentAngle + deltaAngle / 2);
  currentY += deltaDist * sin(currentAngle + deltaAngle / 2);
  currentAngle += deltaAngle;
  
  // Сброс счетчиков
  leftTicks = 0;
  rightTicks = 0;
}

void moveToTarget() {
  // Вычисление вектора к цели
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  
  float distanceToTarget = sqrt(dx*dx + dy*dy);
  float targetAngle = atan2(dy, dx);
  
  // Вычисление угловой ошибки
  float angleError = targetAngle - currentAngle;
  
  // Нормализация угла
  while (angleError > PI) angleError -= 2*PI;
  while (angleError < -PI) angleError += 2*PI;
  
  // Пропорциональный регулятор
  float baseSpeed = 150; // Базовая скорость
  float turnSpeed = 100 * angleError; // Скорость поворота
  
  int leftSpeed = baseSpeed - turnSpeed;
  int rightSpeed = baseSpeed + turnSpeed;
  
  // Ограничение скорости
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Установка скорости моторов
  motors.setSpeeds(leftSpeed, rightSpeed);
  
  // Проверка достижения цели
  if (distanceToTarget < 5.0) { // 5 см
    motors.stop();
    currentState = IDLE;
    Serial.println("TARGET_REACHED");
  }
}

void checkObstacles() {
  float frontDistance = sensors.getUltrasonicDistance();
  int leftEdge = sensors.getIRLeft();
  int rightEdge = sensors.getIRRight();
  
  // Проверка препятствий
  if (frontDistance < 15.0) {
    currentState = AVOIDING;
    Serial.println("OBSTACLE_DETECTED");
  }
  
  // Проверка края полигона
  if (leftEdge > 500 || rightEdge > 500) {
    motors.stop();
    
    // Отъезд от края
    motors.moveBackward(100, 100);
    delay(300);
    motors.turnRight(100);
    delay(200);
    
    Serial.println("EDGE_DETECTED");
    currentState = MOVING;
  }
}

void avoidObstacle() {
  motors.stop();
  
  // Простой алгоритм объезда
  motors.moveBackward(100, 100);
  delay(300);
  
  // Поворот в случайном направлении
  if (random(2) == 0) {
    motors.turnLeft(100);
  } else {
    motors.turnRight(100);
  }
  
  delay(400);
  currentState = MOVING;
}

void performGrab() {
  // Процедура захвата объекта
  motors.stop();
  
  // 1. Опускаем манипулятор
  for (int pos = 30; pos <= 80; pos += 1) {
    servoArm.write(pos);
    delay(15);
  }
  
  // 2. Захватываем объект
  for (int pos = 90; pos >= 30; pos -= 1) {
    servoGrip.write(pos);
    delay(15);
  }
  
  // 3. Поднимаем объект
  for (int pos = 80; pos >= 30; pos -= 1) {
    servoArm.write(pos);
    delay(15);
  }
  
  currentState = IDLE;
  Serial.println("GRAB_SUCCESS");
}

void performRelease() {
  // Процедура сброса объекта
  motors.stop();
  
  // 1. Опускаем манипулятор над люком
  for (int pos = 30; pos <= 80; pos += 1) {
    servoArm.write(pos);
    delay(15);
  }
  
  // 2. Отпускаем объект
  for (int pos = 30; pos <= 90; pos += 1) {
    servoGrip.write(pos);
    delay(15);
  }
  
  // 3. Поднимаем манипулятор
  for (int pos = 80; pos >= 30; pos -= 1) {
    servoArm.write(pos);
    delay(15);
  }
  
  // 4. Возвращаем в исходное положение
  servoBase.write(90);
  
  currentState = IDLE;
  Serial.println("RELEASE_SUCCESS");
}

void patrolMode() {
  // Режим патрулирования для поиска объектов
  static int patrolDirection = 1;
  
  motors.setSpeeds(100 * patrolDirection, 100 * patrolDirection);
  
  // Проверка краев
  int leftEdge = sensors.getIRLeft();
  int rightEdge = sensors.getIRRight();
  
  if (leftEdge > 500 || rightEdge > 500) {
    motors.stop();
    patrolDirection *= -1; // Меняем направление
    delay(500);
  }
}
void sendStatus() {
  // Отправка состояния
  Serial.print("STATUS ");
  Serial.print(currentX);
  Serial.print(" ");
  Serial.print(currentY);
  Serial.print(" ");
  Serial.print(currentAngle * 180 / PI);
  Serial.print(" ");
  Serial.println(currentState);
}

// Обработчики прерываний для энкодеров
void countLeftTick() {
  leftTicks++;
}

void countRightTick() {
  rightTicks++;
}