// ============================================
// РОБОТ "БЕЗОПАСНЫЙ МАРШРУТ"
// КОД ДЛЯ ARDUINO (ИСПОЛНИТЕЛЬНЫЙ КОНТРОЛЛЕР)
// Версия 1.0
// ============================================

// === БИБЛИОТЕКИ ===
#include <Servo.h>

// === КОНСТАНТЫ И НАСТРОЙКИ ===
// Пины для моторов (L298N драйвер)
#define MOTOR_L1 5    // Левый мотор, контакт 1
#define MOTOR_L2 6    // Левый мотор, контакт 2
#define MOTOR_R1 9    // Правый мотор, контакт 1
#define MOTOR_R2 10   // Правый мотор, контакт 2

// Пины для сервоприводов манипулятора
#define SERVO_BASE 3   // Поворот основания
#define SERVO_ARM  11  // Подъем/опускание рычага
#define SERVO_GRIP 12  // Захват

// Пины для датчиков
#define IR_LEFT   A0   // ИК-датчик левого края
#define IR_RIGHT  A1   // ИК-датчик правого края
#define IR_FRONT  A2   // ИК-датчик спереди (опционально)
#define US_TRIG   7    // Ультразвуковой датчик TRIG
#define US_ECHO   8    // Ультразвуковой датчик ECHO

// Константы для навигации
#define WHEEL_DIAMETER_CM 6.5      // Диаметр колеса в см
#define WHEEL_DISTANCE_CM 15.0     // Расстояние между колесами в см
#define TICKS_PER_REVOLUTION 20    // Тиков энкодера на один оборот
#define EDGE_THRESHOLD 500         // Порог срабатывания датчика края
#define OBSTACLE_THRESHOLD_CM 15.0 // Минимальное расстояние до препятствия

// === ПЕРЕМЕННЫЕ И ОБЪЕКТЫ ===
// Объекты сервоприводов
Servo servoBase;
Servo servoArm;
Servo servoGrip;

// Состояние робота
enum RobotState {
    STATE_IDLE,        // Ожидание команд
    STATE_MOVING,      // Движение к цели
    STATE_GRABBING,    // Захват объекта
    STATE_RELEASING,   // Сброс объекта
    STATE_AVOIDING,    // Объезд препятствия
    STATE_PATROL       // Патрулирование/поиск
};

RobotState currentState = STATE_IDLE;

// Навигационные переменные
volatile int leftEncoderTicks = 0;  // Счетчик тиков левого энкодера
volatile int rightEncoderTicks = 0; // Счетчик тиков правого энкодера

float currentX = 0.0;      // Текущая X координата (см)
float currentY = 0.0;      // Текущая Y координата (см)
float currentAngle = 0.0;  // Текущий угол (радианы)

float targetX = 0.0;       // Целевая X координата
float targetY = 0.0;       // Целевая Y координата

// Время последнего опроса
unsigned long lastSensorRead = 0;
unsigned long lastStatusSend = 0;

// === ПРОТОТИПЫ ФУНКЦИЙ ===
// Управление моторами
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward(int leftSpeed, int rightSpeed);
void turnLeft(int speed);
void turnRight(int speed);

// Управление манипулятором
void initManipulator();
void grabObject();
void releaseObject();
void resetManipulator();

// Датчики
float readUltrasonicDistance();
int readIRSensorLeft();
int readIRSensorRight();
int readIRSensorFront();
void updatePositionFromEncoders();

// Навигация и движение
void moveToTarget();
void checkObstaclesAndEdges();
void avoidObstacle();
void patrolMode();

// Коммуникация
void processSerialCommand(String command);
void sendStatus();

// Обработчики прерываний энкодеров
void countLeftEncoder();
void countRightEncoder();

// === НАСТРОЙКА (SETUP) ===
void setup() {
    // Инициализация последовательного порта
    Serial.begin(115200);
    while (!Serial) {
        ; // Ожидание подключения последовательного порта
    }

    Serial.println("=== РОБОТ 'БЕЗОПАСНЫЙ МАРШРУТ' ===");
    Serial.println("Инициализация...");

    // Настройка пинов моторов
    pinMode(MOTOR_L1, OUTPUT);
    pinMode(MOTOR_L2, OUTPUT);
    pinMode(MOTOR_R1, OUTPUT);
    pinMode(MOTOR_R2, OUTPUT);
    stopMotors();

    // Настройка пинов датчиков
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(IR_FRONT, INPUT);
    pinMode(US_TRIG, OUTPUT);
    pinMode(US_ECHO, INPUT);
    digitalWrite(US_TRIG, LOW);

    // Инициализация манипулятора
    initManipulator();

    // Настройка прерываний для энкодеров
    // ПРИМЕЧАНИЕ: используйте пины, поддерживающие прерывания (2, 3 на Arduino Uno)
    attachInterrupt(digitalPinToInterrupt(2), countLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), countRightEncoder, CHANGE);

    Serial.println("Инициализация завершена");
    Serial.println("ARDUINO_READY");
    Serial.println("=== Готов к работе ===");
}

// === ОСНОВНОЙ ЦИКЛ (LOOP) ===
void loop() {
    // 1. Чтение команд из последовательного порта
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processSerialCommand(command);
    }

    // 2. Выполнение текущего состояния
    switch (currentState) {
    case STATE_MOVING:
        updatePositionFromEncoders();
        moveToTarget();
        checkObstaclesAndEdges();
        break;

    case STATE_GRABBING:
        grabObject();
        currentState = STATE_IDLE;
        break;

    case STATE_RELEASING:
        releaseObject();
        currentState = STATE_IDLE;
        break;

    case STATE_AVOIDING:
        avoidObstacle();
        currentState = STATE_MOVING;
        break;

    case STATE_PATROL:
        patrolMode();
        checkObstaclesAndEdges();
        break;

    case STATE_IDLE:
        // Ничего не делаем, ждем команд
        break;
    }

    // 3. Периодическая отправка статуса
    if (millis() - lastStatusSend > 500) {
        sendStatus();
        lastStatusSend = millis();
    }

    // Небольшая задержка для стабильности
    delay(10);
}

// === РЕАЛИЗАЦИЯ ФУНКЦИЙ ===

// ---------- УПРАВЛЕНИЕ МОТОРАМИ ----------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Ограничение скорости от -255 до 255
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Левый мотор
    if (leftSpeed > 0) {
        analogWrite(MOTOR_L1, leftSpeed);
        analogWrite(MOTOR_L2, 0);
    }
    else if (leftSpeed < 0) {
        analogWrite(MOTOR_L1, 0);
        analogWrite(MOTOR_L2, -leftSpeed);
    }
    else {
        analogWrite(MOTOR_L1, 0);
        analogWrite(MOTOR_L2, 0);
    }

    // Правый мотор
    if (rightSpeed > 0) {
        analogWrite(MOTOR_R1, rightSpeed);
        analogWrite(MOTOR_R2, 0);
    }
    else if (rightSpeed < 0) {
        analogWrite(MOTOR_R1, 0);
        analogWrite(MOTOR_R2, -rightSpeed);
    }
    else {
        analogWrite(MOTOR_R1, 0);
        analogWrite(MOTOR_R2, 0);
    }
}

void stopMotors() {
    setMotorSpeeds(0, 0);
}

void moveForward(int leftSpeed, int rightSpeed) {
    setMotorSpeeds(leftSpeed, rightSpeed);
}

void moveBackward(int leftSpeed, int rightSpeed) {
    setMotorSpeeds(-leftSpeed, -rightSpeed);
}

void turnLeft(int speed) {
    setMotorSpeeds(-speed, speed);
}

void turnRight(int speed) {
    setMotorSpeeds(speed, -speed);
}

// ---------- МАНИПУЛЯТОР ----------
void initManipulator() {
    servoBase.attach(SERVO_BASE);
    servoArm.attach(SERVO_ARM);
    servoGrip.attach(SERVO_GRIP);

    // Установка в начальное положение
    servoBase.write(90);   // Центр
    delay(300);
    servoArm.write(30);    // Поднято
    delay(300);
    servoGrip.write(90);   // Открыт
    delay(300);

    Serial.println("Манипулятор инициализирован");
}

void grabObject() {
    Serial.println("Начало захвата объекта...");

    // 1. Опускаем рычаг
    for (int angle = 30; angle <= 80; angle += 2) {
        servoArm.write(angle);
        delay(30);
    }
    delay(300);

    // 2. Захватываем объект
    for (int angle = 90; angle >= 40; angle -= 2) {
        servoGrip.write(angle);
        delay(30);
    }
    delay(300);

    // 3. Поднимаем объект
    for (int angle = 80; angle >= 30; angle -= 2) {
        servoArm.write(angle);
        delay(30);
    }
    delay(300);

    Serial.println("GRAB_SUCCESS");
}

void releaseObject() {
    Serial.println("Сброс объекта...");

    // 1. Опускаем рычаг над люком
    for (int angle = 30; angle <= 80; angle += 2) {
        servoArm.write(angle);
        delay(30);
    }
    delay(300);

    // 2. Открываем захват
    for (int angle = 40; angle <= 90; angle += 2) {
        servoGrip.write(angle);
        delay(30);
    }
    delay(500); // Даем время объекту упасть

    // 3. Поднимаем рычаг
    for (int angle = 80; angle >= 30; angle -= 2) {
        servoArm.write(angle);
        delay(30);
    }
    delay(300);

    // 4. Возвращаем основание в центр
    servoBase.write(90);
    delay(300);

    Serial.println("RELEASE_SUCCESS");
}

void resetManipulator() {
    servoBase.write(90);
    servoArm.write(30);
    servoGrip.write(90);
}

// ---------- ДАТЧИКИ ----------
float readUltrasonicDistance() {
    // Генерируем импульс
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);

    // Измеряем время отклика
    long duration = pulseIn(US_ECHO, HIGH, 30000); // Таймаут 30 мс

    // Преобразуем время в расстояние (см)
    // Скорость звука ~340 м/с = 0.034 см/мкс
    float distance = duration * 0.034 / 2;

    // Проверка на корректность
    if (distance <= 0 || distance > 200) {
        return 200.0; // Максимальное расстояние
    }

    return distance;
}

int readIRSensorLeft() {
    return analogRead(IR_LEFT);
}

int readIRSensorRight() {
    return analogRead(IR_RIGHT);
}

int readIRSensorFront() {
    return analogRead(IR_FRONT);
}

void updatePositionFromEncoders() {
    // Вычисляем пройденное расстояние каждым колесом
    float leftDistance = (leftEncoderTicks / (float)TICKS_PER_REVOLUTION) *
        PI * WHEEL_DIAMETER_CM;
    float rightDistance = (rightEncoderTicks / (float)TICKS_PER_REVOLUTION) *
        PI * WHEEL_DIAMETER_CM;

    // Среднее расстояние
    float deltaDistance = (leftDistance + rightDistance) / 2.0;

    // Изменение угла
    float deltaAngle = (rightDistance - leftDistance) / WHEEL_DISTANCE_CM;

    // Обновляем позицию
    currentX += deltaDistance * cos(currentAngle + deltaAngle / 2.0);
    currentY += deltaDistance * sin(currentAngle + deltaAngle / 2.0);
    currentAngle += deltaAngle;

    // Нормализуем угол в диапазон [-π, π]
    while (currentAngle > PI) currentAngle -= 2 * PI;
    while (currentAngle < -PI) currentAngle += 2 * PI;

    // Сбрасываем счетчики
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
}

// ---------- НАВИГАЦИЯ ----------
void moveToTarget() {
    // Вычисляем вектор к цели
    float dx = targetX - currentX;
    float dy = targetY - currentY;

    // Расстояние до цели
    float distanceToTarget = sqrt(dx * dx + dy * dy);

    // Если достигли цели
    if (distanceToTarget < 5.0) { // 5 см
        stopMotors();
        currentState = STATE_IDLE;
        Serial.println("TARGET_REACHED");
        return;
    }

    // Желаемый угол к цели
    float targetAngle = atan2(dy, dx);

    // Ошибка угла
    float angleError = targetAngle - currentAngle;

    // Нормализация ошибки угла
    while (angleError > PI) angleError -= 2 * PI;
    while (angleError < -PI) angleError += 2 * PI;

    // Пропорциональный регулятор
    float baseSpeed = 150.0; // Базовая скорость
    float turnGain = 1.5;    // Коэффициент поворота

    // Расчет скоростей
    int leftSpeed = baseSpeed - turnGain * angleError * 100;
    int rightSpeed = baseSpeed + turnGain * angleError * 100;

    // Применение скоростей
    setMotorSpeeds(leftSpeed, rightSpeed);
}

void checkObstaclesAndEdges() {
    // Проверка препятствий спереди
    float frontDistance = readUltrasonicDistance();
    if (frontDistance < OBSTACLE_THRESHOLD_CM) {
        Serial.println("OBSTACLE_DETECTED");
        currentState = STATE_AVOIDING;
        return;
    }

    // Проверка краев полигона
    int leftEdge = readIRSensorLeft();
    int rightEdge = readIRSensorRight();

    if (leftEdge > EDGE_THRESHOLD || rightEdge > EDGE_THRESHOLD) {
        Serial.println("EDGE_DETECTED");

        // Экстренная остановка
        stopMotors();

        // Отъезд от края
        moveBackward(100, 100);
        delay(300);

        // Поворот от края
        if (leftEdge > EDGE_THRESHOLD) {
            turnRight(150);
        }
        else {
            turnLeft(150);
        }
        delay(200);

        // Возврат в состояние движения
        stopMotors();
    }
}

void avoidObstacle() {
    Serial.println("Объезд препятствия...");

    // Простой алгоритм объезда
    stopMotors();
    delay(200);

    // Отъезд назад
    moveBackward(150, 150);
    delay(400);

    // Случайный поворот
    if (random(2) == 0) {
        turnLeft(150);
        Serial.println("Поворот налево");
    }
    else {
        turnRight(150);
        Serial.println("Поворот направо");
    }

    delay(500);
    stopMotors();
    delay(200);

    Serial.println("AVOIDANCE_COMPLETE");
}

void patrolMode() {
    static unsigned long lastTurnTime = 0;
    static int patrolDirection = 1;

    // Движение вперед
    moveForward(100, 100);

    // Каждые 3 секунды проверяем, не нужно ли изменить направление
    if (millis() - lastTurnTime > 3000) {
        // Небольшой случайный поворот для поиска
        if (random(3) == 0) {
            turnLeft(80);
            delay(300);
        }
        else if (random(3) == 1) {
            turnRight(80);
            delay(300);
        }
        lastTurnTime = millis();
    }
}

// ---------- КОММУНИКАЦИЯ ----------
void processSerialCommand(String command) {
    Serial.print("Команда получена: ");
    Serial.println(command);

    // Разбор команды
    if (command.startsWith("MOVE_TO")) {
        // Формат: MOVE_TO X Y
        int firstSpace = command.indexOf(' ');
        int secondSpace = command.indexOf(' ', firstSpace + 1);

        if (firstSpace > 0 && secondSpace > 0) {
            targetX = command.substring(firstSpace + 1, secondSpace).toFloat();
            targetY = command.substring(secondSpace + 1).toFloat();

            Serial.print("Движение к точке: X=");
            Serial.print(targetX);
            Serial.print(", Y=");
            Serial.println(targetY);

            currentState = STATE_MOVING;
        }
    }
    else if (command == "STOP") {
        stopMotors();
        currentState = STATE_IDLE;
        Serial.println("STOPPED");
    }
    else if (command == "GRAB") {
        currentState = STATE_GRABBING;
        Serial.println("GRABBING_STARTED");
    }
    else if (command == "RELEASE") {
        currentState = STATE_RELEASING;
        Serial.println("RELEASING_STARTED");
    }
    else if (command == "PATROL") {
        currentState = STATE_PATROL;
        Serial.println("PATROL_STARTED");
    }
    else if (command == "RESET") {
        stopMotors();
        resetManipulator();
        currentX = 0;
        currentY = 0;
        currentAngle = 0;
        currentState = STATE_IDLE;
        Serial.println("SYSTEM_RESET");
    }
    else if (command.startsWith("GET_")) {
        if (command == "GET_ULTRASONIC") {
            float distance = readUltrasonicDistance();
            Serial.print("ULTRASONIC ");
            Serial.println(distance, 1);
        }
        else if (command == "GET_IR") {
            int leftIR = readIRSensorLeft();
            int rightIR = readIRSensorRight();
            Serial.print("IR ");
            Serial.print(leftIR);
            Serial.print(" ");
            Serial.println(rightIR);
        }
        else if (command == "GET_POSITION") {
            Serial.print("POSITION ");
            Serial.print(currentX, 1);
            Serial.print(" ");
            Serial.print(currentY, 1);
            Serial.print(" ");
            Serial.println(currentAngle * 180.0 / PI, 1);
        }
        else if (command == "GET_STATE") {
            Serial.print("STATE ");
            switch (currentState) {
            case STATE_IDLE: Serial.println("IDLE"); break;
            case STATE_MOVING: Serial.println("MOVING"); break;
            case STATE_GRABBING: Serial.println("GRABBING"); break;
            case STATE_RELEASING: Serial.println("RELEASING"); break;
            case STATE_AVOIDING: Serial.println("AVOIDING"); break;
            case STATE_PATROL: Serial.println("PATROL"); break;
            }
        }
    }
    else if (command == "HELP") {
        Serial.println("=== ДОСТУПНЫЕ КОМАНДЫ ===");
        Serial.println("MOVE_TO X Y - движение к координатам");
        Serial.println("STOP - остановка");
        Serial.println("GRAB - захват объекта");
        Serial.println("RELEASE - сброс объекта");
        Serial.println("PATROL - режим патрулирования");
        Serial.println("RESET - сброс системы");
        Serial.println("GET_ULTRASONIC - расстояние спереди");
        Serial.println("GET_IR - показания ИК-датчиков");
        Serial.println("GET_POSITION - текущая позиция");
        Serial.println("GET_STATE - состояние робота");
        Serial.println("HELP - эта справка");
    }
    else {
        Serial.print("НЕИЗВЕСТНАЯ КОМАНДА: ");
        Serial.println(command);
    }
}

void sendStatus() {
    // Отправка текущего статуса
    Serial.print("STATUS ");
    Serial.print(currentX, 1);
    Serial.print(",");
    Serial.print(currentY, 1);
    Serial.print(",");
    Serial.print(currentAngle * 180.0 / PI, 1);
    Serial.print(",");

    switch (currentState) {
    case STATE_IDLE: Serial.print("IDLE"); break;
    case STATE_MOVING: Serial.print("MOVING"); break;
    case STATE_GRABBING: Serial.print("GRABBING"); break;
    case STATE_RELEASING: Serial.print("RELEASING"); break;
    case STATE_AVOIDING: Serial.print("AVOIDING"); break;
    case STATE_PATROL: Serial.print("PATROL"); break;
    }

    Serial.print(",");
    Serial.print(readUltrasonicDistance(), 1);
    Serial.print(",");
    Serial.print(readIRSensorLeft());
    Serial.print(",");
    Serial.println(readIRSensorRight());
}

// ---------- ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ----------
void countLeftEncoder() {
    leftEncoderTicks++;
}

void countRightEncoder() {
    rightEncoderTicks++;
}

// ============================================
// КОНЕЦ КОДА
// ============================================

