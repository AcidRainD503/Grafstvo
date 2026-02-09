// ============================================
// РОБОТ "БЕЗОПАСНЫЙ МАРШРУТ"
// КОД ДЛЯ RASPBERRY PI (ОСНОВНОЙ МОЗГ)
// Версия 1.0 - Один файл
// Язык: C++17
// ============================================

// === БИБЛИОТЕКИ ===
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <serial/serial.h>

// === КОНСТАНТЫ И НАСТРОЙКИ ===
// Конфигурация
#define ARDUINO_PORT "/dev/ttyACM0"
#define ARDUINO_BAUD 115200
#define CAMERA_ID 0
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define FRAME_RATE 30

// Цветовые диапазоны для объектов (HSV)
struct ColorRange {
    std::string name;
    cv::Scalar lower;
    cv::Scalar upper;
    cv::Scalar displayColor;
};

// Карта полигона
struct MapData {
    cv::Point startPoint = cv::Point(320, 240);  // Центр
    std::map<std::string, cv::Point> hatches = {
        {"RED", cv::Point(100, 100)},
        {"BLUE", cv::Point(540, 100)},
        {"GREEN", cv::Point(100, 380)},
        {"BLACK", cv::Point(540, 380)}
    };
};

// Объект для перемещения
struct DetectedObject {
    std::string color;
    cv::Point position;
    cv::Rect boundingBox;
    double area;
    bool collected = false;
};

// === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
std::atomic<bool> g_running(true);
std::atomic<bool> g_paused(false);

// === КЛАССЫ ===

// 1. КЛАСС ДЛЯ РАБОТЫ С ARDUINO
class ArduinoCommunicator {
private:
    std::unique_ptr<serial::Serial> serialPort;
    std::thread readThread;
    std::atomic<bool> running{ false };
    std::vector<std::string> responseQueue;
    std::mutex queueMutex;

public:
    ArduinoCommunicator() = default;

    bool connect(const std::string& port = ARDUINO_PORT, int baudrate = ARDUINO_BAUD) {
        try {
            serialPort = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(1000));

            if (!serialPort->isOpen()) {
                std::cerr << "Не удалось открыть порт " << port << std::endl;
                return false;
            }

            // Даем Arduino время на инициализацию
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Очищаем буфер
            serialPort->flush();

            // Запускаем поток чтения
            running = true;
            readThread = std::thread(&ArduinoCommunicator::readLoop, this);

            std::cout << "Подключено к Arduino на порту " << port << std::endl;
            return true;

        }
        catch (const std::exception& e) {
            std::cerr << "Ошибка подключения к Arduino: " << e.what() << std::endl;
            return false;
        }
    }

    void disconnect() {
        running = false;
        if (readThread.joinable()) {
            readThread.join();
        }
        if (serialPort && serialPort->isOpen()) {
            sendCommand("STOP");
            serialPort->close();
        }
        std::cout << "Отключено от Arduino" << std::endl;
    }

    void sendCommand(const std::string& command) {
        if (serialPort && serialPort->isOpen()) {
            try {
                std::string fullCommand = command + "\n";
                size_t written = serialPort->write(fullCommand);

                if (written == fullCommand.length()) {
                    std::cout << "-> Arduino: " << command << std::endl;
                }
                else {
                    std::cerr << "Ошибка отправки команды: " << command << std::endl;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Исключение при отправке команды: " << e.what() << std::endl;
            }
        }
        else {
            std::cerr << "Порт не открыт для отправки команды: " << command << std::endl;
        }
    }

    std::string waitForResponse(const std::string& expectedPrefix, int timeoutMs = 5000) {
        auto startTime = std::chrono::steady_clock::now();

        while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime).count() < timeoutMs) {

            std::lock_guard<std::mutex> lock(queueMutex);
            for (auto it = responseQueue.begin(); it != responseQueue.end(); ++it) {
                if (it->find(expectedPrefix) != std::string::npos) {
                    std::string response = *it;
                    responseQueue.erase(it);
                    return response;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return "";
    }

    float getUltrasonicDistance() {
        sendCommand("GET_ULTRASONIC");
        std::string response = waitForResponse("ULTRASONIC", 2000);

        if (!response.empty()) {
            try {
                size_t spacePos = response.find(' ');
                if (spacePos != std::string::npos) {
                    return std::stof(response.substr(spacePos + 1));
                }
            }
            catch (...) {
                // Игнорируем ошибки парсинга
            }
        }

        return -1.0f; // Ошибка
    }

    std::pair<float, float> getPosition() {
        sendCommand("GET_POSITION");
        std::string response = waitForResponse("POSITION", 2000);

        if (!response.empty()) {
            try {
                std::istringstream iss(response);
                std::string token;
                std::vector<std::string> tokens;

                while (std::getline(iss, token, ' ')) {
                    tokens.push_back(token);
                }

                if (tokens.size() >= 3) {
                    float x = std::stof(tokens[1]);
                    float y = std::stof(tokens[2]);
                    return { x, y };
                }
            }
            catch (...) {
                // Игнорируем ошибки парсинга
            }
        }

        return { 0.0f, 0.0f }; // По умолчанию
    }

private:
    void readLoop() {
        while (running) {
            try {
                if (serialPort && serialPort->isOpen() && serialPort->available() > 0) {
                    std::string line = serialPort->readline();
                    if (!line.empty()) {
                        // Убираем символы новой строки
                        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());

                        if (!line.empty()) {
                            std::lock_guard<std::mutex> lock(queueMutex);
                            responseQueue.push_back(line);
                            std::cout << "<- Arduino: " << line << std::endl;
                        }
                    }
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Ошибка чтения из Arduino: " << e.what() << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

// 2. КЛАСС СИСТЕМЫ КОМПЬЮТЕРНОГО ЗРЕНИЯ
class VisionSystem {
private:
    cv::VideoCapture camera;
    std::vector<ColorRange> colorRanges;
    cv::Mat currentFrame;
    std::mutex frameMutex;

public:
    VisionSystem() {
        // Определяем цветовые диапазоны (HSV)
        colorRanges = {
            {"RED", cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), cv::Scalar(0, 0, 255)},
            {"RED2", cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), cv::Scalar(0, 0, 255)}, // Красный имеет два диапазона в HSV
            {"BLUE", cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), cv::Scalar(255, 0, 0)},
            {"GREEN", cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), cv::Scalar(0, 255, 0)},
            {"WHITE", cv::Scalar(0, 0, 200), cv::Scalar(180, 50, 255), cv::Scalar(255, 255, 255)}
        };
    }

    bool initialize(int cameraId = CAMERA_ID) {
        try {
            camera.open(cameraId);
            if (!camera.isOpened()) {
                std::cerr << "Не удалось открыть камеру с ID: " << cameraId << std::endl;
                return false;
            }

            camera.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
            camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
            camera.set(cv::CAP_PROP_FPS, FRAME_RATE);

            std::cout << "Камера инициализирована: "
                << CAMERA_WIDTH << "x" << CAMERA_HEIGHT
                << " @ " << FRAME_RATE << " FPS" << std::endl;
            return true;

        }
        catch (const std::exception& e) {
            std::cerr << "Ошибка инициализации камеры: " << e.what() << std::endl;
            return false;
        }
    }

    cv::Mat captureFrame() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (camera.isOpened()) {
            camera >> currentFrame;
            if (!currentFrame.empty()) {
                return currentFrame.clone();
            }
        }
        return cv::Mat(); // Пустой кадр
    }

    std::vector<DetectedObject> detectObjects(const cv::Mat& frame) {
        std::vector<DetectedObject> objects;

        if (frame.empty()) {
            return objects;
        }

        // Конвертируем в HSV
        cv::Mat hsvFrame;
        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

        // Обрабатываем каждый цвет
        for (const auto& colorRange : colorRanges) {
            cv::Mat mask;

            // Для красного цвета нужно объединить два диапазона
            if (colorRange.name == "RED") {
                cv::Mat mask1, mask2;
                cv::inRange(hsvFrame, colorRange.lower, colorRange.upper, mask1);

                // Ищем второй диапазон красного
                auto red2 = std::find_if(colorRanges.begin(), colorRanges.end(),
                    [](const ColorRange& cr) { return cr.name == "RED2"; });

                if (red2 != colorRanges.end()) {
                    cv::inRange(hsvFrame, red2->lower, red2->upper, mask2);
                    mask = mask1 | mask2;
                }
                else {
                    mask = mask1;
                }
            }
            else {
                cv::inRange(hsvFrame, colorRange.lower, colorRange.upper, mask);
            }

            // Морфологические операции для очистки маски
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

            // Находим контуры
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > 500) { // Минимальная площадь
                    cv::Rect boundingBox = cv::boundingRect(contour);
                    cv::Point center(boundingBox.x + boundingBox.width / 2,
                        boundingBox.y + boundingBox.height / 2);

                    DetectedObject obj;
                    obj.color = colorRange.name;
                    obj.position = center;
                    obj.boundingBox = boundingBox;
                    obj.area = area;

                    objects.push_back(obj);
                }
            }
        }

        return objects;
    }

    cv::Mat drawDetections(const cv::Mat& frame, const std::vector<DetectedObject>& objects) {
        cv::Mat displayFrame = frame.clone();

        for (const auto& obj : objects) {
            // Определяем цвет для отрисовки
            cv::Scalar color;
            if (obj.color == "RED" || obj.color == "RED2") color = cv::Scalar(0, 0, 255);
            else if (obj.color == "BLUE") color = cv::Scalar(255, 0, 0);
            else if (obj.color == "GREEN") color = cv::Scalar(0, 255, 0);
            else color = cv::Scalar(255, 255, 255); // WHITE

            // Рисуем прямоугольник
            cv::rectangle(displayFrame, obj.boundingBox, color, 2);

            // Рисуем центр
            cv::circle(displayFrame, obj.position, 5, color, -1);

            // Подпись
            std::string label = obj.color + " (" + std::to_string((int)obj.area) + ")";
            cv::putText(displayFrame, label,
                cv::Point(obj.boundingBox.x, obj.boundingBox.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }

        return displayFrame;
    }

    void cleanup() {
        if (camera.isOpened()) {
            camera.release();
        }
        cv::destroyAllWindows();
        std::cout << "Система зрения остановлена" << std::endl;
    }
};

// 3. КЛАСС НАВИГАЦИИ
class NavigationSystem {
private:
    MapData map;

public:
    NavigationSystem() = default;

    DetectedObject selectTargetObject(const std::vector<DetectedObject>& objects,
        const std::pair<float, float>& currentPosition) {
        if (objects.empty()) {
            return DetectedObject();
        }

        DetectedObject nearest;
        float minDistance = std::numeric_limits<float>::max();

        cv::Point currentPos(currentPosition.first, currentPosition.second);

        for (const auto& obj : objects) {
            if (obj.collected) continue;

            float distance = std::sqrt(
                std::pow(obj.position.x - currentPos.x, 2) +
                std::pow(obj.position.y - currentPos.y, 2)
            );

            if (distance < minDistance) {
                minDistance = distance;
                nearest = obj;
            }
        }

        return nearest;
    }

    std::string getTargetHatch(const std::string& objectColor) {
        if (objectColor == "WHITE" || objectColor == "WHITE") {
            // Белый объект можно сбросить в любой люк
            return "RED"; // Выбираем красный по умолчанию
        }

        // Для цветных объектов - соответствующий люк
        if (objectColor == "RED" || objectColor == "RED2") return "RED";
        if (objectColor == "BLUE") return "BLUE";
        if (objectColor == "GREEN") return "GREEN";

        return "BLACK"; // Любой другой - в черный (со штрафом)
    }

    cv::Point getHatchPosition(const std::string& hatchColor) {
        auto it = map.hatches.find(hatchColor);
        if (it != map.hatches.end()) {
            return it->second;
        }
        return map.hatches["BLACK"]; // По умолчанию черный люк
    }

    bool isCloseToTarget(const std::pair<float, float>& currentPos,
        const cv::Point& targetPos,
        float threshold = 20.0f) {
        float dx = targetPos.x - currentPos.first;
        float dy = targetPos.y - currentPos.second;
        return std::sqrt(dx * dx + dy * dy) < threshold;
    }

    float calculateDistance(const std::pair<float, float>& pos1, const cv::Point& pos2) {
        float dx = pos2.x - pos1.first;
        float dy = pos2.y - pos1.second;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// 4. ГЛАВНЫЙ КОНТРОЛЛЕР РОБОТА
class RobotController {
private:
    enum class State {
        INITIALIZING,
        SEARCHING,
        APPROACHING,
        GRABBING,
        DELIVERING,
        COMPLETED,
        ERROR
    };

    State currentState = State::INITIALIZING;

    // Компоненты
    std::unique_ptr<ArduinoCommunicator> arduino;
    std::unique_ptr<VisionSystem> vision;
    std::unique_ptr<NavigationSystem> navigation;

    // Данные
    std::vector<DetectedObject> detectedObjects;
    DetectedObject currentTarget;
    std::string targetHatch;
    int objectsCollected = 0;
    const int totalObjects = 8; // Всего объектов по заданию

    // Время
    std::chrono::steady_clock::time_point startTime;
    const int maxTimeSeconds = 600; // 10 минут

public:
    RobotController()
        : arduino(std::make_unique<ArduinoCommunicator>()),
        vision(std::make_unique<VisionSystem>()),
        navigation(std::make_unique<NavigationSystem>()) {
    }

    bool initialize() {
        std::cout << "Инициализация робота..." << std::endl;

        // Инициализация камеры
        if (!vision->initialize()) {
            std::cerr << "Ошибка инициализации камеры" << std::endl;
            return false;
        }

        // Подключение к Arduino
        if (!arduino->connect()) {
            std::cerr << "Ошибка подключения к Arduino" << std::endl;
            return false;
        }

        // Даем время на установку связи
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Сброс позиции
        arduino->sendCommand("RESET");

        currentState = State::SEARCHING;
        startTime = std::chrono::steady_clock::now();

        std::cout << "Робот инициализирован успешно" << std::endl;
        return true;
    }

    void runOneCycle() {
        if (g_paused) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }

        // Проверка времени
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - startTime).count();

        if (elapsed > maxTimeSeconds) {
            std::cout << "Время вышло (" << elapsed << " сек)" << std::endl;
            arduino->sendCommand("STOP");
            currentState = State::COMPLETED;
            return;
        }

        switch (currentState) {
        case State::SEARCHING:
            searchObjects();
            break;

        case State::APPROACHING:
            approachObject();
            break;

        case State::GRABBING:
            grabObject();
            break;

        case State::DELIVERING:
            deliverObject();
            break;

        case State::COMPLETED:
            std::cout << "Задание выполнено!" << std::endl;
            arduino->sendCommand("STOP");
            g_running = false;
            break;

        case State::ERROR:
            std::cerr << "Робот в состоянии ошибки" << std::endl;
            arduino->sendCommand("STOP");
            break;
        default:
            break;
        }
    }

    void searchObjects() {
        std::cout << "Поиск объектов..." << std::endl;

        // Захватываем кадр
        cv::Mat frame = vision->captureFrame();
        if (frame.empty()) {
            std::cerr << "Пустой кадр с камеры" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return;
        }

        // Детектируем объекты
        detectedObjects = vision->detectObjects(frame);

        // Отображаем результат (для отладки)
        cv::Mat displayFrame = vision->drawDetections(frame, detectedObjects);
        cv::imshow("Обнаружение объектов", displayFrame);
        cv::waitKey(1); // Обновление окна

        if (!detectedObjects.empty()) {
            // Выбираем ближайший объект
            auto currentPos = arduino->getPosition();