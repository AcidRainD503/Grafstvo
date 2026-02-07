import time
import serial
import cv2
from vision import VisionSystem
from navigation import NavigationSystem
from communication import ArduinoCommunicator

class RobotController:
    def init(self):
        # Настройки подключения к Arduino
        self.arduino = ArduinoCommunicator(port='/dev/ttyACM0', baudrate=115200)
        self.vision = VisionSystem()
        self.navigation = NavigationSystem()
        
        # Состояние робота
        self.objects_collected = []
        self.current_target = None
        self.state = "SEARCHING"
        
        # Карта полигона (пример)
        self.map_data = {
            'hatches': {
                'RED': (100, 100),
                'BLUE': (400, 100),
                'GREEN': (100, 400),
                'BLACK': (400, 400)
            },
            'start': (250, 250)
        }
    
    def run(self):
        """Основной цикл работы робота"""
        try:
            print("Запуск робота...")
            self.arduino.connect()
            
            # Главный цикл
            while True:
                if self.state == "SEARCHING":
                    self.search_objects()
                elif self.state == "APPROACHING":
                    self.approach_object()
                elif self.state == "GRABBING":
                    self.grab_object()
                elif self.state == "DELIVERING":
                    self.deliver_object()
                elif self.state == "COMPLETED":
                    print("Задание выполнено!")
                    break
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nОстановка робота...")
        finally:
            self.cleanup()
    
    def search_objects(self):
        """Поиск объектов на полигоне"""
        print("Поиск объектов...")
        
        # Получаем кадр с камеры
        frame = self.vision.capture_frame()
        
        # Находим объекты
        objects = self.vision.detect_objects(frame)
        
        if objects:
            # Выбираем ближайший объект
            target = self.navigation.select_nearest_object(objects, self.current_position)
            self.current_target = target
            
            print(f"Найден объект: {target['color']} в позиции {target['position']}")
            
            # Отправляем команду на движение к объекту
            self.arduino.send_command(f"MOVE_TO {target['position'][0]} {target['position'][1]}")
            self.state = "APPROACHING"
        else:
            # Если объектов нет, продолжаем поиск
            self.arduino.send_command("PATROL")
    
    def approach_object(self):
        """Подход к объекту"""
        # Проверяем расстояние до объекта через датчики
        distance = self.arduino.get_sensor_data('ultrasonic')
        
        if distance < 10:  # 10 см до объекта
            print("Объект достигнут")
            self.state = "GRABBING"
            self.arduino.send_command("STOP")
    
    def grab_object(self):
        """Захват объекта"""
        print("Захват объекта...")
        
        # Команда Arduino на захват
        self.arduino.send_command("GRAB")
        
        # Ждем подтверждения захвата
        response = self.arduino.wait_response("GRAB_SUCCESS", timeout=5)
        
        if response:
            print("Объект захвачен")
            self.objects_collected.append(self.current_target)
            
            # Определяем целевой люк
            target_color = self.current_target['color']
            if target_color == 'WHITE':
                # Белый объект можно сбросить в любой люк
                target_hatch = 'RED'  # пример
            else:
                target_hatch = target_color
            
            hatch_pos = self.map_data['hatches'][target_hatch]
# Двигаемся к люку
            self.arduino.send_command(f"MOVE_TO {hatch_pos[0]} {hatch_pos[1]}")
            self.state = "DELIVERING"
        else:
            print("Ошибка захвата")
            self.state = "SEARCHING"
    
    def deliver_object(self):
        """Доставка объекта к люку"""
        # Проверяем позицию
        current_pos = self.arduino.get_position()
        target_pos = self.map_data['hatches'][self.current_target['color']]
        
        if self.navigation.is_close(current_pos, target_pos, threshold=15):
            print("Люк достигнут, сброс объекта...")
            
            # Сброс объекта
            self.arduino.send_command("RELEASE")
            
            # Ждем подтверждения
            if self.arduino.wait_response("RELEASE_SUCCESS", timeout=3):
                print("Объект сброшен")
                
                # Проверяем, все ли объекты собраны
                if len(self.objects_collected) >= 8:  # всего 8 объектов
                    self.state = "COMPLETED"
                else:
                    self.state = "SEARCHING"
            else:
                print("Ошибка сброса")
                self.state = "SEARCHING"
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.arduino.send_command("STOP")
        self.arduino.disconnect()
        self.vision.cleanup()
        cv2.destroyAllWindows()

if name == "main":
    robot = RobotController()
    robot.run()
