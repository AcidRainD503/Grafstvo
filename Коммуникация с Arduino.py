import serial
import time
import threading

class ArduinoCommunicator:
    def init(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.response_queue = []
        self.lock = threading.Lock()
    
    def connect(self):
        """Подключение к Arduino"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Ждем инициализации Arduino
            print(f"Подключено к Arduino на {self.port}")
            
            # Запускаем поток для чтения ответов
            self.running = True
            thread = threading.Thread(target=self._read_thread)
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            print(f"Ошибка подключения к Arduino: {e}")
            raise
    
    def send_command(self, command):
        """Отправка команды на Arduino"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{command}\n".encode('utf-8'))
                print(f"Отправлено: {command}")
            except Exception as e:
                print(f"Ошибка отправки команды: {e}")
    
    def _read_thread(self):
        """Поток для чтения ответов от Arduino"""
        while self.running and self.serial:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        print(f"Получено от Arduino: {line}")
                        
                        with self.lock:
                            self.response_queue.append(line)
            except Exception as e:
                print(f"Ошибка чтения из Arduino: {e}")
                time.sleep(0.1)
    
    def wait_response(self, expected_response, timeout=5):
        """Ожидание определенного ответа"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.lock:
                for response in self.response_queue:
                    if expected_response in response:
                        self.response_queue.remove(response)
                        return response
            
            time.sleep(0.1)
        
        return None
    
    def get_sensor_data(self, sensor_type):
        """Запрос данных с датчиков"""
        self.send_command(f"GET_{sensor_type}")
        response = self.wait_response(sensor_type, timeout=2)
        
        if response:
            # Парсим ответ, например: "ULTRASONIC 25.5"
            parts = response.split()
            if len(parts) == 2:
                try:
                    return float(parts[1])
                except:
                    pass
        
        return None
    
    def get_position(self):
        """Получение текущей позиции от одометрии"""
        self.send_command("GET_POSITION")
        response = self.wait_response("POSITION", timeout=2)
        
        if response:
            # Формат: "POSITION X Y"
            parts = response.split()
            if len(parts) == 3:
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    return (x, y)
                except:
                    pass
        
        return (0, 0)
    
    def disconnect(self):
        """Отключение от Arduino"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Отключено от Arduino")
