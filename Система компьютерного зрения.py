import cv2
import numpy as np

class VisionSystem:
    def init(self, camera_id=0):
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Цветовые диапазоны для объектов (HSV)
        self.color_ranges = {
            'RED': ([0, 100, 100], [10, 255, 255]),
            'BLUE': ([100, 100, 100], [130, 255, 255]),
            'GREEN': ([40, 100, 100], [80, 255, 255]),
            'WHITE': ([0, 0, 200], [180, 50, 255])
        }
    
    def capture_frame(self):
        """Захват кадра с камеры"""
        ret, frame = self.camera.read()
        if not ret:
            raise RuntimeError("Не удалось получить кадр с камеры")
        return frame
    
    def detect_objects(self, frame):
        """Обнаружение объектов по цвету"""
        objects = []
        
        # Конвертируем в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        for color_name, (lower, upper) in self.color_ranges.items():
            # Создаем маску
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Морфологические операции для улучшения маски
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Находим контуры
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # минимальный размер
                    # Вычисляем центр объекта
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        objects.append({
                            'color': color_name,
                            'position': (cx, cy),
                            'contour': contour
                        })
        
        return objects
    
    def detect_hatches(self, frame):
        """Обнаружение люков по цветному периметру"""
        # Аналогично detect_objects, но для прямоугольников
        pass
    
    def cleanup(self):
        """Освобождение ресурсов камеры"""
        if self.camera.isOpened():
            self.camera.release()
