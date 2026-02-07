import math

class NavigationSystem:
    def init(self):
        self.path = []
    
    def select_nearest_object(self, objects, current_position):
        """Выбор ближайшего объекта"""
        if not objects:
            return None
        
        nearest = None
        min_distance = float('inf')
        
        for obj in objects:
            distance = self.calculate_distance(current_position, obj['position'])
            if distance < min_distance:
                min_distance = distance
                nearest = obj
        
        return nearest
    
    def calculate_distance(self, pos1, pos2):
        """Вычисление расстояния между двумя точками"""
        return math.sqrt((pos2[0] - pos1[0])2 + (pos2[1] - pos1[1])2)
    
    def is_close(self, pos1, pos2, threshold=20):
        """Проверка близости двух точек"""
        return self.calculate_distance(pos1, pos2) < threshold
    
    def plan_path(self, start, end, obstacles=[]):
        """Планирование пути с учетом препятствий (люков)"""
        # Упрощенный алгоритм планирования
        # В реальном проекте можно использовать A*, RRT и т.д.
        return [start, end]
