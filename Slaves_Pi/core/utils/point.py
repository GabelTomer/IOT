import math

class Point:
    __slots__ = ['x', 'y', 'z']

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self) -> str:
        return f"({self.x}, {self.y}, {self.z})"

    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y}, {self.z})"
    
    def __add__(self, other: 'Point') -> 'Point':
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Point') -> 'Point':
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, other: float) -> 'Point':
        return Point(self.x * other, self.y * other, self.z * other)
    
    def __truediv__(self, other: float) -> 'Point':
        return Point(self.x / other, self.y / other, self.z / other)
    
    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Point):
            return NotImplemented
        return self.x == value.x and self.y == value.y and self.z == value.z
    
    def __ne__(self, value: object) -> bool:
        return not self == value
    
    def __lt__(self, value: 'Point') -> bool:
        return self.x < value.x and self.y < value.y and self.z < value.z
    
    def __le__(self, value: 'Point') -> bool:
        return self.x <= value.x and self.y <= value.y and self.z <= value.z
    
    def __gt__(self, value: 'Point') -> bool:
        return self.x > value.x and self.y > value.y and self.z > value.z
    
    def __ge__(self, value: 'Point') -> bool:
        return self.x >= value.x and self.y >= value.y and self.z >= value.z
    
    def distance(self, other: 'Point') -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5
    
    def angle(self, other: 'Point') -> float:
        return math.degrees(math.atan2(other.y - self.y, other.x - self.x))