import math
class points:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.distance = 0.0
        self.angle = 0.0

    def __str__(self):
        return f"({self.x}, {self.y}, {self.z})"

    def __repr__(self):
        return f"points({self.x}, {self.y}, {self.z})"
    def __add__(self, other):
        return points(self.x + other.x, self.y + other.y, self.z + other.z)
    def __sub__(self, other):
        return points(self.x - other.x, self.y - other.y, self.z - other.z)
    def __mul__(self, other):
        return points(self.x * other, self.y * other, self.z * other)
    def __truediv__(self, other):
        return points(self.x / other, self.y / other, self.z / other)
    def __eq__(self, value):
        return self.x == value.x and self.y == value.y and self.z == value.z
    def __ne__(self, value):    
        return self.x != value.x or self.y != value.y or self.z != value.z
    def __lt__(self, value):
        return self.x < value.x and self.y < value.y and self.z < value.z
    def __le__(self, value):
        return self.x <= value.x and self.y <= value.y and self.z <= value.z
    def __gt__(self, value):    
        return self.x > value.x and self.y > value.y and self.z > value.z
    def __ge__(self, value):
        return self.x >= value.x and self.y >= value.y and self.z >= value.z
    def distance(self, other):
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5
    def angle(self, other):
        return math.atan2(other.y - self.y, other.x - self.x) * 180 / math.pi