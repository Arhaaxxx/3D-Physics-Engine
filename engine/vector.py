import math

class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Vector3(self.x + other.x,
                       self.y + other.y,
                       self.z + other.z)
    
    def __sub__(self, other):
        return Vector3(self.x - other.x,
                       self.y - other.y,
                       self.z - other.z)
    
    def __mul__(self, scalar):
        return Vector3(self.x * scalar,
                       self.y * scalar,
                       self.z * scalar)
    
    def __truediv__(self, scalar):
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector3()
        return self*(1/mag)
    
    def to_list(self):
        return [self.x, self.y, self.z]