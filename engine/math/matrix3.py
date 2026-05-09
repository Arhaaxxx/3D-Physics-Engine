class Matrix3:

    def __init__(self, rows=None):

        if rows is None:
            self.m = [[0.0, 0.0, 0.0,],
                      [0.0, 0.0, 0.0,],
                      [0.0, 0.0, 0.0]]
            
        else:
            self.m = rows

    @staticmethod
    def identity():
        return Matrix3(
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        )
    
    def __mul__(self, vec):
        x = (
            self.m[0][0] * vec.x +
            self.m[0][1] * vec.y +
            self.m[0][2] * vec.z
        )

        y = (
            self.m[1][0] * vec.x +
            self.m[1][1] * vec.y +
            self.m[1][2] * vec.z
        )

        z = (
            self.m[2][0] * vec.x +
            self.m[2][1] * vec.y +
            self.m[2][2] * vec.z
        )

        from engine.vector import Vector3
        return Vector3(x, y, z)
    
    def transpose(self):

        return Matrix3([
            [self.m[0][0], self.m[1][0], self.m[2][0]],
            [self.m[0][1], self.m[1][1], self.m[2][1]],
            [self.m[0][2], self.m[1][2], self.m[2][2]]
        ])
    
    def matmul(self, other):

        result = Matrix3()

        for i in range(3):
            for j in range(3):

                result.m[i][j] = (
                    self.m[i][0] * other.m[0][j] +
                    self.m[i][1] * other.m[1][j] +
                    self.m[i][2] * other.m[2][j]
                )

        return result