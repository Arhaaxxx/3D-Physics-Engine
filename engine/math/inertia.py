from engine.math.matrix3 import Matrix3

def capsule_inertia_tensor(mass, radius, height):

    r2 = radius * radius
    h2 = height * height

    ix = (1/12) * mass * (3*r2 + h2)
    iy = 0.5 * mass * r2
    iz = ix

    return Matrix3([
        [ix, 0, 0],
        [0, iy, 0],
        [0, 0, iz]
    ])

def inverse_tensor(tensor):

    from engine.math.matrix3 import Matrix3

    return Matrix3([
        [
            1.0 / tensor.m[0][0] if tensor.m[0][0] != 0 else 0,
            0,
            0
        ],
        [
            0,
            1.0 / tensor.m[1][1] if tensor.m[1][1] != 0 else 0,
            0
        ],
        [
            0,
            0,
            1.0 / tensor.m[2][2] if tensor.m[2][2] != 0 else 0
        ]
    ])