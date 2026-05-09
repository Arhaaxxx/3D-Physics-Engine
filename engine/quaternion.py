from engine.vector import Vector3

def quat_to_axis(q):

    w, x, y, z = q

    # local up vector
    vx, vy, vz = 0, 1, 0

    # q * v
    ix =  w * vx + y * vz - z * vy
    iy =  w * vy + z * vx - x * vz
    iz =  w * vz + x * vy - y * vx
    iw = -x * vx - y * vy - z * vz

    # result * conj(q)
    rx = ix * w + iw * -x + iy * -z - iz * -y
    ry = iy * w + iw * -y + iz * -x - ix * -z
    rz = iz * w + iw * -z + ix * -y - iy * -x

    axis = Vector3(rx, ry, rz)

    if axis.magnitude() < 1e-6:
        return Vector3(0,0,1)

    return axis.normalize()