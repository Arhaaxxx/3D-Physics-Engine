import math
from engine.vector import *
import numpy as np

def dot(a, b):
    return a.x*b.x + a.y*b.y + a.z*b.z

#forces definition
def compute_drag(body, k=0.01):
    return body.velocity * -k
    
def gravity(body):
    return Vector3(0, 0, -9.81* 2 * body.mass)

def wind_field(postition):
    wx = math.sin(postition.y * 0.2) * 2
    wy = math.cos(postition.x * 0.2) * 2
    wz = 0
    return Vector3(wx, wy, wz)
    
def compute_wind_drag(body, k=0.02):
    wind = wind_field(body.position)
    relative_velocity = body.velocity - wind
    return relative_velocity * -k

def ground_friction(body, mu=0.6):
    if body.position.z <= body.radius:

        v = Vector3(body.velocity.x, body.velocity.y, 0)
        speed = v.magnitude()

        if speed > 0:
            friction = v.normalize() * -mu * body.mass
            return friction

    return Vector3(0,0,0)

def integrate_rotation(q, omega, dt):

    w, x, y, z = q

    ox, oy, oz = omega.x, omega.y, omega.z

    dq = [
        0,
        ox,
        oy,
        oz
    ]

    qw = -x * ox + y * oy - z * oz
    qx = w * ox + y * oz - z * oy
    qy = w * oy + z * ox - x * oz
    qz = w * oz + x * oy - y * ox

    w += 0.5 * qw * dt
    x += 0.5 * qx * dt
    y += 0.5 * qy * dt
    z += 0.5 * qz * dt
    
    mag = math.sqrt(w * w + x * x + y * y + z * z)

    if mag < 1e-8:
        return [1, 0, 0, 0]

    return [w/mag, x/mag, y/mag, z/mag]

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