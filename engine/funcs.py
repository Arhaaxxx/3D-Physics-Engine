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
    return [w/mag, x/mag, y/mag, z/mag]

def quat_to_axis(q):
    w, x, y, z = q

    return Vector3(
        2 * (x * z + w * y),
        2 * (y * z + w * x),
        1 - 2 * (x * x + y * y)
    )