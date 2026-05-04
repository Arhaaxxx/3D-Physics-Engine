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

