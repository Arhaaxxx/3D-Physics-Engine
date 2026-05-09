import json
import os
import math
import time
import socket
from engine.engine import PhysicsEngine, Body, Vector3
from engine.capsule import RigidCapsule
from engine.rendering.export import export_state

engine = PhysicsEngine()

engine.capsules = []

cap = RigidCapsule(
    position=Vector3(0, 0, 20),
    height=10,
    mass=5,
    radius=1
)

# cap.orientation = Vector3(0,0,1)
cap.angular_velocity = Vector3(0.5,0.2,0)
cap.velocity = Vector3(100,0,0)

engine.capsules.append(cap)

# target = Body(
#     mass=0.01,
#     position=Vector3(0,0.25,5),
#     velocity=Vector3(0,0,0),
#     shape="sphere",
#     radius=0.2
# )

# rocket.angular_velocity = Vector3(0, 0, 1)

# engine.add_bodies(target)

# for x in range(-3, 3):
#     for y in range(-3, 3):
#         for z in range(1, 4):

#             engine.add_bodies(
#                 Body(
#                     mass=0.05,
#                     position=Vector3(x*0.6, y*0.6, z*0.6+2),
#                     velocity=Vector3(0,0,0),
#                     shape="sphere",
#                     radius=0.2
#                 )
#             )

engine.build_arrays()

#loop
dt = 0.01


HOST = '127.0.0.1'
PORT = 65432

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("Waiting for renderer to connect...")
conn, addr = server.accept()
print("Connected:", addr)

while True:
    engine.accumulator += dt

    while engine.accumulator >= engine.fixed_dt:
        engine.step(engine.fixed_dt)
        engine.accumulator -= engine.fixed_dt

    try:
        # ---------- PREPARE DATA ----------
        state = export_state(engine)
        data = json.dumps(state).encode("utf-8")

        # ---------- SEND LENGTH FIRST ----------
        size = len(data)
        conn.sendall(size.to_bytes(4, byteorder="little"))

        # ---------- SEND DATA ----------
        conn.sendall(data)

    except Exception as e:
        print("Connection lost:", e)
        break

    time.sleep(dt)