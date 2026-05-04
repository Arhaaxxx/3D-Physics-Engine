import json
import math
import time
from engine.engine import PhysicsEngine, Body, Vector3
from engine.capsule import Capsule

engine = PhysicsEngine()

engine.capsules = []

cap = Capsule(
    position=Vector3(0, 0, 20),
    height=6,
    mass=10,
    radius=2
)

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

for x in range(-5, 5):
    for y in range(-5, 5):
        for z in range(1, 6):

            engine.add_bodies(
                Body(
                    mass=0.1,
                    position=Vector3(x*0.8, y*0.8, z*0.8+2),
                    velocity=Vector3(0,0,0),
                    shape="sphere",
                    radius=0.2
                )
            )

#loop
dt = 0.01

while True:
    engine.accumulator += dt

    while engine.accumulator >= engine.fixed_dt:
        engine.step(engine.fixed_dt)
        engine.accumulator -= engine.fixed_dt

    with open("state.json", "w") as f:
        json.dump(engine.export_state(), f)

    time.sleep(dt)