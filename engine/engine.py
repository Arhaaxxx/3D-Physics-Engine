import json
import time
import math

from engine.vector import Vector3
from engine.funcs import *
from engine.collision import *
from engine.capsule import *
from engine.body import Body

import numpy as np


# proper engine part
class PhysicsEngine:
    def __init__(self):
        self.bodies = []
        self.capsules = []   # ✅ added (was missing)
        self.accumulator = 0.0
        self.fixed_dt = 0.01   # 100 Hz physics

    def add_bodies(self, body):
        self.bodies.append(body)

    # ✅ FIX: add self
    def build_grid(self, bodies, cell_size):
        grid = {}

        for b in bodies:
            key = (
                int(b.position.x // cell_size),
                int(b.position.y // cell_size),
                int(b.position.z // cell_size)
            )

            if key not in grid:
                grid[key] = []
            grid[key].append(b)

        return grid

    def build_arrays(self):
        n = len(self.bodies)

        self.pos = np.zeros((n, 3))
        self.vel = np.zeros((n, 3))
        self.radius = np.zeros(n)
        self.mass = np.zeros(n)

        for i, b in enumerate(self.bodies):
            self.pos[i] = [b.position.x, b.position.y, b.position.z]
            self.vel[i] = [b.velocity.x, b.velocity.y, b.velocity.z]
            self.radius[i] = b.radius
            self.mass[i] = b.mass

        # CAPSULE ARRAYS
        m = len(self.capsules)

        self.cap_pos1 = np.zeros((m, 3))
        self.cap_pos2 = np.zeros((m, 3))
        self.cap_vel1 = np.zeros((m, 3))
        self.cap_vel2 = np.zeros((m, 3))

        for i, cap in enumerate(self.capsules):
            self.cap_pos1[i] = [
                cap.p1.position.x,
                cap.p1.position.y,
                cap.p1.position.z
            ]
            self.cap_pos2[i] = [
                cap.p2.position.x,
                cap.p2.position.y,
                cap.p2.position.z
            ]
            self.cap_vel1[i] = [
                cap.p1.velocity.x,
                cap.p1.velocity.y,
                cap.p1.velocity.z
            ]
            self.cap_vel2[i] = [
                cap.p2.velocity.x,
                cap.p2.velocity.y,
                cap.p2.velocity.z
            ]

        # store previous positions for rendering
        self.prev_pos = self.pos.copy()

        if len(self.capsules) > 0:
            self.prev_cap_pos1 = self.cap_pos1.copy()
            self.prev_cap_pos2 = self.cap_pos2.copy()

    def write_back(self):
        for i, b in enumerate(self.bodies):
            b.position.x, b.position.y, b.position.z = self.pos[i]
            b.velocity.x, b.velocity.y, b.velocity.z = self.vel[i]
        
        for i, cap in enumerate(self.capsules):
            cap.p1.position.x, cap.p1.position.y, cap.p1.position.z = self.cap_pos1[i]
            cap.p2.position.x, cap.p2.position.y, cap.p2.position.z = self.cap_pos2[i]

            cap.p1.velocity.x, cap.p1.velocity.y, cap.p1.velocity.z = self.cap_vel1[i]
            cap.p2.velocity.x, cap.p2.velocity.y, cap.p2.velocity.z = self.cap_vel2[i]

    def ground_collision_array(self, pos, vel, radius=0.5, restitution=0.3):
            for i in range(len(pos)):
                if pos[i][2] < radius:
                    pos[i][2] = radius
                    if vel[i][2] < 0:
                        vel[i][2] *= -restitution

    def step(self, dt):

        self.build_arrays()

        # store previous positions for velocity correction
        for cap in self.capsules:
            cap.p1.prev_position = Vector3(
                cap.p1.position.x,
                cap.p1.position.y,
                cap.p1.position.z
            )
            cap.p2.prev_position = Vector3(
                cap.p2.position.x,
                cap.p2.position.y,
                cap.p2.position.z
            )

        # ---------- ARRAY UPDATE ----------
        self.vel[:, 2] -= 9.81 * dt
        self.vel *= 0.999
        self.pos += self.vel * dt

        # ---------- CAPSULE ARRAY UPDATE ----------
        if len(self.capsules) > 0:
            self.cap_vel1[:, 2] -= 9.81 * dt
            self.cap_vel2[:, 2] -= 9.81 * dt

            self.cap_pos1 += self.cap_vel1 * dt
            self.cap_pos2 += self.cap_vel2 * dt

            # ground for capsule (ARRAY LEVEL)
            self.ground_collision_array(self.cap_pos1, self.cap_vel1)
            self.ground_collision_array(self.cap_pos2, self.cap_vel2)

        # ---------- SYNC ARRAYS → OBJECTS (ONLY ONCE) ----------
        self.write_back()

        # ---------- GROUND ----------
        for body in self.bodies:
            ground_collision(body)

        for cap in self.capsules:
            ground_collision(cap.p1)
            ground_collision(cap.p2)

        # ---------- COLLISIONS ----------

        grid = self.build_grid(self.bodies, cell_size=1.0)

        for cell in grid.values():
            for i in range(len(cell)):
                for j in range(i+1, len(cell)):
                    object_collisions(cell[i], cell[j])

        # capsule-sphere
        for cap in self.capsules:
            for body in self.bodies:

                if body.shape != "sphere":
                    continue

                dx = body.position.x - cap.p1.position.x
                dy = body.position.y - cap.p1.position.y
                dz = body.position.z - cap.p1.position.z

                if dx*dx + dy*dy + dz*dz > 25:
                    continue

                capsule_sphere_response(cap, body)

        # ---------- CAPSULE CONSTRAINT ----------
        for cap in self.capsules:
            solve_capsule_constraint(cap)

        # ---------- FIX CAPSULE VELOCITY (CRITICAL) ----------
        for cap in self.capsules:
            # velocity = (new_position - old_position) / dt

            vx1 = (cap.p1.position.x - cap.p1.prev_position.x) / dt
            vy1 = (cap.p1.position.y - cap.p1.prev_position.y) / dt
            vz1 = (cap.p1.position.z - cap.p1.prev_position.z) / dt

            vx2 = (cap.p2.position.x - cap.p2.prev_position.x) / dt
            vy2 = (cap.p2.position.y - cap.p2.prev_position.y) / dt
            vz2 = (cap.p2.position.z - cap.p2.prev_position.z) / dt

            cap.p1.velocity = Vector3(vx1, vy1, vz1)
            cap.p2.velocity = Vector3(vx2, vy2, vz2)

        # ---------- BOX ----------
        for body in self.bodies:
            box_collision(body)

        self.curr_pos = self.pos.copy()

        if len(self.capsules) > 0:
            self.curr_cap_pos1 = self.cap_pos1.copy()
            self.curr_cap_pos2 = self.cap_pos2.copy()

    # ✅ KEPT EXACTLY AS YOU WROTE
    def export_state(self, alpha=1.0):
        # alpha = interpolation factor (0 → previous, 1 → current)

        bodies_out = []
        for i, b in enumerate(self.bodies):
            interp = self.prev_pos[i] * (1 - alpha) + self.curr_pos[i] * alpha

            bodies_out.append({
                **b.serialize(),
                "position": interp.tolist()
            })

        capsules_out = []
        for i, cap in enumerate(self.capsules):
            p1 = self.prev_cap_pos1[i] * (1 - alpha) + self.curr_cap_pos1[i] * alpha
            p2 = self.prev_cap_pos2[i] * (1 - alpha) + self.curr_cap_pos2[i] * alpha

            capsules_out.append({
                "p1": p1.tolist(),
                "p2": p2.tolist(),
                "radius": cap.radius,
                "height": cap.rest_length
            })

        return {
            "bodies": bodies_out,
            "capsules": capsules_out
        }