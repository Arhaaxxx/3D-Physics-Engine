import json
import time
import math

from engine.vector import Vector3
from engine.funcs import *
from engine.collision.collision_utils import *
from engine.capsule import *
from engine.bodies.body import Body
from engine.quaternion import quat_to_axis
from engine.collision.capsule_ground import solve_capsule_ground
from engine.collision.sphere_capsule import solve_sphere_capsule
from engine.integration.integrate_angular import integrate_angular
from engine.collision.sphere_sphere import object_collisions
from engine.integration.integrate_linear import *
from engine.solver.sleep import apply_sleep

import numpy as np


class PhysicsEngine:
    def __init__(self):
        self.bodies = []
        self.capsules = []
        self.accumulator = 0.0
        self.fixed_dt = 0.01

    def add_bodies(self, body):
        self.bodies.append(body)

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

        self.pos = np.zeros((0, 3))
        self.vel = np.zeros((0, 3))
        self.radius = np.zeros(0)
        self.mass = np.zeros(0)

        # ---------- SPHERES ----------
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

        # ---------- SLEEP SYSTEM ----------
        self.sleep_timer = np.zeros(n)
        self.sleeping = np.zeros(n, dtype=bool)

        # ---------- CAPSULES ----------
        # ❗ IMPORTANT: Capsules are now rigid bodies → DO NOT use arrays
        # We just ensure required properties exist

        for cap in self.capsules:

            # safety initialization (in case missing)
            if not hasattr(cap, "velocity"):
                cap.velocity = Vector3(0, 0, 0)

            if not hasattr(cap, "angular_velocity"):
                cap.angular_velocity = Vector3(0, 0, 0)

            if not hasattr(cap, "orientation"):
                cap.orientation = Vector3(0, 0, 1)

            if not hasattr(cap, "inv_mass"):
                cap.inv_mass = 1.0 / cap.mass if cap.mass > 0 else 0

            if not hasattr(cap, "inv_inertia"):
                # simple inertia approximation
                I = (1/12) * cap.mass * (3*cap.radius*cap.radius + (2*cap.half_length)**2)
                cap.inv_inertia = 1.0 / I if I > 0 else 0
                

        # ---------- PREV (for interpolation) ----------
        self.prev_pos = self.pos.copy()

    def step(self, dt):

        substeps = 3
        h = dt / substeps

        for _ in range(substeps):

            integrate_spheres(self, h)
            integrate_capsules(self, h)

            #syncing
            for i, b in enumerate(self.bodies):
                b.position.x, b.position.y, b.position.z = self.pos[i]
                b.velocity.x, b.velocity.y, b.velocity.z = self.vel[i]

            iterations = 5

            for _ in range(iterations):

                # ---- SPHERE-SPHERE ----
                grid = self.build_grid(self.bodies, cell_size=1.0)

                for cell in grid.values():
                    for i in range(len(cell)):
                        for j in range(i + 1, len(cell)):
                            object_collisions(cell[i], cell[j])

                # ---- BOX (SPHERES) ----
                # for body in self.bodies:
                #     box_collision(body)

                for body in self.bodies:
                    ground_collision(body)

                for cap in self.capsules:
                    solve_capsule_ground(cap)

                # ---------- SPHERE-CAPSULE ----------
                for body in self.bodies:

                    for cap in self.capsules:

                        solve_sphere_capsule(cap, body)

                #sync
                for i, b in enumerate(self.bodies):
                    self.pos[i] = [b.position.x, b.position.y, b.position.z]
                    self.vel[i] = [b.velocity.x, b.velocity.y, b.velocity.z]

            for cap in self.capsules:
                apply_sleep(cap)

            # ---------- CAPSULE ROTATION ----------
            print("ANG VEL:", cap.angular_velocity.magnitude())
            for cap in self.capsules:
                integrate_angular(cap, h)
        # ---------- FINAL ----------
        self.curr_pos = self.pos.copy()