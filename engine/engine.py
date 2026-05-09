import json
import time
import math

from engine.vector import Vector3
from engine.funcs import *
from engine.collision import *
from engine.capsule import *
from engine.body import Body

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

    def solve_capsules_array(self):
        for i in range(len(self.cap_pos1)):

            p1 = self.cap_pos1[i]
            p2 = self.cap_pos2[i]

            delta = p2 - p1
            dist = np.linalg.norm(delta)

            if dist < 1e-6:
                continue

            direction = delta / dist
            error = dist - self.cap_rest_length[i]

            correction = 0.5 * error * direction

            self.cap_pos1[i] += correction
            self.cap_pos2[i] -= correction

    def step(self, dt):

        substeps = 3
        h = dt / substeps

        for _ in range(substeps):

            # ---------- 1. APPLY FORCES ----------
            # ---------- SPHERE UPDATE ----------
            if len(self.pos) > 0:

                self.vel[:, 2] -= 9.81 * h
                self.vel *= 0.999

                self.pos += self.vel * h

            for cap in self.capsules:
                cap.velocity.z -= 9.81 * h

            # ---------- 2. INTEGRATE ----------
            self.pos += self.vel * h

            for cap in self.capsules:
                cap.position += cap.velocity * h

            # ---------- 3. SYNC ARRAYS → OBJECTS ----------
            for i, b in enumerate(self.bodies):
                b.position.x, b.position.y, b.position.z = self.pos[i]
                b.velocity.x, b.velocity.y, b.velocity.z = self.vel[i]

            # ---------- 4. SOLVER ----------
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

                # ---------- CAPSULE-GROUND (ADD HERE) ----------
                for cap in self.capsules:

                    axis = quat_to_axis(cap.orientation)

                    p1 = cap.position + axis * cap.half_length
                    p2 = cap.position - axis * cap.half_length

                    contact = p1 if p1.z < p2.z else p2

                    if contact.z - cap.radius < 0:
                        normal = Vector3(0, 0, 1)

                        penetration = cap.radius - contact.z

                        # position correction
                        cap.position.z += penetration * 0.8

                        # ---------- CONTACT VECTOR ----------
                        r = contact - cap.position

                        # ---------- CONTACT VELOCITY ----------
                        v_contact = cap.velocity + Vector3(
                            cap.angular_velocity.y * r.z - cap.angular_velocity.z * r.y,
                            cap.angular_velocity.z * r.x - cap.angular_velocity.x * r.z,
                            cap.angular_velocity.x * r.y - cap.angular_velocity.y * r.x
                        )

                        # ---------- NORMAL VELOCITY ----------
                        # use stronger downward component
                        vn = min(v_contact.dot(normal), cap.velocity.z)

                        if vn < 0:

                            restitution = 0.1 if abs(vn) > 0.5 else 0.0

                            # effective rotational mass
                            r_cross_n = Vector3(
                                r.y * normal.z - r.z * normal.y,
                                r.z * normal.x - r.x * normal.z,
                                r.x * normal.y - r.y * normal.x
                            )

                            effective_mass = (
                                cap.inv_mass +
                                (r_cross_n.magnitude() ** 2) * cap.inv_inertia
                            )

                            j = -(1 + restitution) * vn

                            if effective_mass > 1e-6:
                                j /= effective_mass

                            impulse = normal * j

                            # ---------- LINEAR ----------
                            cap.velocity += impulse * cap.inv_mass

                            # horizontal damping on impact
                            cap.velocity.x *= 0.95
                            cap.velocity.y *= 0.95

                            # ---------- ANGULAR ----------
                            torque = Vector3(
                                r.y * impulse.z - r.z * impulse.y,
                                r.z * impulse.x - r.x * impulse.z,
                                r.x * impulse.y - r.y * impulse.x
                            )

                            cap.angular_velocity += torque * cap.inv_inertia * 0.6

                        # ---------- SMALL GROUND TORQUE ----------
                        r = contact - cap.position

                        gravity_force = Vector3(0, 0, -9.81 * cap.mass)

                        torque = Vector3(
                            r.y * gravity_force.z - r.z * gravity_force.y,
                            r.z * gravity_force.x - r.x * gravity_force.z,
                            r.x * gravity_force.y - r.y * gravity_force.x
                        )

                        cap.angular_velocity -= torque * cap.inv_inertia * 0.01


                # ---------- SPHERE-CAPSULE ----------
                for body in self.bodies:

                    for cap in self.capsules:

                        # endpoints from rigid capsule
                        axis = quat_to_axis(cap.orientation)
                        p1 = cap.position + axis * cap.half_length
                        p2 = cap.position - axis * cap.half_length

                        # closest point
                        ab = p2 - p1
                        ab_len = ab.dot(ab)

                        if ab_len < 1e-6:
                            t = 0
                        else:
                            t = (body.position - p1).dot(ab) / ab_len
                            t = max(0.0, min(1.0, t))

                        closest = p1 + ab * t

                        delta = body.position - closest
                        dist = delta.magnitude()

                        min_dist = body.radius + cap.radius

                        if dist < min_dist and dist > 1e-6:

                            normal = delta / dist
                            penetration = min_dist - dist

                            # ---------- POSITION CORRECTION ----------
                            total_inv_mass = body.inv_mass + cap.inv_mass

                            correction = normal * (penetration / total_inv_mass) * 0.15

                            body.position += correction * body.inv_mass
                            cap.position -= correction * cap.inv_mass

                            # ---------- VELOCITY ----------
                            rel_v = body.velocity - cap.velocity
                            vn = rel_v.dot(normal)

                            if vn < 0:

                                restitution = 0.2

                                j = -(1 + restitution) * vn
                                j /= total_inv_mass

                                impulse = normal * j

                                body.velocity += impulse * body.inv_mass
                                cap.velocity -= impulse * cap.inv_mass

                                # ---------- ANGULAR ----------
                                r = closest - cap.position

                                torque = Vector3(
                                    r.y * impulse.z - r.z * impulse.y,
                                    r.z * impulse.x - r.x * impulse.z,
                                    r.x * impulse.y - r.y * impulse.x
                                )

                                angular_str = 0.1;
                                cap.angular_velocity -= torque * cap.inv_inertia * angular_str

                # ---- SYNC BACK ----
                for i, b in enumerate(self.bodies):
                    self.pos[i] = [b.position.x, b.position.y, b.position.z]
                    self.vel[i] = [b.velocity.x, b.velocity.y, b.velocity.z]

            for cap in self.capsules:
                cap.velocity *= 0.999
                if abs(cap.velocity.z) < 0.2:
                    cap.angular_velocity *= 0.98
                else:
                    cap.angular_velocity *= 0.995

                # ---------- SLEEP ----------
                if (
                    cap.velocity.magnitude() < 0.03 and
                    cap.angular_velocity.magnitude() < 0.03
                ):
                    cap.velocity = Vector3(0,0,0)
                    cap.angular_velocity = Vector3(0,0,0)

            # ---------- CAPSULE ROTATION ----------
            print("ANG VEL:", cap.angular_velocity.magnitude())
            for cap in self.capsules:
                cap.orientation = integrate_rotation(
                    cap.orientation,
                    cap.angular_velocity,
                    h
                )
        # ---------- FINAL ----------
        self.curr_pos = self.pos.copy()

    def export_state(self, alpha=1.0):

        # ---------- SPHERES ----------
        bodies_out = []

        for i, b in enumerate(self.bodies):

            interp = self.prev_pos[i] * (1 - alpha) + self.curr_pos[i] * alpha

            bodies_out.append({
                **b.serialize(),
                "position": interp.tolist()
            })

        # ---------- CAPSULES (RIGID BODY → ENDPOINTS) ----------
        capsules_out = []

        for cap in self.capsules:

            axis = quat_to_axis(cap.orientation)

            p1 = cap.position + axis * cap.half_length
            p2 = cap.position - axis * cap.half_length

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z

            height = max(0.001, math.sqrt(dx*dx + dy*dy + dz*dz))

            capsules_out.append({
                "p1": [p1.x, p1.y, p1.z],
                "p2": [p2.x, p2.y, p2.z],
                "radius": cap.radius,
                "height": height
            })

        # print("EXPORT:", len(bodies_out), "bodies,", len(capsules_out), "capsules")

        return {
            "bodies": bodies_out,
            "capsules": capsules_out
        }