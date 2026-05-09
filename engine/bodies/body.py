from engine.vector import *
from engine.funcs import *

class Body:
    def __init__(self, mass, position, velocity,shape= "sphere", radius=1.0, height=1.0):
        self.mass = mass
        self.position = position
        self.velocity = velocity

        self.inv_mass = 1.0 / mass if mass > 0 else 0.0 

        self.shape = shape
        self.radius = radius
        self.height = height

        self.orientation = Vector3(0, 0, 1)
        self.angular_velocity = Vector3(0, 0, 0)

        self.force = Vector3()
        self.torque = Vector3()
        self.inertia = self.mass * 0.2

    def apply_force(self, force):
        if force is None:
            return
        self.force = self.force + force

    def update(self, dt):
        # -------------------------
        # LINEAR MOTION
        # -------------------------
        acceleration = self.force * (1 / self.mass)

        self.velocity = self.velocity + acceleration * dt
        self.position = self.position + self.velocity * dt

        # -------------------------
        # ANGULAR MOTION
        # -------------------------
        ang_acc = self.torque * (1 / self.inertia)

        self.angular_velocity = self.angular_velocity + ang_acc * dt

        # -------------------------
        # PROPER ROTATION UPDATE (CRITICAL FIX)
        # -------------------------
        omega_mag = self.angular_velocity.magnitude()

        if omega_mag > 0:
            axis = self.angular_velocity * (1.0 / omega_mag)
            angle = omega_mag * dt

            k = axis
            v = self.orientation

            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            self.orientation = Vector3(
                v.x * cos_a + (k.y * v.z - k.z * v.y) * sin_a + k.x * dot(k, v) * (1 - cos_a),
                v.y * cos_a + (k.z * v.x - k.x * v.z) * sin_a + k.y * dot(k, v) * (1 - cos_a),
                v.z * cos_a + (k.x * v.y - k.y * v.x) * sin_a + k.z * dot(k, v) * (1 - cos_a),
            )

            # normalize to prevent drift
            self.orientation = self.orientation.normalize()

        # -------------------------
        # DAMPING (STABILITY)
        # -------------------------
        self.angular_velocity = self.angular_velocity * 0.955

        # if self.angular_velocity.magnitude() < 0.01:
        #     self.angular_velocity = Vector3(0,0,0)

        print("torque:", self.torque.x, self.torque.y, self.torque.z)
        print(f"ang vel: {self.angular_velocity.x:.6f} {self.angular_velocity.y:.6f} {self.angular_velocity.z:.6f}")
        print("inertia:", self.inertia)
        print("ang_acc:", (self.torque * (1 / self.inertia)).magnitude())

        # -------------------------
        # RESET FORCES
        # -------------------------
        self.force = Vector3()
        self.torque = Vector3()

    def serialize(self):
        return {
            "position": self.position.to_list(),
            "velocity": self.velocity.to_list(),
            "rotation": self.orientation.to_list(),
            "radius": self.radius
        }