from engine.vector import Vector3
from engine.quaternion import quat_to_axis
from engine.solver.friction_solver import apply_friction

def solve_capsule_ground(cap):

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
        v_contact = cap.velocity + cap.angular_velocity.cross(r)

        # ---------- NORMAL VELOCITY ----------
        # use stronger downward component
        vn = min(v_contact.dot(normal), cap.velocity.z)

        if vn < 0:

            restitution = 0.3

            # effective rotational mass
            r_cross_n = Vector3(
                r.y * normal.z - r.z * normal.y,
                r.z * normal.x - r.x * normal.z,
                r.x * normal.y - r.y * normal.x
            )

            effective_mass = (
                cap.inv_mass +
                (r_cross_n.magnitude() ** 2) * (
                    (
                        cap.inv_inertia_tensor.m[0][0] +
                        cap.inv_inertia_tensor.m[1][1] +
                        cap.inv_inertia_tensor.m[2][2]
                    ) / 3.0
                )
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
            torque = r.cross(impulse)

            angular_accel = cap.world_inv_inertia_tensor * torque

            cap.angular_velocity += angular_accel * 0.6

            apply_friction(
                cap,
                normal,
                v_contact,
                vn,
                r,
                j
            )