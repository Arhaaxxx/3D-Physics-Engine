from engine.vector import Vector3
from engine.quaternion import quat_to_axis

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