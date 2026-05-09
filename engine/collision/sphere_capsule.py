from engine.vector import Vector3
from engine.quaternion import quat_to_axis

def solve_sphere_capsule(cap, body):
    
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