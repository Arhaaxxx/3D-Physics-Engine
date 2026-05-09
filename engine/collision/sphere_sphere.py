from engine.vector import Vector3

def object_collisions(b1, b2):

    delta = b2.position - b1.position
    dist = delta.magnitude()

    if dist == 0:
        return

    normal = delta * (1.0 / dist)
    min_dist = b1.radius + b2.radius

    penetration = min_dist - dist

    if penetration > 0:

        # ---------- POSITION CORRECTION (BAUMGARTE) ----------
        percent = 0.2     # small correction
        slop = 0.01       # ignore tiny overlaps

        correction_mag = max(penetration - slop, 0.0) * percent
        correction = normal * correction_mag

        b1.position -= correction * 0.5
        b2.position += correction * 0.5

        # ---------- RELATIVE VELOCITY ----------
        rel_v = b2.velocity - b1.velocity
        vn = rel_v.dot(normal)

        # Do not resolve if separating
        if vn > 0:
            return

        # ---------- NORMAL IMPULSE ----------
        restitution = 0.2   # realistic bounce

        inv_mass1 = 1.0 / b1.mass
        inv_mass2 = 1.0 / b2.mass

        j = -(1 + restitution) * vn
        j /= (inv_mass1 + inv_mass2)

        impulse = normal * j

        b1.velocity -= impulse * inv_mass1
        b2.velocity += impulse * inv_mass2

        # ---------- FRICTION IMPULSE ----------
        rel_v = b2.velocity - b1.velocity  # recompute after normal impulse

        tangent = rel_v - normal * rel_v.dot(normal)

        if tangent.magnitude() > 1e-6:
            tangent = tangent.normalize()

            jt = -rel_v.dot(tangent)
            jt /= (inv_mass1 + inv_mass2)

            mu = 0.4   # friction coefficient

            # Coulomb friction clamp
            jt = max(-j * mu, min(jt, j * mu))

            friction_impulse = tangent * jt

            b1.velocity -= friction_impulse * inv_mass1
            b2.velocity += friction_impulse * inv_mass2

        # ---------- ANGULAR EFFECT (BASIC) ----------
        # Optional but improves realism
        r1 = normal * b1.radius
        r2 = normal * b2.radius

        torque1 = Vector3(
            r1.y * impulse.z - r1.z * impulse.y,
            r1.z * impulse.x - r1.x * impulse.z,
            r1.x * impulse.y - r1.y * impulse.x
        )

        torque2 = Vector3(
            r2.y * impulse.z - r2.z * impulse.y,
            r2.z * impulse.x - r2.x * impulse.z,
            r2.x * impulse.y - r2.y * impulse.x
        )

        b1.angular_velocity -= torque1 * inv_mass1 * 0.1
        b2.angular_velocity += torque2 * inv_mass2 * 0.1