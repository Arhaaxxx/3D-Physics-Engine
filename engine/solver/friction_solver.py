from engine.vector import Vector3

def apply_friction(
    cap,
    normal,
    v_contact,
    vn,
    r,
    j
):

    tangent = v_contact - normal * vn

    tangent_mag = tangent.magnitude()

    if tangent_mag < 1e-6:
        return

    tangent = tangent / tangent_mag

    vt = v_contact.dot(tangent)

    r_cross_t = Vector3(
        r.y * tangent.z - r.z * tangent.y,
        r.z * tangent.x - r.x * tangent.z,
        r.x * tangent.y - r.y * tangent.x
    )

    friction_effective_mass = (
        cap.inv_mass +
        (r_cross_t.magnitude() ** 2) * (
            (
                cap.inv_inertia_tensor.m[0][0] +
                cap.inv_inertia_tensor.m[1][1] +
                cap.inv_inertia_tensor.m[2][2]
            ) / 3.0
        )
    )

    jt = -vt

    if friction_effective_mass > 1e-6:
        jt /= friction_effective_mass

    mu = 0.5

    jt = max(-mu * j, min(jt, mu * j))

    friction_impulse = tangent * jt

    # ---------- LINEAR ----------
    cap.velocity += friction_impulse * cap.inv_mass

    # ---------- ANGULAR ----------
    friction_torque = Vector3(
        r.y * friction_impulse.z - r.z * friction_impulse.y,
        r.z * friction_impulse.x - r.x * friction_impulse.z,
        r.x * friction_impulse.y - r.y * friction_impulse.x
    )

    friction_angular_accel = (
        cap.world_inv_inertia_tensor * friction_torque
    )

    cap.angular_velocity += friction_angular_accel