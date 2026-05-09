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

    tangent = tangent.normalize()

    # tangential speed
    vt = v_contact.dot(tangent)

    # ---------- SMALL SURFACE FRICTION ----------
    friction_strength = 0.08

    friction_delta = tangent * (-vt * friction_strength)

    # apply ONLY tangent damping
    cap.velocity += friction_delta

    # ---------- SMALL ROLLING RESPONSE ----------

    rolling_axis = normal.cross(tangent)

    rolling_transfer = friction_delta.magnitude() * 0.2

    cap.angular_velocity += (
        rolling_axis * rolling_transfer
    )