from engine.vector import Vector3

def apply_sleep(cap):

    cap.velocity *= 0.999

    if abs(cap.velocity.z) < 0.2:
        cap.angular_velocity *= 0.98
    else:
        cap.angular_velocity *= 0.995

    if (
        cap.velocity.magnitude() < 0.03 and
        cap.angular_velocity.magnitude() < 0.03
    ):
        cap.velocity = Vector3(0,0,0)
        cap.angular_velocity = Vector3(0,0,0)