from engine.engine import integrate_rotation

def integrate_angular(cap, h):

    cap.orientation = integrate_rotation(
        cap.orientation,
        cap.angular_velocity,
        h
    )