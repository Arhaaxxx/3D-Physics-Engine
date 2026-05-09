from engine.engine import integrate_rotation
from engine.math.transforms import quaternion_to_matrix3

def integrate_angular(cap, h):

    R = quaternion_to_matrix3(cap.orientation)

    RT = R.transpose()

    cap.world_inv_inertia_tensor = (
        R.matmul(cap.inv_inertia_tensor).matmul(RT)
    )

    cap.orientation = integrate_rotation(
        cap.orientation,
        cap.angular_velocity,
        h
    )