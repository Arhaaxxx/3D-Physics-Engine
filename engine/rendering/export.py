from engine.quaternion import quat_to_axis
import math

def export_state(engine, alpha=1.0):

        # ---------- SPHERES ----------
        bodies_out = []

        for i, b in enumerate(engine.bodies):

            interp = engine.prev_pos[i] * (1 - alpha) + engine.curr_pos[i] * alpha

            bodies_out.append({
                **b.serialize(),
                "position": interp.tolist()
            })

        # ---------- CAPSULES (RIGID BODY → ENDPOINTS) ----------
        capsules_out = []

        for cap in engine.capsules:

            axis = quat_to_axis(cap.orientation)

            p1 = cap.position + axis * cap.half_length
            p2 = cap.position - axis * cap.half_length

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.z - p1.z

            height = max(0.001, math.sqrt(dx*dx + dy*dy + dz*dz))

            capsules_out.append({
                "p1": [p1.x, p1.y, p1.z],
                "p2": [p2.x, p2.y, p2.z],
                "radius": cap.radius,
                "height": height
            })

        # print("EXPORT:", len(bodies_out), "bodies,", len(capsules_out), "capsules")

        return {
            "bodies": bodies_out,
            "capsules": capsules_out
        }