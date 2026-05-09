from engine.vector import Vector3

def integrate_spheres(engine, h):

    if len(engine.pos) > 0:

        engine.vel[:, 2] -= 9.81 * h
        engine.vel *= 0.999

        engine.pos += engine.vel * h


def integrate_capsules(engine, h):

    for cap in engine.capsules:

        cap.velocity.z -= 9.81 * h
        cap.position += cap.velocity * h