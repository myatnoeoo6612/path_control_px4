import numpy as np

dt = 0.1
v = 0.3
z = 3.0

L = 12.0        # total length
A = 1.0         # zigzag amplitude
zig_len = 2.0   # forward distance per segment

def yaw_to_quat(yaw):
    return np.cos(yaw/2), 0, 0, np.sin(yaw/2)

def write_line(f, t, x, y, yaw, vx, vy):
    qw, qx, qy, qz = yaw_to_quat(yaw)
    f.write(f"{t:.2f} {x:.3f} {y:.3f} {z:.2f} {qw:.4f} {qx} {qy} {qz:.4f} {vx:.3f} {vy:.3f} 0 0 0 0\n")

def line_segment(f, t, p0, p1):
    """Generate straight line between two points"""
    dist = np.linalg.norm(p1 - p0)
    steps = int(dist / (v * dt))

    for i in range(steps):
        alpha = i / steps
        p = (1 - alpha) * p0 + alpha * p1

        direction = p1 - p0
        yaw = np.arctan2(direction[1], direction[0])

        vx = v * np.cos(yaw)
        vy = v * np.sin(yaw)

        write_line(f, t, p[0], p[1], yaw, vx, vy)
        t += dt

    return t


with open("zigzag_sharp.txt", "w") as f:

    t = 0.0

    # ===== FORWARD ZIGZAG =====
    points = []
    x = 0.0

    toggle = True
    while x < L:
        y = A if toggle else -A
        points.append(np.array([x, y]))
        x += zig_len
        toggle = not toggle

    # ensure final point exactly at L
    points.append(np.array([L, 0.0]))

    # start from origin
    prev = np.array([0.0, 0.0])

    for p in points:
        t = line_segment(f, t, prev, p)
        prev = p

    # ===== BACKWARD (return) =====
    for p in reversed(points[:-1]):
        t = line_segment(f, t, prev, p)
        prev = p

    # back to origin
    t = line_segment(f, t, prev, np.array([0.0, 0.0]))

print("Sharp zigzag (forward + return) generated")