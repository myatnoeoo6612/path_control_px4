import numpy as np

dt = 0.1
v = 0.3
z = 3.0

segment_length = 12.0
segment_time = segment_length / v

def yaw_to_quat(yaw):
    qw = np.cos(yaw/2)
    qz = np.sin(yaw/2)
    return qw, 0.0, 0.0, qz

def write_line(f, t, x, y, z, yaw, vx, vy, vz):
    qw, qx, qy, qz = yaw_to_quat(yaw)
    f.write(f"{t:.2f} {x:.3f} {y:.3f} {z:.2f} {qw:.4f} {qx} {qy} {qz:.4f} {vx:.3f} {vy:.3f} {vz} 0 0 0\n")

with open("square.txt", "w") as f:

    t = 0.0

    # ===== SEGMENT 1 =====
    t_local = 0.0
    while t_local < segment_time:
        x = v * t_local
        y = 0.0
        write_line(f, t, x, y, z, 0.0, v, 0.0, 0.0)
        t += dt
        t_local += dt

    # ===== SEGMENT 2 =====
    t_local = 0.0
    while t_local < segment_time:
        x = segment_length
        y = v * t_local
        write_line(f, t, x, y, z, np.pi/2, 0.0, v, 0.0)
        t += dt
        t_local += dt

    # ===== SEGMENT 3 =====
    t_local = 0.0
    while t_local < segment_time:
        x = segment_length - v * t_local
        y = segment_length
        write_line(f, t, x, y, z, np.pi, -v, 0.0, 0.0)
        t += dt
        t_local += dt

    # ===== SEGMENT 4 =====
    t_local = 0.0
    while t_local < segment_time:
        x = 0.0
        y = segment_length - v * t_local
        write_line(f, t, x, y, z, -np.pi/2, 0.0, -v, 0.0)
        t += dt
        t_local += dt

print("Square trajectory fixed")