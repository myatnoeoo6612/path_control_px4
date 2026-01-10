import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===================== LOAD CSV FILES =====================
est = pd.read_csv("flight_estimated.csv")
sp  = pd.read_csv("flight_setpoint.csv")

print("Loaded:")
print(f"  Estimated samples : {len(est)}")
print(f"  Setpoint samples  : {len(sp)}")

# ===================== 2D TRAJECTORY (EAST-NORTH) =====================
plt.figure(figsize=(8, 6))

plt.plot(
    est["x"], est["y"],
    label="Estimated",
    linewidth=2,
    color="tab:blue"
)

plt.plot(
    sp["x"], sp["y"],
    "--",
    label="Setpoint",
    linewidth=2,
    color="tab:orange"
)

plt.xlabel("East [m]")
plt.ylabel("North [m]")
plt.title("2D Trajectory (East–North)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()

# ===================== 3D TRAJECTORY (ENU) =====================
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")

# PX4 publishes Z-down → convert to Z-up for visualization
ax.plot(
    est["x"], est["y"], -est["z"],
    label="Estimated",
    linewidth=2,
    color="tab:blue"
)

ax.plot(
    sp["x"], sp["y"], -sp["z"],
    "--",
    label="Setpoint",
    linewidth=2,
    color="tab:orange"
)

ax.set_xlabel("East [m]")
ax.set_ylabel("North [m]")
ax.set_zlabel("Up [m]")
ax.set_title("3D Trajectory (ENU)")
ax.legend()

# Equal axis scaling (important for correctness)
max_range = max(
    est["x"].max() - est["x"].min(),
    est["y"].max() - est["y"].min(),
    (-est["z"]).max() - (-est["z"]).min()
)

mid_x = (est["x"].max() + est["x"].min()) * 0.5
mid_y = (est["y"].max() + est["y"].min()) * 0.5
mid_z = ((-est["z"]).max() + (-est["z"]).min()) * 0.5

ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

plt.tight_layout()
plt.show()

