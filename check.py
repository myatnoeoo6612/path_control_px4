import rosbag2_py
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = "rosbag2_2026_02_09-01_37_52"

storage_options = rosbag2_py.StorageOptions(
    uri=bag_path,
    storage_id="sqlite3"
)
converter_options = rosbag2_py.ConverterOptions("", "")

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_type = get_message("geometry_msgs/msg/PoseStamped")

xs = []
ys = []
ts = []

t0 = None

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/aruco/pose_world":
        msg = deserialize_message(data, topic_type)

        if t0 is None:
            t0 = t

        ts.append((t - t0) * 1e-9)  # seconds
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)

# -------- Plot X ----------
plt.figure()
plt.plot(ts, xs)
plt.xlabel("Time (s)")
plt.ylabel("X position (m)")
plt.title("Landing X trajectory")
plt.grid()

# -------- Plot Y ----------
plt.figure()
plt.plot(ts, ys)
plt.xlabel("Time (s)")
plt.ylabel("Y position (m)")
plt.title("Landing Y trajectory")
plt.grid()

plt.show()
