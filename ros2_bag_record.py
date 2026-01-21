from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg
from pathlib import Path
import pandas as pd

# ================= CONFIG =================
bag_path = Path("/home/myat/path_control_px4/rosbag2_a")  # CHANGE ME
px4_msg_path = Path("/home/myat/path_control_px4/src/px4_msgs/msg")

# ================= LOAD TYPESTORE =================
typestore = get_typestore(Stores.ROS2_HUMBLE)

# Load PX4 msg definitions
for msg_file in px4_msg_path.glob("*.msg"):
    msg_name = f"px4_msgs/msg/{msg_file.stem}"
    typestore.register(get_types_from_msg(msg_file.read_text(), msg_name))

# ================= READ BAG =================
est_data = []
sp_data = []

with AnyReader([bag_path], default_typestore=typestore) as reader:
    for connection, timestamp, rawdata in reader.messages():
        msg = reader.deserialize(rawdata, connection.msgtype)

        if connection.topic == "/fmu/out/vehicle_odometry":
            est_data.append([
                timestamp * 1e-9,
                msg.position[0],
                msg.position[1],
                msg.position[2]
            ])

        elif connection.topic == "/fmu/in/trajectory_setpoint":
            sp_data.append([
                timestamp * 1e-9,
                msg.position[0],
                msg.position[1],
                msg.position[2]
            ])

# ================= SAVE CSV =================
df_est = pd.DataFrame(est_data, columns=["time", "x", "y", "z"])
df_sp  = pd.DataFrame(sp_data,  columns=["time", "x", "y", "z"])

df_est.to_csv("flight_estimated.csv", index=False)
df_sp.to_csv("flight_setpoint.csv", index=False)

print("Saved:")
print(" - flight_estimated.csv")
print(" - flight_setpoint.csv")

