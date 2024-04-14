from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg
from rosbags.typesys import Stores, get_typestore
from IPython import embed
import numpy as np 

bagpath = Path('/home/m4pc/m4v2-code/m4_home/m4_ws/rosbag2_2024_04_07-13_56_09')
topic_name = 'mpc_status'
type_path = '/home/m4pc/m4v2-code/m4_home/m4_ws/src/custom_msgs/msg/MPCStatus.msg'

###### Add custom type to typestore ######

# Read definitions to python strings.
msg_text = Path(type_path).read_text()

# Plain dictionary to hold message definitions.
add_types = {}

# Add definitions from one msg file to the dict.
add_types.update(get_types_from_msg(msg_text, 'custom_msgs/msg/MPCStatus'))
typestore = get_typestore(Stores.ROS2_FOXY)
typestore.register(add_types)

###### Post-Processing Step #######

# create reader instance and open for reading
N_samples = 0
rmse = 0.0
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == topic_name]
    for connection, timestamp, rawdata in reader.messages(connections=connections):
         msg = reader.deserialize(rawdata, connection.msgtype)
         p    = np.array([msg.x,msg.y,msg.z])
         pref = np.array([msg.xref[0],msg.xref[1],msg.xref[2]])
         e = np.linalg.norm(p-pref)
         rmse += e
         N_samples += 1
rmse = 1/N_samples*rmse
rmse = np.sqrt(rmse)
print(rmse)