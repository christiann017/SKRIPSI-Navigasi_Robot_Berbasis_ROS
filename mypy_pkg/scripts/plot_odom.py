import rosbag
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import tf.transformations as transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Fungsi untuk mengubah orientasi quaternion menjadi yaw (rotasi di sekitar sumbu z)
def quaternion_to_yaw(orientation):
    euler = transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return euler[2]

# Membuka file rosbag
bag_file = 'sl_data.bag'
bag = rosbag.Bag(bag_file)
odom_data = []

# Membaca pesan dari topik /odom dan /pose
for topic, msg, t in bag.read_messages(topics=['/odom', '/pose']):
    timestamp = t.to_sec()
    
    if topic == '/odom':
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
    elif topic == '/pose':
        position = msg.pose.position
        orientation = msg.pose.orientation

    yaw = quaternion_to_yaw(orientation)
    
    odom_data.append({
        'timestamp': timestamp,
        'position_x': position.x,
        'position_y': position.y,
        'orientation_yaw': yaw,
    })
print("X pos:",position.x)
print("Y pos:",position.y)
print("Z pos:",yaw)
bag.close()

# Konversi data ke DataFrame untuk analisis lebih lanjut
df = pd.DataFrame(odom_data)

# Pastikan data yang akan diindeks adalah numpy array
x_positions = np.array(df['position_x'])
y_positions = np.array(df['position_y'])

# Plot data odometri
plt.plot(x_positions, y_positions)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Odometry Data')
plt.grid()
plt.show()

