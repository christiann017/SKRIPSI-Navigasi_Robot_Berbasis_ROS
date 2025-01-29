import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion

# Buka file rosbag
bag = rosbag.Bag('sl_data.bag')

# Inisialisasi list untuk menyimpan data odometry
odom_data = {'x': [], 'y': []}

# Baca pesan dari topik /odom
for topic, msg, t in bag.read_messages(topics=['/odom']):
    position = msg.pose.pose.position
    odom_data['x'].append(position.x)
    odom_data['y'].append(position.y)

# Baca pesan dari topik /map
map_msg = None
for topic, msg, t in bag.read_messages(topics=['/map']):
    map_msg = msg
    break  # Ambil hanya pesan pertama dari topik /map

bag.close()

# Konversi data odometry ke numpy array
odom_data['x'] = np.array(odom_data['x'])
odom_data['y'] = np.array(odom_data['y'])

if map_msg:
    # Ekstrak informasi dari OccupancyGrid
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin = map_msg.info.origin.position

    # Konversi data peta ke numpy array
    map_data = np.array(map_msg.data).reshape((height, width))

    # Buat koordinat kartesius untuk peta
    x_coords = np.linspace(origin.x, origin.x + width * resolution, width)
    y_coords = np.linspace(origin.y, origin.y + height * resolution, height)
    X, Y = np.meshgrid(x_coords, y_coords)

    # Plot data odometry dan peta secara bersamaan
    plt.figure(figsize=(10, 8))
    plt.imshow(map_data, cmap='gray', origin='lower', extent=[origin.x, origin.x + width * resolution, origin.y, origin.y + height * resolution])
    plt.plot(odom_data['x'], odom_data['y'], label='Odometry', color='red', linewidth=2)
    plt.colorbar(label='Occupancy')
    plt.title('Odometry and Occupancy Grid Map')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
else:
    print("No map data found.")

