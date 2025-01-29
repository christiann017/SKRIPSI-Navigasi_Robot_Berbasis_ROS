import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

# Buka file rosbag
bag = rosbag.Bag('nv_data.bag')

# Baca pesan dari topik /map
map_msg = None
for topic, msg, t in bag.read_messages(topics=['/map']):
    map_msg = msg
    break  # Ambil hanya pesan pertama dari topik /map

bag.close()

if map_msg:
    # Ekstrak informasi dari OccupancyGrid
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin = map_msg.info.origin.position

    # Konversi data peta ke numpy array
    map_data = np.array(map_msg.data).reshape((height, width))

    # Plot data peta
    plt.figure(figsize=(10, 8))
    plt.imshow(map_data, cmap='gray', origin='lower')
    plt.colorbar(label='Occupancy')
    plt.title('Occupancy Grid Map')
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.show()
else:
    print("No map data found.")

