import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

def plot_map_odom_scan(bag_file, map_timestamp, target_timestamp):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)

    # Inisialisasi variabel untuk menyimpan data peta, odometri, dan scan
    map_data = None
    odom_data = {'x': [], 'y': []}
    scan_data = None
    scan_x = []
    scan_y = []
    reference_point_x = None
    reference_point_y = None

    # Baca pesan dari topik /map dan pilih berdasarkan timestamp
    for topic, msg, t in bag.read_messages(topics=['/map']):
        timestamp = t.to_sec()
        if np.isclose(timestamp, map_timestamp, atol=1e-1):
            map_data = msg
            break  # Berhenti jika menemukan data peta yang sesuai

    # Baca pesan dari topik /odom
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        position = msg.pose.pose.position
        odom_data['x'].append(position.x)
        odom_data['y'].append(position.y)

    # Baca pesan dari topik /scan dan pilih berdasarkan timestamp
    for topic, msg, t in bag.read_messages(topics=['/scan']):
        timestamp = msg.header.stamp.to_sec()
        if np.isclose(timestamp, target_timestamp, atol=1e-3):
            scan_data = msg
            break  # Berhenti jika menemukan data scan yang sesuai

    bag.close()

    if map_data is None:
        print(f"No map data found at timestamp {map_timestamp}.")
        return

    if scan_data is None:
        print(f"No scan data found at timestamp {target_timestamp}.")
        return

    # Konversi data peta ke numpy array
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    map_array = np.array(map_data.data).reshape((height, width))

    # Plot peta menggunakan matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray', origin='lower',
               extent=[origin_x, origin_x + width * resolution,
                       origin_y, origin_y + height * resolution])
    plt.colorbar(label='Occupancy Probability')

    # Plot jalur odometri di atas peta
    plt.plot(odom_data['x'], odom_data['y'], 'b-', label='Odometry Path')

    # Konversi data scan ke koordinat x, y
    ranges = np.array(scan_data.ranges)
    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))
    scan_x = ranges * np.cos(angles)
    scan_y = ranges * np.sin(angles)

    # Plot semua data scan dengan titik hijau
    plt.plot(scan_x, scan_y, 'g.', label='Laser Scan Data')

    # Ambil titik referensi pada derajat ke-30
    reference_index = int((180.0 - np.degrees(scan_data.angle_min)) / np.degrees(scan_data.angle_increment))
    if 0 <= reference_index < len(ranges):
        reference_point_x = scan_x[reference_index]
        reference_point_y = scan_y[reference_index]
        # Plot titik referensi dengan warna merah
        plt.plot(reference_point_x, reference_point_y, 'ro', label='Reference Point (180 degrees)')

    # Konfigurasi plot
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title(f'Occupancy Grid Map at {map_timestamp:.3f} sec with Laser Scan at {target_timestamp:.3f} sec')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Contoh penggunaan
bag_file = 'sl_data.bag'  # Ganti dengan nama file rosbag Anda
map_timestamp = 300 # Ganti dengan timestamp untuk peta
target_timestamp = 99.01 # Ganti dengan timestamp untuk scan
plot_map_odom_scan(bag_file, map_timestamp, target_timestamp)

