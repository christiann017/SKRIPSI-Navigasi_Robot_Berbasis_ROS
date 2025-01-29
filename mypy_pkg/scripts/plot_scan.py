import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

def plot_scan_data_at_timestamp(bag_file, target_timestamp):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)
    
    # Inisialisasi variabel untuk menyimpan data laser scan
    scan_data = None
    
    # Baca pesan dari topik /scan
    for topic, msg, t in bag.read_messages(topics=['/scan']):
        timestamp = msg.header.stamp.to_sec()
        
        if np.isclose(timestamp, target_timestamp, atol=1e-3):
            scan_data = msg
            break  # Berhenti jika menemukan data yang sesuai dengan timestamp
    
    bag.close()
    
    if scan_data is None:
        print(f"No scan data found at timestamp {target_timestamp}.")
        return
    
    # Ekstraksi data range dari pesan LaserScan
    ranges = np.array(scan_data.ranges)
    angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))
    
    # Konversi data range dan angle menjadi koordinat x dan y
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    
    # Plot data laser scan
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'o')
    plt.title(f'Laser Scan at Timestamp {target_timestamp}')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Contoh penggunaan
bag_file = 'sl_data.bag'
target_timestamp = 307.715  # Ganti dengan timestamp yang diinginkan
plot_scan_data_at_timestamp(bag_file, target_timestamp)

