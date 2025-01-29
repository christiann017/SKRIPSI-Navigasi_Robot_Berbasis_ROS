import rosbag
import numpy as np
from sensor_msgs.msg import LaserScan

def save_scan_data_at_timestamp(bag_file, target_timestamp, output_file):
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
    
    # Simpan data ke file teks
    with open(output_file, 'w') as f:
        f.write('x,y\n')
        for xi, yi in zip(x, y):
            f.write(f"{xi},{yi}\n")
    
    print(f"Data saved to {output_file}")

# Contoh penggunaan
bag_file = 'sl_data.bag'
target_timestamp = 307.715  # Ganti dengan timestamp yang diinginkan
output_file = 'scanT_data.txt'
save_scan_data_at_timestamp(bag_file, target_timestamp, output_file)

