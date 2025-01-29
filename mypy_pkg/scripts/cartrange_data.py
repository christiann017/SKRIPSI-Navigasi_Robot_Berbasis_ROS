import rosbag
import numpy as np
import pandas as pd
from sensor_msgs.msg import LaserScan

def polar_to_cartesian(ranges, angle_min, angle_increment):
    angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    return x, y

def extract_scan_data(bagfile, scan_topic):
    bag = rosbag.Bag(bagfile)
    scan_data = []

    for topic, msg, t in bag.read_messages(topics=[scan_topic]):
        time = t.to_sec()
        ranges = np.array(msg.ranges)
        x, y = polar_to_cartesian(ranges, msg.angle_min, msg.angle_increment)
        
        for xi, yi in zip(x, y):
            scan_data.append([time, xi, yi])

    bag.close()

    # Create a pandas DataFrame
    df = pd.DataFrame(scan_data, columns=['Time', 'X', 'Y'])
    return df

def save_scan_data_to_text(df, filename):
    with open(filename, 'w') as f:
        for index, row in df.iterrows():
            f.write(f"Time: {row['Time']}\n")
            f.write(f"X: {row['X']}, Y: {row['Y']}\n")
            f.write("\n")
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'nv_data.bag'  # Ganti dengan path ke file rosbag Anda
    scan_topic = '/scan'  # Ganti dengan nama topik scan Anda

    df = extract_scan_data(bagfile, scan_topic)
    save_scan_data_to_text(df, 'koordinatrange_data.txt')

