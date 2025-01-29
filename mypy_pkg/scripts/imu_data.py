import rosbag
import pandas as pd
import numpy as np
from sensor_msgs.msg import Imu

def extract_imu_data(bagfile, topic):
    bag = rosbag.Bag(bagfile)
    imu_data = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        imu_data.append([
            t.to_sec(), 
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            np.array(msg.orientation_covariance).reshape(3, 3).tolist(),
            np.array(msg.linear_acceleration_covariance).reshape(3, 3).tolist(),
            np.array(msg.angular_velocity_covariance).reshape(3, 3).tolist()
        ])

    bag.close()

    # Create a pandas DataFrame
    df = pd.DataFrame(imu_data, columns=[
        'Time', 
        'OrientationX', 'OrientationY', 'OrientationZ', 'OrientationW',
        'LinearAccelX', 'LinearAccelY', 'LinearAccelZ', 
        'AngularVelX', 'AngularVelY', 'AngularVelZ',
        'OrientationCovariance', 'LinearAccelCovariance', 'AngularVelCovariance'
    ])
    return df

def save_imu_data_to_text(df, filename):
    with open(filename, 'w') as f:
        for index, row in df.iterrows():
            f.write(f"Time: {row['Time']}\n")
            f.write(f"Orientation: [{row['OrientationX']}, {row['OrientationY']}, {row['OrientationZ']}, {row['OrientationW']}]\n")
            f.write(f"Linear Acceleration: [{row['LinearAccelX']}, {row['LinearAccelY']}, {row['LinearAccelZ']}]\n")
            f.write(f"Angular Velocity: [{row['AngularVelX']}, {row['AngularVelY']}, {row['AngularVelZ']}]\n")
            f.write(f"Orientation Covariance: {row['OrientationCovariance']}\n")
            f.write(f"Linear Acceleration Covariance: {row['LinearAccelCovariance']}\n")
            f.write(f"Angular Velocity Covariance: {row['AngularVelCovariance']}\n")
            f.write("\n")
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'nv_data.bag'  # Ganti dengan path ke file rosbag Anda
    topic = '/imu'  # Ganti dengan nama topik IMU Anda

    df = extract_imu_data(bagfile, topic)
    save_imu_data_to_text(df, 'imu_data22.txt')

