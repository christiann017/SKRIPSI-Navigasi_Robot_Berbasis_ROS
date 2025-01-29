import rosbag
import pandas as pd
import tf.transformations as transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def quaternion_to_yaw(q):
    """ Convert a quaternion into a yaw angle (in radians). """
    euler = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    yaw = euler[2]
    return yaw

def extract_position_orientation_from_bag(bag_file):
    bag = rosbag.Bag(bag_file)
    data = []

    for topic, msg, t in bag.read_messages(topics=['/odom', '/pose']):
        timestamp = t.to_sec()
        if topic == '/odom':
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
        elif topic == '/pose':
            position = msg.pose.position
            orientation = msg.pose.orientation

        yaw = quaternion_to_yaw(orientation)
        
        # Append data to list in dictionary form
        data.append({
            'timestamp': timestamp,
            'position_x': position.x,
            'position_y': position.y,
            'orientation_yaw': yaw,
        })

    bag.close()
    return data

def filter_data_per_second(data):
    df = pd.DataFrame(data)
    
    # Round timestamp to the nearest second
    df['timestamp_rounded'] = df['timestamp'].round().astype(int)
    
    # Group by rounded timestamp (one data per second)
    df_filtered = df.groupby('timestamp_rounded').first().reset_index()
    
    # Rename columns as necessary
    df_filtered.rename(columns={'timestamp_rounded': 'timestamp'}, inplace=True)
    
    return df_filtered

def main():
    bag_file = 'sl_data.bag'  # Ganti dengan file bag Anda
    output_file = 'pose_data_per_second.xlsx'  # Nama file Excel output
    
    # Ekstrak data dari file ROS bag
    data = extract_position_orientation_from_bag(bag_file)
    
    # Filter data berdasarkan satu data per detik
    filtered_data = filter_data_per_second(data)
    
    # Simpan data ke file Excel
    filtered_data.to_excel(output_file, index=False)
    
    print(f"Data telah disimpan ke {output_file}")

if __name__ == "__main__":
    main()

