import rosbag
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
import pandas as pd

def quaternion_to_euler(quaternion):
    """
    Konversi quaternion ke sudut Euler (roll, pitch, yaw)
    """
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
    return roll, pitch, yaw

def extract_amcl_pose_data(bagfile, amcl_pose_topic):
    bag = rosbag.Bag(bagfile)
    amcl_pose_data = []

    for topic, msg, t in bag.read_messages(topics=[amcl_pose_topic]):
        time = t.to_sec()

        # Ekstrak posisi dan orientasi
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(orientation)

        # Simpan data waktu, posisi, dan orientasi (yaw) ke list
        amcl_pose_data.append([
            time,
            position.x,
            position.y,
            yaw  # Kita hanya menyimpan yaw untuk orientasi 2D
        ])

    bag.close()

    # Membuat pandas DataFrame
    df = pd.DataFrame(amcl_pose_data, columns=[
        'Time', 
        'PositionX', 
        'PositionY', 
        'Yaw'
    ])
    return df

def save_amcl_pose_data_to_text(df, filename):
    with open(filename, 'w') as f:
        for index, row in df.iterrows():
            f.write(f"Time: {row['Time']}\n")
            f.write(f"  Position - x: {row['PositionX']}, y: {row['PositionY']}\n")
            f.write(f"  Orientation (radian) - yaw: {row['Yaw']}\n")
            f.write("\n")
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'nv_data.bag'  # Ganti dengan path ke file rosbag Anda
    amcl_pose_topic = '/amcl_pose'  # Ganti dengan topik amcl_pose

    # Ekstrak data /amcl_pose dari rosbag
    df = extract_amcl_pose_data(bagfile, amcl_pose_topic)
    
    # Simpan data ke file teks
    save_amcl_pose_data_to_text(df, 'amcl_pose_data.txt')

