import rosbag
from nav_msgs.msg import Odometry
import rospy

def calculate_active_time(bag_path, odom_topic, linear_threshold=0.0003, angular_threshold=0.01):
    bag = rosbag.Bag(bag_path)
    
    total_active_time = 0.0
    last_active_time = None

    for topic, msg, t in bag.read_messages(topics=[odom_topic]):
        if topic == odom_topic:
            linear_speed = msg.twist.twist.linear.x
            angular_speed = msg.twist.twist.angular.z

            if abs(linear_speed) >= linear_threshold or abs(angular_speed) >= angular_threshold:
                if last_active_time is not None:
                    duration = (t.to_sec() - last_active_time.to_sec())
                    total_active_time += duration
                last_active_time = t
            else:
                last_active_time = None

    bag.close()
    return total_active_time

if __name__ == "__main__":
    bag_path = 'nv_data.bag'  # Ganti dengan path ke file rosbag Anda
    odom_topic = '/odom'  # Ganti dengan topik odometry yang sesuai
    total_active_time = calculate_active_time(bag_path, odom_topic)
    print(f"Total active time: {total_active_time} seconds")

