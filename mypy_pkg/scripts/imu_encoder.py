import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu

def extract_sensor_data(bag_path, imu_topic):
    bag = rosbag.Bag(bag_path)
    imu_times = []
    imu_orientations = []

    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        if topic == imu_topic:
            imu_times.append(t.to_sec())
            imu_orientations.append(msg.linear_acceleration)  # Sesuaikan dengan data IMU yang ingin dianalisis
    bag.close()
    return imu_times, imu_orientations

def plot_sensor_data(imu_times, imu_orientations):
    plt.figure(figsize=(10, 5))
    plt.subplot(2, 1, 1)
    plt.plot(imu_times, [o.x for o in imu_orientations], label="IMU Orientation X")
    plt.xlabel("Time (s)")
    plt.ylabel("Orientation X")
    plt.title("IMU Orientation Over Time")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    bag_path = 'sl_data.bag'  # Ganti dengan path ke file rosbag Anda
    imu_topic = '/imu'  # Ganti dengan topik IMU Anda

    imu_times, imu_orientations = extract_sensor_data(bag_path, imu_topic)
    plot_sensor_data(imu_times, imu_orientations)

if __name__ == "__main__":
    main()

