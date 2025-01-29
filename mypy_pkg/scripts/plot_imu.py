import rosbag
import pandas as pd
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu

def extract_imu_data(bagfile, topic):
    bag = rosbag.Bag(bagfile)
    imu_data = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        imu_data.append([t.to_sec(), msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    bag.close()

    # Create a pandas DataFrame
    df = pd.DataFrame(imu_data, columns=['Time', 'LinearAccelX', 'LinearAccelY', 'LinearAccelZ', 
                                         'AngularVelX', 'AngularVelY', 'AngularVelZ'])
    return df

def plot_imu_data(df):
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

    ax1.plot(df['Time'].values, df['LinearAccelX'].values, label='Linear Acceleration X')
    ax1.plot(df['Time'].values, df['LinearAccelY'].values, label='Linear Acceleration Y')
    ax1.plot(df['Time'].values, df['LinearAccelZ'].values, label='Linear Acceleration Z')
    ax1.set_ylabel('Linear Acceleration (m/s^2)')
    ax1.legend(loc='upper right')  # Pilih lokasi yang diinginkan

    ax2.plot(df['Time'].values, df['AngularVelX'].values, label='Angular Velocity X')
    ax2.plot(df['Time'].values, df['AngularVelY'].values, label='Angular Velocity Y')
    ax2.plot(df['Time'].values, df['AngularVelZ'].values, label='Angular Velocity Z')
    ax2.set_ylabel('Angular Velocity (rad/s)')
    ax2.set_xlabel('Time (s)')
    ax2.legend(loc='upper right')  # Pilih lokasi yang diinginkan

    plt.show()

if __name__ == "__main__":
    bagfile = 'sl10_data.bag'  # Ganti dengan path ke file rosbag Anda
    topic = '/imu'  # Ganti dengan nama topik IMU Anda

    df = extract_imu_data(bagfile, topic)
    plot_imu_data(df)
'''import rosbag
import matplotlib.pyplot as plt

# Ganti path ke file rosbag Anda
bag_file = "sl10_data.bag"

# Inisialisasi list untuk menyimpan data waktu, kecepatan linear, dan kecepatan angular
time_stamps = []
linear_velocities = []
angular_velocities = []

# Membaca file rosbag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
        # Menyimpan timestamp, kecepatan linear, dan kecepatan angular
        time_stamps.append(t.to_sec())
        linear_velocities.append(msg.linear.x)
        angular_velocities.append(msg.angular.z)

# Periksa apakah list time_stamps tidak kosong
if time_stamps:
    # Mengubah waktu relatif terhadap waktu awal
    initial_time = time_stamps[0]
    time_stamps = [t - initial_time for t in time_stamps]

    # Plotting
    plt.figure(figsize=(12, 6))

    # Plot kecepatan linear
    plt.subplot(2, 1, 1)
    plt.plot(time_stamps, linear_velocities, label='Linear Velocity (m/s)', color='b')
    plt.ylabel("Linear Velocity (m/s)")
    plt.xlabel("Time (s)")
    plt.legend(loc="upper right")


    # Plot kecepatan angular
    plt.subplot(2, 1, 2)
    plt.plot(time_stamps, angular_velocities, label='Angular Velocity (rad/s)', color='r')
    plt.ylabel("Angular Velocity (rad/s)")
    plt.xlabel("Time (s)")
    plt.legend(loc="upper right")

    # Tampilkan plot
    plt.tight_layout()
    plt.show()
else:
    print("Tidak ada data pada topik /cmd_vel dalam rosbag.")'''

