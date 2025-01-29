import rosbag
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
import tf.transformations as tf

def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle (rotation around the z-axis)."""
    euler = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]  # Return yaw angle (rotation around z-axis)

def plot_particle_cloud_on_map_separated(bag_file, duration=15, interval=0.0001):
    """Plot all particles from the start of the simulation and particle cloud every `interval` seconds after velocity command in separate plots."""
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)

    velocity_detected = False
    plotting_start_time = None
    map_data = None

    # Inisialisasi variabel untuk menyimpan partikel
    all_initial_particles = []
    timestamps = []
    all_particle_data = []

    # Baca data peta dari topik /map
    """for topic, msg, t in bag.read_messages(topics=['/map']):
        map_data = msg
        break  # Ambil hanya data peta pertama yang ditemukan

    if map_data is None:
        print("No map data found in the rosbag.")
        return

    # Ekstrak informasi dari pesan OccupancyGrid
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    # Konversi data peta menjadi numpy array
    map_array = np.array(map_data.data).reshape((height, width))"""
    
    # Baca pesan dari topik /cmd_vel dan /particlecloud
    for topic, msg, t in bag.read_messages(topics=['/cmd_vel', '/particlecloud']):
        timestamp = t.to_sec()

        # Jika data partikel masuk sebelum perintah kecepatan terdeteksi, simpan sebagai data awal
        if topic == '/particlecloud' and not velocity_detected:
            all_initial_particles.append(msg)

        # Deteksi perintah kecepatan dari /cmd_vel
        if topic == '/cmd_vel':
            if not velocity_detected:
                vel_linear = msg.linear.x
                vel_angular = msg.angular.z
                # Jika perintah kecepatan terdeteksi (baik linier atau angular)
                if vel_linear != 0 or vel_angular != 0:
                    velocity_detected = True
                    plotting_start_time = timestamp
                    print(f"Velocity command detected at {timestamp} sec. Start plotting after this point.")

        # Mulai memplot setelah perintah kecepatan terdeteksi
        if velocity_detected and topic == '/particlecloud':
            # Mulai plot hanya setelah perintah kecepatan terdeteksi
            elapsed_time = timestamp - plotting_start_time
            if elapsed_time >= 0 and elapsed_time <= duration and np.isclose(elapsed_time % interval, 0, atol=0.1):
                # Simpan data partikel pada waktu tertentu (tiap 1 detik)
                timestamps.append(timestamp)
                all_particle_data.append(msg)

    bag.close()

    if len(all_particle_data) == 0:
        print("No particle data found after velocity command.")
        return

    # Plot partikel awal pada plot terpisah
    plt.figure(figsize=(10, 10))
    plt.title('Initial Particles')

    # Plot peta sebagai latar belakang
    #plt.imshow(map_array, cmap='gray', origin='lower', extent=[origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution])
    
    # Plot partikel awal
    for particle_data in all_initial_particles:
        for pose in particle_data.poses:
            # Dapatkan posisi partikel
            x = pose.position.x
            y = pose.position.y

            # Dapatkan orientasi partikel (yaw)
            yaw = quaternion_to_yaw(pose.orientation)

            # Plot partikel sebagai titik biru
            plt.plot(x, y, 'bo', markersize=3)

            # Plot orientasi partikel sebagai panah biru
            dx = np.cos(yaw) * 0.1  # Skala panjang panah
            dy = np.sin(yaw) * 0.1  # Skala panjang panah
            plt.arrow(x, y, dx, dy, head_width=0.01, head_length=0.05, fc='b', ec='b')

    # Konfigurasi plot partikel awal
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.grid(True)
    plt.axis('equal')

    # Tampilkan plot partikel awal
    plt.tight_layout()
    plt.show()

    # Plot setiap perubahan partikel setelah perintah kecepatan, pada plot terpisah
    for idx, particle_data in enumerate(all_particle_data):
        plt.figure(figsize=(10, 10))
        plt.title(f'Particles at {timestamps[idx]:.3f} sec')

        # Plot peta sebagai latar belakang
        """plt.imshow(map_array, cmap='gray', origin='lower', extent=[origin_x, origin_x + width * resolution, origin_y, origin_y + height * resolution])"""
        
        for pose in particle_data.poses:
            # Dapatkan posisi partikel
            x = pose.position.x
            y = pose.position.y

            # Dapatkan orientasi partikel (yaw)
            yaw = quaternion_to_yaw(pose.orientation)

            # Plot partikel sebagai titik merah
            plt.plot(x, y, 'ro', markersize=3)

            # Plot orientasi partikel sebagai panah merah
            dx = np.cos(yaw) * 0.1  # Skala panjang panah
            dy = np.sin(yaw) * 0.1  # Skala panjang panah
            plt.arrow(x, y, dx, dy, head_width=0.01, head_length=0.05, fc='r', ec='r')

        # Konfigurasi plot
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.grid(True)
        plt.axis('equal')

        # Tampilkan plot
        plt.tight_layout()
        plt.show()

# Contoh penggunaan
bag_file = 'nv_data.bag'  # Ganti dengan nama file rosbag Anda
plot_particle_cloud_on_map_separated(bag_file, duration=15, interval=0.0001)

