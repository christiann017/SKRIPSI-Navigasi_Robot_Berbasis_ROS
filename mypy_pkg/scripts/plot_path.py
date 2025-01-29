import rosbag
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

# Fungsi untuk menghitung lintasan berdasarkan kecepatan linear dan angular
def generate_trajectory(initial_pose, linear_velocity, angular_velocity, delta_time):
    x, y, theta = initial_pose
    trajectory = [(x, y)]
    
    for _ in range(int(delta_time / 0.1)):  # Resolusi 0.1 detik untuk hasil yang lebih halus
        if angular_velocity == 0:  # Gerakan lurus
            x += linear_velocity * 0.1 * np.cos(theta)
            y += linear_velocity * 0.1 * np.sin(theta)
        else:  # Gerakan melingkar
            radius = linear_velocity / angular_velocity
            theta += angular_velocity * 0.1
            x += radius * (np.sin(theta) - np.sin(theta - angular_velocity * 0.1))
            y += radius * (-np.cos(theta) + np.cos(theta - angular_velocity * 0.1))
        
        trajectory.append((x, y))
    
    return np.array(trajectory)

# Load rosbag dan ambil data costmap
bag = rosbag.Bag('nv_data2.bag')
costmap_data = None

for topic, msg, t in bag.read_messages(topics=['/move_base/global_costmap/costmap']):
    costmap_data = msg
    break  # Ambil hanya costmap pertama untuk keperluan plot

# Resolusi costmap dalam meter per sel
resolution = costmap_data.info.resolution

# Pose awal dalam koordinat meter (misalnya, x=0, y=0, theta=0)
initial_pose = (-0.08664, 0.13543, -0.11096)

# Daftar kombinasi perintah kecepatan linear dan angular
commands = [
    (0.125, -1.82),
    (0.25, -1.82),
    (0.25, 0.0),
    (0.125, 0.0),
    (0.125, 1.82),
    (0.25, 1.82),  # Gerakan lurus
]

# Waktu perubahan (2 detik)
delta_time = 2.0

# Konversi costmap ke numpy array untuk visualisasi
costmap = np.array(costmap_data.data).reshape(costmap_data.info.height, costmap_data.info.width)

# Tentukan origin costmap dalam meter
origin_x = costmap_data.info.origin.position.x
origin_y = costmap_data.info.origin.position.y

# Plot costmap sebagai background
extent = (origin_x, origin_x + costmap_data.info.width * resolution, 
          origin_y, origin_y + costmap_data.info.height * resolution)
plt.imshow(costmap, origin='lower', cmap='gray', extent=extent)

# Tampilkan nilai cost di tengah setiap sel
for i in range(costmap_data.info.height):
    for j in range(costmap_data.info.width):
        x = origin_x + (j + 0.5) * resolution  # Tengah sel di koordinat x
        y = origin_y + (i + 0.5) * resolution  # Tengah sel di koordinat y
        cost = costmap[i, j]
        plt.text(x, y, f"{cost}", ha='center', va='center', color='red', fontsize=8)

# Plot setiap lintasan untuk kombinasi kecepatan
for linear_vel, angular_vel in commands:
    trajectory = generate_trajectory(initial_pose, linear_vel, angular_vel, delta_time)
    plt.plot(trajectory[:, 0], trajectory[:, 1], label=f"v={linear_vel}, ω={angular_vel}")

# Pengaturan plot
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Lintasan Transisi Keadaan dari Kombinasi Kecepatan dengan Cost Value")
plt.legend()
plt.grid()

# Tampilkan plot dengan fungsi blocking agar interaktif
plt.show(block=True)

# Tutup bag setelah plotting selesai
bag.close()

