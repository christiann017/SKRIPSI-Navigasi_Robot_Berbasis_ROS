import numpy as np
import matplotlib.pyplot as plt
import rosbag
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

# Fungsi untuk membaca data OccupancyGrid dari rosbag
def read_costmap_from_bag(bag_file, topic="/move_base/global_costmap/costmap"):
    bag = rosbag.Bag(bag_file)
    costmap_data = None

    for topic, msg, t in bag.read_messages(topics=[topic]):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Konversi data OccupancyGrid menjadi array numpy
        costmap_data = np.array(msg.data).reshape((height, width))
        break  # Ambil hanya data pertama di topik ini untuk visualisasi awal

    bag.close()
    return costmap_data, width, height, resolution, origin_x, origin_y

# Fungsi untuk membaca global plan dari rosbag
def read_global_plan_from_bag(bag_file, topic="/move_base/DWAPlannerROS/local_plan"):
    bag = rosbag.Bag(bag_file)
    global_plan_positions = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            global_plan_positions.append((x, y))
        break  # Ambil hanya data pertama di topik ini untuk visualisasi awal

    bag.close()
    return global_plan_positions

# Fungsi untuk memplot costmap dengan jalur global plan serta cost value di setiap cell
def plot_costmap_with_global_plan(costmap_data, width, height, resolution, origin_x, origin_y, global_plan_positions, start_x=0, start_y=0):
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Plot costmap
    cax = ax.imshow(costmap_data, cmap='viridis', origin="lower", extent=(origin_x, origin_x + width * resolution, 
                                                                         origin_y, origin_y + height * resolution))
    plt.colorbar(cax, label="Cost Value")
    ax.set_title("Costmap with Global Plan Path and Cost Values")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    
    # Hitung offset untuk titik awal
    if global_plan_positions:
        initial_x, initial_y = global_plan_positions[0]
        x_offset = start_x - initial_x
        y_offset = start_y - initial_y
    else:
        x_offset = y_offset = 0

    # Fungsi untuk memperbarui plot dengan posisi global plan dan nilai cost
    def update_plot_elements():
        # Hapus semua teks dan plot jalur sebelumnya
        ax.texts.clear()
        
        # Tambahkan nilai cost di tengah setiap cell grid yang terlihat
        x_min, x_max = ax.get_xlim()
        y_min, y_max = ax.get_ylim()
        
        x_min_index = int(np.floor((x_min - origin_x) / resolution))
        x_max_index = int(np.ceil((x_max - origin_x) / resolution))
        y_min_index = int(np.floor((y_min - origin_y) / resolution))
        y_max_index = int(np.ceil((y_max - origin_y) / resolution))

        for i in range(y_min_index, y_max_index):
            for j in range(x_min_index, x_max_index):
                if 0 <= i < height and 0 <= j < width:
                    value = costmap_data[i, j]
                    if value >= 0:  # Tampilkan angka hanya untuk cell yang diketahui
                        # Posisi tengah cell
                        cell_center_x = origin_x + (j + 0.5) * resolution
                        cell_center_y = origin_y + (i + 0.5) * resolution
                        ax.text(cell_center_x, cell_center_y, f"{value}", 
                                ha="center", va="center", color="white" if value > 50 else "black", fontsize=8)
        
        # Plot jalur global plan
        for (x, y) in global_plan_positions:
            x += x_offset
            y += y_offset
            ax.plot(x, y, color="blue", marker="o", markersize=2)

        fig.canvas.draw_idle()

    # Fungsi untuk menangani zoom dan scroll event
    def on_scroll(event):
        # Periksa apakah event berada dalam area plot
        if event.xdata is None or event.ydata is None:
            return  # Abaikan event di luar area plot

        x_min, x_max = ax.get_xlim()
        y_min, y_max = ax.get_ylim()
        x_range, y_range = x_max - x_min, y_max - y_min

        # Adjust zoom level
        zoom_factor = 0.9 if event.button == 'up' else 1.1  # Scroll up to zoom in, down to zoom out

        new_width = x_range * zoom_factor
        new_height = y_range * zoom_factor

        # Center the zoom on the mouse location
        rel_x = (event.xdata - x_min) / x_range
        rel_y = (event.ydata - y_min) / y_range

        new_x_min = event.xdata - new_width * rel_x
        new_x_max = event.xdata + new_width * (1 - rel_x)
        new_y_min = event.ydata - new_height * rel_y
        new_y_max = event.ydata + new_height * (1 - rel_y)

        ax.set_xlim(new_x_min, new_x_max)
        ax.set_ylim(new_y_min, new_y_max)

        # Perbarui anotasi teks dan plot global plan
        update_plot_elements()

    # Hubungkan fungsi zoom dengan scroll event
    fig.canvas.mpl_connect('scroll_event', on_scroll)

    # Tampilkan teks dan jalur global plan pertama kali
    update_plot_elements()
    plt.show()

# Path file rosbag
bag_file = "nv_data2.bag"

# Baca data OccupancyGrid dari rosbag
costmap_data, width, height, resolution, origin_x, origin_y = read_costmap_from_bag(bag_file)

# Baca global plan dari rosbag
global_plan_positions = read_global_plan_from_bag(bag_file)

# Plot peta dengan zoom interaktif dan nilai cost di setiap grid cell, serta jalur global plan
plot_costmap_with_global_plan(costmap_data, width, height, resolution, origin_x, origin_y, global_plan_positions, start_x=0, start_y=0)
'''import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

# Fungsi untuk mengambil data local_plan berdasarkan durasi segmentasi
def get_segmented_local_plan_from_bag(bag_file, segment_duration_sec):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)
    
    # Menentukan waktu awal rosbag untuk perhitungan durasi
    start_time_bag = bag.get_start_time()

    # List untuk menyimpan pose dari setiap segmen local plan
    segmented_trajectories = []
    current_segment = []
    current_segment_end_time = start_time_bag + segment_duration_sec

    for topic, msg, t in bag.read_messages(topics=['/move_base/DWAPlannerROS/local_plan']):
        # Hitung offset waktu tiap pesan
        time_from_start = t.to_sec() - start_time_bag

        # Jika waktu pesan melewati batas segmen saat ini
        if t.to_sec() > current_segment_end_time:
            # Simpan segmen dan mulai segmen baru
            segmented_trajectories.append(current_segment)
            current_segment = []
            current_segment_end_time += segment_duration_sec
        
        # Tambahkan data pose ke segmen saat ini
        trajectory = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        current_segment.append(trajectory)
    
    # Tambahkan segmen terakhir jika tidak kosong
    if current_segment:
        segmented_trajectories.append(current_segment)

    bag.close()
    return segmented_trajectories

# Fungsi untuk plot local plan per segmen
def plot_segmented_local_plan(segmented_trajectories, segment_duration_sec):
    for i, trajectories in enumerate(segmented_trajectories):
        plt.figure()
        for trajectory in trajectories:
            xs, ys = zip(*trajectory)
            plt.plot(xs, ys)
        
        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.title(f"Local Plans - Segment {i+1} (Duration: {segment_duration_sec} seconds)")
        plt.grid()
        plt.show()

# Input file bag dan durasi segmentasi dalam detik
bag_file = 'nv_data3.bag'
segment_duration_sec = 100  # Durasi per segmen dalam detik

# Ambil dan plot local plan per segmen
segmented_trajectories = get_segmented_local_plan_from_bag(bag_file, segment_duration_sec)
plot_segmented_local_plan(segmented_trajectories, segment_duration_sec)'''


