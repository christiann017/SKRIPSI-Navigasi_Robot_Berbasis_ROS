'''import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

def plot_map_with_arrows(bag_file, particle_data):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)

    # Inisialisasi variabel untuk menyimpan data peta
    map_data = None

    # Baca pesan dari topik /map dan ambil yang pertama
    for topic, msg, t in bag.read_messages(topics=['/map']):
        map_data = msg
        break  # Berhenti jika menemukan data peta

    bag.close()

    if map_data is None:
        print("No map data found.")
        return

    # Konversi data peta ke numpy array
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    map_array = np.array(map_data.data).reshape((height, width))

    # Plot peta menggunakan matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray', origin='lower',
               extent=[origin_x, origin_x + width * resolution,
                       origin_y, origin_y + height * resolution])
    plt.colorbar(label='Occupancy Probability')

    # Menampilkan semua titik grid dengan warna hijau
    for y in range(height):
        for x in range(width):
            real_x = origin_x + x * resolution
            real_y = origin_y + y * resolution
            plt.scatter(real_x, real_y, color='green', s=1)  # Titik grid berwarna hijau

    # Menghitung dan menggambar anak panah untuk setiap partikel
    arrow_length = 0.16  # Panjang anak panah
    for x_pos, y_pos, orientation in particle_data:
        # Hitung koordinat anak panah
        arrow_x = arrow_length * np.cos(orientation)  # Koordinat x anak panah
        arrow_y = arrow_length * np.sin(orientation)  # Koordinat y anak panah

        # Plot anak panah di posisi (x_pos, y_pos)
        plt.arrow(x_pos, y_pos, arrow_x, arrow_y, head_width=0.1, head_length=0.1, fc='red', ec='red')

    # Konfigurasi plot
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Occupancy Grid Map with Orientation Arrows')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Contoh penggunaan
bag_file = 'nv_data.bag'  # Ganti dengan nama file rosbag Anda

# Daftar partikel, masing-masing dengan posisi (x, y) dan orientasi dalam radian
particle_data = [(0.05393970746874614, 0.10118628866645957, -0.0038382868398422125),(-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (0.19643273801059585, 0.25069861646029473, 0.7617138347893229), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (0.05393970746874614, 0.10118628866645957, -0.0038382868398422125), (0.01775031155827076, 0.4868092277631639, 0.08486999569103429), (-0.03307012600148146, -0.12581327295501296, 0.04145123241828018), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (-0.48284965367128224, -0.8975720398479596, -0.11069665794490266), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (0.32087357926060384, 0.637975297274069, -0.16355015711576673)]  # Ganti dengan data partikel yang diinginkan

plot_map_with_arrows(bag_file, particle_data)
'''
'''import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

def plot_map_with_arrows(bag_file, particle_data):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)

    # Inisialisasi variabel untuk menyimpan data peta
    map_data = None

    # Baca pesan dari topik /map dan ambil yang pertama
    for topic, msg, t in bag.read_messages(topics=['/map']):
        map_data = msg
        break  # Berhenti jika menemukan data peta

    bag.close()

    if map_data is None:
        print("No map data found.")
        return

    # Konversi data peta ke numpy array
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    map_array = np.array(map_data.data).reshape((height, width))

    # Plot peta menggunakan matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray', origin='lower',
               extent=[origin_x, origin_x + width * resolution,
                       origin_y, origin_y + height * resolution])
    plt.colorbar(label='Occupancy Probability')

    # Menampilkan semua titik grid dengan warna hijau
    for y in range(height):
        for x in range(width):
            real_x = origin_x + x * resolution
            real_y = origin_y + y * resolution
            plt.scatter(real_x, real_y, color='green', s=1)  # Titik grid berwarna hijau

    # Menghitung dan menggambar dua anak panah untuk setiap partikel
    arrow_length = 0.5  # Panjang anak panah
    for x_pos, y_pos, orientation in particle_data:
        # Hitung koordinat anak panah untuk orientasi asli
        arrow_x = arrow_length * np.cos(orientation)  # Koordinat x anak panah asli
        arrow_y = arrow_length * np.sin(orientation)  # Koordinat y anak panah asli

        # Hitung koordinat anak panah untuk orientasi berlawanan
        reverse_arrow_x = arrow_length * np.cos(orientation + np.pi)  # Arah berlawanan (tambah π radian)
        reverse_arrow_y = arrow_length * np.sin(orientation + np.pi)

        # Plot anak panah asli dengan warna merah
        plt.arrow(x_pos, y_pos, arrow_x, arrow_y, head_width=0.1, head_length=0.1, fc='red', ec='red')

        # Plot anak panah berlawanan arah dengan warna biru
        plt.arrow(x_pos, y_pos, reverse_arrow_x, reverse_arrow_y, head_width=0.1, head_length=0.1, fc='blue', ec='blue')

    # Konfigurasi plot
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Occupancy Grid Map with Orientation Arrows and Reverse Arrows')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Contoh penggunaan
bag_file = 'nv_data.bag'  # Ganti dengan nama file rosbag Anda

# Daftar partikel, masing-masing dengan posisi (x, y) dan orientasi dalam radian
particle_data = [(0.05393970746874614, 0.10118628866645957, -0.0038382868398422125),(-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (0.19643273801059585, 0.25069861646029473, 0.7617138347893229), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (0.05393970746874614, 0.10118628866645957, -0.0038382868398422125), (0.01775031155827076, 0.4868092277631639, 0.08486999569103429), (-0.03307012600148146, -0.12581327295501296, 0.04145123241828018), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (-0.48284965367128224, -0.8975720398479596, -0.11069665794490266), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (0.32087357926060384, 0.637975297274069, -0.16355015711576673)]  # Ganti dengan data partikel yang diinginkan

plot_map_with_arrows(bag_file, particle_data)'''
'''import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid

def plot_map_and_calculate_distance(bag_file, particle_data):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)

    # Inisialisasi variabel untuk menyimpan data peta
    map_data = None

    # Baca pesan dari topik /map dan ambil yang pertama
    for topic, msg, t in bag.read_messages(topics=['/map']):
        map_data = msg
        break  # Berhenti jika menemukan data peta

    bag.close()

    if map_data is None:
        print("No map data found.")
        return

    # Konversi data peta ke numpy array
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    map_array = np.array(map_data.data).reshape((height, width))

    # Plot peta menggunakan matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray', origin='lower',
               extent=[origin_x, origin_x + width * resolution,
                       origin_y, origin_y + height * resolution])
    plt.colorbar(label='Occupancy Probability')

    # Menampilkan semua titik grid dengan warna hijau
    for y in range(height):
        for x in range(width):
            real_x = origin_x + x * resolution
            real_y = origin_y + y * resolution
            plt.scatter(real_x, real_y, color='green', s=5)  # Titik grid berwarna hijau

    # Menghitung dan menggambar dua anak panah untuk setiap partikel
    arrow_length = 0.5  # Panjang anak panah
    for idx, (x_pos, y_pos, orientation) in enumerate(particle_data):
        # Hitung koordinat anak panah untuk orientasi asli
        arrow_x = arrow_length * np.cos(orientation)  # Koordinat x anak panah asli
        arrow_y = arrow_length * np.sin(orientation)  # Koordinat y anak panah asli

        # Hitung koordinat anak panah untuk orientasi berlawanan
        reverse_orientation = orientation + np.pi  # Arah berlawanan (tambah π radian)
        reverse_arrow_x = arrow_length * np.cos(reverse_orientation)
        reverse_arrow_y = arrow_length * np.sin(reverse_orientation)

        # Plot anak panah asli dengan warna merah
        plt.arrow(x_pos, y_pos, arrow_x, arrow_y, head_width=0.1, head_length=0.1, fc='red', ec='red')

        # Plot anak panah berlawanan arah dengan warna biru
        plt.arrow(x_pos, y_pos, reverse_arrow_x, reverse_arrow_y, head_width=0.1, head_length=0.1, fc='blue', ec='blue')

        # Hitung jarak ke dinding di arah berlawanan
        distance_to_wall = calculate_distance_to_wall(map_array, x_pos, y_pos, reverse_orientation, origin_x, origin_y, resolution)
        print(f'Particle {idx+1}: Distance to wall in reverse direction = {distance_to_wall:.8f} meters')

    # Konfigurasi plot
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Occupancy Grid Map with Orientation Arrows and Reverse Arrows')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def calculate_distance_to_wall(map_array, x_init, y_init, orientation, origin_x, origin_y, resolution):
    """
    Menghitung jarak dari titik awal ke dinding di arah orientasi yang diberikan.
    """
    max_distance = 3.5  # Batasi jarak maksimum yang dihitung
    distance_step = 0.05   # Langkah jarak setiap iterasi (meter)

    x_current = x_init
    y_current = y_init

    for _ in np.arange(0, max_distance, distance_step):
        # Update posisi berdasarkan orientasi
        x_current += np.cos(orientation) * distance_step
        y_current += np.sin(orientation) * distance_step

        # Konversi posisi real ke indeks grid
        grid_x = int((x_current - origin_x) / resolution)
        grid_y = int((y_current - origin_y) / resolution)

        # Pastikan indeks valid
        if grid_x < 0 or grid_x >= map_array.shape[1] or grid_y < 0 or grid_y >= map_array.shape[0]:
            return max_distance  # Jika di luar peta, asumsikan tidak ada dinding (jarak maksimal)

        # Periksa apakah ada dinding (misalnya, nilai occupancy > 50)
        if map_array[grid_y, grid_x] > 50:
            return np.sqrt((x_current - x_init)**2 + (y_current - y_init)**2)

    return max_distance  # Jika tidak ditemukan dinding, kembalikan jarak maksimum

# Contoh penggunaan
bag_file = 'nv_data.bag'  # Ganti dengan nama file rosbag Anda

# Daftar partikel, masing-masing dengan posisi (x, y) dan orientasi dalam radian
particle_data = [(0.0000013, 0.0000179, 0.00011116), (0.05393970746874614, 0.10118628866645957, -0.0038382868398422125), (-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (0.19643273801059585, 0.25069861646029473, 0.7617138347893229), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (0.05393970746874614, 0.10118628866645957, -0.0038382868398422125), (0.01775031155827076, 0.4868092277631639, 0.08486999569103429), (-0.03307012600148146, -0.12581327295501296, 0.04145123241828018), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (-0.48284965367128224, -0.8975720398479596, -0.11069665794490266), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (0.32087357926060384, 0.637975297274069, -0.16355015711576673)]  # Ganti dengan data partikel yang diinginkan

plot_map_and_calculate_distance(bag_file, particle_data)'''
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import matplotlib.cm as cm

def plot_map_and_calculate_distance(bag_file, particle_data):
    # Buka file rosbag
    bag = rosbag.Bag(bag_file)

    # Inisialisasi variabel untuk menyimpan data peta
    map_data = None

    # Baca pesan dari topik /map dan ambil yang pertama
    for topic, msg, t in bag.read_messages(topics=['/map']):
        map_data = msg
        break  # Berhenti jika menemukan data peta

    bag.close()

    if map_data is None:
        print("No map data found.")
        return

    # Konversi data peta ke numpy array
    width = map_data.info.width
    height = map_data.info.height
    resolution = map_data.info.resolution
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y

    map_array = np.array(map_data.data).reshape((height, width))

    # Plot peta menggunakan matplotlib
    plt.figure(figsize=(10, 10))
    plt.imshow(map_array, cmap='gray', origin='lower',
               extent=[origin_x, origin_x + width * resolution,
                       origin_y, origin_y + height * resolution])
    plt.colorbar(label='Occupancy Probability')

    # Menampilkan semua titik grid dengan warna hijau
    for y in range(height):
        for x in range(width):
            real_x = origin_x + x * resolution
            real_y = origin_y + y * resolution
            plt.scatter(real_x, real_y, color='green', s=2)  # Titik grid berwarna hijau

    # Menghitung dan menggambar dua anak panah untuk setiap partikel
    arrow_length = 0.5  # Panjang anak panah
    cmap = cm.get_cmap('tab20', len(particle_data))  # Gunakan cmap dengan warna berbeda
    for idx, (x_pos, y_pos, orientation) in enumerate(particle_data):
        # Pilih warna berbeda untuk setiap partikel
        color = cmap(idx)

        # Hitung koordinat anak panah untuk orientasi asli
        arrow_x = arrow_length * np.cos(orientation)  # Koordinat x anak panah asli
        arrow_y = arrow_length * np.sin(orientation)  # Koordinat y anak panah asli

        # Hitung koordinat anak panah untuk orientasi berlawanan
        reverse_orientation = orientation + np.pi  # Arah berlawanan (tambah π radian)
        reverse_arrow_x = arrow_length * np.cos(reverse_orientation)
        reverse_arrow_y = arrow_length * np.sin(reverse_orientation)

        # Plot anak panah asli dengan warna merah
        plt.arrow(x_pos, y_pos, arrow_x, arrow_y, head_width=0.1, head_length=0.1, fc='red', ec='red')

        # Plot anak panah berlawanan arah dengan warna biru
        plt.arrow(x_pos, y_pos, reverse_arrow_x, reverse_arrow_y, head_width=0.1, head_length=0.1, fc='blue', ec='blue')

        # Plot nomor partikel dengan warna berbeda
        plt.text(x_pos + 0.1, y_pos + 0.1, f'{idx+1}', fontsize=20, color=color)

        # Hitung jarak ke dinding di arah berlawanan
        distance_to_wall = calculate_distance_to_wall(map_array, x_pos, y_pos, reverse_orientation, origin_x, origin_y, resolution)
        print(f'Particle {idx+1}: Distance to wall in reverse direction = {distance_to_wall:.2f} meters')

    # Konfigurasi plot
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Occupancy Grid Map with Orientation Arrows and Particle Numbers')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def calculate_distance_to_wall(map_array, x_init, y_init, orientation, origin_x, origin_y, resolution):
    """
    Menghitung jarak dari titik awal ke dinding di arah orientasi yang diberikan.
    """
    max_distance = 3.5  # Batasi jarak maksimum yang dihitung
    distance_step = 0.05   # Langkah jarak setiap iterasi (meter)

    x_current = x_init
    y_current = y_init

    for _ in np.arange(0, max_distance, distance_step):
        # Update posisi berdasarkan orientasi
        x_current += np.cos(orientation) * distance_step
        y_current += np.sin(orientation) * distance_step

        # Konversi posisi real ke indeks grid
        grid_x = int((x_current - origin_x) / resolution)
        grid_y = int((y_current - origin_y) / resolution)

        # Pastikan indeks valid
        if grid_x < 0 or grid_x >= map_array.shape[1] or grid_y < 0 or grid_y >= map_array.shape[0]:
            return max_distance  # Jika di luar peta, asumsikan tidak ada dinding (jarak maksimal)

        # Periksa apakah ada dinding (misalnya, nilai occupancy > 50)
        if map_array[grid_y, grid_x] > 50:
            return np.sqrt((x_current - x_init)**2 + (y_current - y_init)**2)

    return max_distance  # Jika tidak ditemukan dinding, kembalikan jarak maksimum

# Contoh penggunaan
bag_file = 'nv_data.bag'  # Ganti dengan nama file rosbag Anda

# Daftar partikel, masing-masing dengan posisi (x, y) dan orientasi dalam radian
particle_data = [(0.0000013, 0.0000179, 0.00011116), (0.05393970746874614, 0.10118628866645957, -0.0038382868398422125), (-0.09841696244865149, 0.007091665333566934, -0.3292144999956623), (0.19643273801059585, 0.25069861646029473, 0.7617138347893229), (0.07249358530067358, -0.2636184727232211, -0.06631562148536746), (2, 2, 2), (0.01775031155827076, 0.4868092277631639, 0.08486999569103429), (-0.03307012600148146, -0.12581327295501296, 0.04145123241828018), (0.19014409711975108, 1.0496457947431066, -0.4600774921926942), (-0.08663866554674315, 0.13543239602563945, -0.11096158150289141), (2, 2, 2), (-0.3061151924486886, 0.5418432774488244, 0.08981631030523149), (2, 2, 2), (2, 2, 2), (2, 2, 2), (-0.48284965367128224, -0.8975720398479596, -0.11069665794490266), (2, 2, 2), (2, 2, 2), (2, 2, 2), (2, 2, 2), (0.32087357926060384, 0.637975297274069, -0.16355015711576673)] # Ganti dengan data partikel yang diinginkan

plot_map_and_calculate_distance(bag_file, particle_data)

