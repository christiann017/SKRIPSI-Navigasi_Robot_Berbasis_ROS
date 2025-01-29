import rosbag
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import matplotlib.pyplot as plt

def world_to_map(x, y, origin_x, origin_y, resolution):
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)
    return grid_x, grid_y

def transform_scan(pose, scan):
    x0, y0, theta0 = pose
    ranges, angles = scan
    transformed_points = []
    
    for r, alpha in zip(ranges, angles):
        if r < np.inf:  # Hanya pertimbangkan titik yang valid
            x = x0 + r * math.cos(theta0 + alpha)
            y = y0 + r * math.sin(theta0 + alpha)
            transformed_points.append((x, y))
    
    return transformed_points

def evaluate_match(map_data, transformed_points, origin, resolution):
    height, width = map_data.shape
    matches = 0
    total_points = len(transformed_points)
    matched_coords = []
    unmatched_coords = []  # List untuk titik yang tidak cocok

    for (x, y) in transformed_points:
        grid_x, grid_y = world_to_map(x, y, origin[0], origin[1], resolution)
        if 0 <= grid_x < width and 0 <= grid_y < height:
            cell_value = map_data[grid_y, grid_x]
            if cell_value == -1:
                unmatched_coords.append((x, y))  # Lingkungan tidak diketahui
            elif cell_value > 0:  # Halangan
                matches += 1
                matched_coords.append((x, y))
            elif cell_value <= 0:  # Bebas halangan
                unmatched_coords.append((x, y))  # Tidak cocok dengan halangan

    print(f"matches: {matches}, total_points: {total_points}")
    
    # Print koordinat yang cocok
    print("Koordinat yang cocok:")
    formatted_coords = [f"({x:.5f}, {y:.5f})" for x, y in matched_coords]
    print(", ".join(formatted_coords))
    
    return matches / total_points if total_points > 0 else 0, matched_coords, unmatched_coords
    
# Buka file rosbag
bag = rosbag.Bag('sl_data.bag')

scan_data = None
map_data = None

# Target timestamp dalam detik
target_timestamp = 307.715  # Ubah ini sesuai kebutuhan Anda

# Ekstrak data /scan dan /map
for topic, msg, t in bag.read_messages(topics=['/scan', '/map']):
    if topic == '/scan':
       msg_timestamp = msg.header.stamp.to_sec()
       if abs(msg_timestamp - target_timestamp) < 0.01:
          ranges = np.array(msg.ranges)
          angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
          scan_data = (ranges, angles)
    elif topic == '/map' and map_data is None:
        map_info = msg.info
        map_data = np.array(msg.data).reshape((map_info.height, map_info.width))
        map_resolution = map_info.resolution
        map_origin = (map_info.origin.position.x, map_info.origin.position.y)

bag.close()

if scan_data is None:
    print("Data /scan tidak ditemukan dalam rentang timestamp yang ditentukan.")
else:
    # Misalkan hipotesis pose adalah (2.5, 1.5, 0)
    hypothesis_pose = (-0.1114905,  -0.4991213, 0.0076996)

    # Transformasikan titik scan
    transformed_points = transform_scan(hypothesis_pose, scan_data)

    # Evaluasi kecocokan
    match_score, matched_coords, unmatched_coords = evaluate_match(map_data, transformed_points, map_origin, map_resolution)
    print(f"Kecocokan transformasi: {match_score:.5f}")

    # Plot semua titik scan dan koordinat yang cocok dan tidak cocok
    if transformed_points:
        transformed_points = np.array(transformed_points)
        plt.figure(figsize=(8, 8))

        # Plot semua titik dari data scan (hijau)
        plt.plot(transformed_points[:, 0], transformed_points[:, 1], 'go', label='All Scan Points')

        # Plot titik yang cocok (biru)
        if matched_coords:
            matched_coords = np.array(matched_coords)
            plt.plot(matched_coords[:, 0], matched_coords[:, 1], 'bo', label='Matched Points')

        # Plot titik yang tidak cocok (kuning)
        if unmatched_coords:
            unmatched_coords = np.array(unmatched_coords)
            plt.plot(unmatched_coords[:, 0], unmatched_coords[:, 1], 'yo', label='Unmatched Points')

        # Plot posisi sensor (merah)
        plt.plot(hypothesis_pose[0], hypothesis_pose[1], 'ro', label='Sensor Position')

        # Setting tampilan
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Scan, Matched and Unmatched Points')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()
    else:
        print("Tidak ada titik scan yang ditemukan.")

