#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(data):
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution
    origin = data.info.origin
    map_data = data.data

    # Konversi data peta menjadi matriks 2D
    map_2d = []
    for i in range(height):
        row = []
        for j in range(width):
            row.append(map_data[i * width + j])
        map_2d.append(row)
    
    # Simpan data peta ke file teks
    with open("map_data.txt", "w") as f:
        f.write("Width: {}\n".format(width))
        f.write("Height: {}\n".format(height))
        f.write("Resolution: {}\n".format(resolution))
        f.write("Origin: ({}, {}, {})\n".format(origin.position.x, origin.position.y, origin.position.z))
        f.write("Map Data:\n")
        for row in map_2d:
            f.write(" ".join(map(str, row)) + "\n")

    rospy.loginfo("Data peta disimpan")

def listener():
    rospy.init_node('map_listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

