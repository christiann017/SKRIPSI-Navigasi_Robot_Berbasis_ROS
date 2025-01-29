import rosbag
import pandas as pd
from collections import defaultdict

def extract_cmd_vel_data_per_second(bagfile, cmd_vel_topic):
    """
    Ekstrak data kecepatan dari topik /cmd_vel dan menyimpannya dalam satu data per detik.
    """
    bag = rosbag.Bag(bagfile)
    cmd_vel_data = defaultdict(list)

    # Baca semua pesan dari topik /cmd_vel
    for topic, msg, t in bag.read_messages(topics=[cmd_vel_topic]):
        time = int(t.to_sec())  # Membulatkan waktu ke detik (tanpa desimal)
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Simpan hanya satu data per detik
        if time not in cmd_vel_data:
            cmd_vel_data[time] = [linear_x, angular_z]

    bag.close()
    
    # Mengubah dictionary menjadi list untuk membuat DataFrame
    cmd_vel_data_list = [[time, data[0], data[1]] for time, data in sorted(cmd_vel_data.items())]
    
    return cmd_vel_data_list

def save_cmd_vel_data_to_excel(cmd_vel_data, filename):
    """
    Simpan data /cmd_vel ke file Excel dengan kolom terfilter.
    """
    # Buat DataFrame dengan kolom waktu, kecepatan linear (x), dan kecepatan angular (z)
    df = pd.DataFrame(cmd_vel_data, columns=['Time', 'LinearVelocityX', 'AngularVelocityZ'])
    
    # Simpan ke file Excel
    df.to_excel(filename, index=False)
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'sl10_data.bag'  # Ganti dengan path ke file rosbag Anda
    cmd_vel_topic = '/cmd_vel'  # Ganti dengan topik cmd_vel Anda
    
    # Ekstrak data /cmd_vel dari rosbag
    cmd_vel_data = extract_cmd_vel_data_per_second(bagfile, cmd_vel_topic)
    
    # Simpan data ke file Excel
    save_cmd_vel_data_to_excel(cmd_vel_data, 'cmd_vel_data_per_second.xlsx')

