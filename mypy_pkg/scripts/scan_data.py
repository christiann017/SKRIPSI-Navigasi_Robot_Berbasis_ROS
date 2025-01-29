#!/usr/bin/env python3

import rosbag
import pandas as pd

# Gantilah ini dengan nama file rosbag Anda
bag_file = 'sl_data.bag'
# Gantilah ini dengan nama topik yang ingin Anda ekstrak
topic_name = '/scan'
# Nama file Excel output
output_file = 'scan_data.xlsx'

def save_range_data_to_excel(bag_file, topic_name, output_file):
    try:
        # List untuk menyimpan data
        data = []

        # Membaca rosbag
        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                if hasattr(msg, 'ranges'):
                    timestamp = t.to_sec()
                    
                    # Memfilter berdasarkan timestamp
                    if timestamp == 118.617 or timestamp == 307.715:
                        # Membatasi angka di belakang koma menjadi 5 angka
                        ranges = [round(r, 5) for r in msg.ranges]
                        data.append([timestamp, ranges])
        
        # Membuat DataFrame pandas
        df = pd.DataFrame(data, columns=["Timestamp", "Ranges"])
        
        # Menyimpan ke file Excel
        df.to_excel(output_file, index=False)

        print(f"Data has been successfully saved to {output_file}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    save_range_data_to_excel(bag_file, topic_name, output_file)

