import rosbag
import pandas as pd
import tf

def quaternion_to_euler(quaternion):
    """
    Konversi quaternion ke sudut Euler (roll, pitch, yaw)
    """
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)
    return roll, pitch, yaw

def extract_particlecloud_data(bagfile, particlecloud_topic):
    bag = rosbag.Bag(bagfile)
    particlecloud_data = []

    for topic, msg, t in bag.read_messages(topics=[particlecloud_topic]):
        time = t.to_sec()

        # Iterasi melalui setiap partikel di PoseArray
        for i, pose in enumerate(msg.poses):
            position = pose.position
            orientation = pose.orientation
            roll, pitch, yaw = quaternion_to_euler(orientation)

            particlecloud_data.append([
                time,
                i + 1,  # Nomor partikel (indeks 1-based)
                position.x,
                position.y,
                position.z,
                yaw
            ])

    bag.close()

    # Membuat pandas DataFrame
    df = pd.DataFrame(particlecloud_data, columns=[
        'Time', 
        'ParticleIndex',
        'PositionX', 
        'PositionY', 
        'PositionZ',
        'Yaw'
    ])
    return df

def save_particlecloud_data_to_text(df, filename):
    with open(filename, 'w') as f:
        # Ambil daftar waktu unik
        unique_times = df['Time'].unique()
        
        for time in unique_times:
            f.write(f"Time: {time}\n")
            # Ambil semua partikel untuk waktu ini
            particles_at_time = df[df['Time'] == time]
            
            for index, row in particles_at_time.iterrows():
                f.write(f"Particle {int(row['ParticleIndex'])}:\n")
                f.write(f"  Position - x: {row['PositionX']}, y: {row['PositionY']}, z: {row['Yaw']}\n")
            f.write("\n")
    
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'nv_data.bag'  # Ganti dengan path ke file rosbag Anda
    particlecloud_topic = '/particlecloud'  # Ganti dengan nama topik particlecloud Anda

    df = extract_particlecloud_data(bagfile, particlecloud_topic)
    save_particlecloud_data_to_text(df, 'particlecloud_data.txt')

