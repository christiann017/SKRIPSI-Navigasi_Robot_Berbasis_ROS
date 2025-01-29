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

def extract_particlecloud_data_at_precise_time(bagfile, particlecloud_topic, target_time, tolerance=0.01):
    """
    Ekstrak data partikel pada waktu tertentu dengan presisi tinggi.
    target_time: Waktu dalam detik (float) di mana data partikel ingin diambil.
    tolerance: Batas toleransi untuk memilih waktu partikel yang dekat dengan target_time.
    """
    bag = rosbag.Bag(bagfile)
    particlecloud_data = []

    for topic, msg, t in bag.read_messages(topics=[particlecloud_topic]):
        time = t.to_sec()
        
        # Jika waktu dalam toleransi (±tolerance detik) dari target_time
        if abs(time - target_time) > tolerance:
            continue  # Lewati jika waktu tidak dalam toleransi

        num_particles = len(msg.poses)
        
        if num_particles == 0:
            continue

        # Inisialisasi variabel untuk menghitung sum dari posisi dan orientasi (yaw)
        sum_x = 0
        sum_y = 0
        sum_yaw = 0
        
        x_values = []
        y_values = []
        yaw_values = []

        # Iterasi melalui setiap partikel di PoseArray
        for pose in msg.poses:
            position = pose.position
            orientation = pose.orientation
            
            # Konversi quaternion ke yaw
            roll, pitch, yaw = quaternion_to_euler(orientation)
            
            # Simpan posisi dan yaw untuk perhitungan varians
            x_values.append(position.x)
            y_values.append(position.y)
            yaw_values.append(yaw)
            
            # Hitung sum untuk mean
            sum_x += position.x
            sum_y += position.y
            sum_yaw += yaw

        # Hitung mean posisi (x, y) dan orientasi (yaw)
        mean_x = sum_x / num_particles
        mean_y = sum_y / num_particles
        mean_yaw = sum_yaw / num_particles
        
        # Hitung varians posisi (x, y) dan orientasi (yaw)
        var_x = sum((x - mean_x) ** 2 for x in x_values) / num_particles
        var_y = sum((y - mean_y) ** 2 for y in y_values) / num_particles
        var_yaw = sum((yaw - mean_yaw) ** 2 for yaw in yaw_values) / num_particles
        
        # Simpan data waktu, mean, dan varians
        particlecloud_data.append([time, mean_x, mean_y, mean_yaw, var_x, var_y, var_yaw])

    bag.close()

    # Membuat pandas DataFrame untuk menyimpan hasil mean dan varians
    df = pd.DataFrame(particlecloud_data, columns=[
        'Time', 
        'MeanPositionX', 
        'MeanPositionY', 
        'MeanYaw', 
        'VariancePositionX', 
        'VariancePositionY', 
        'VarianceYaw'
    ])
    return df

def save_mean_variance_particlecloud_data_to_text(df, filename):
    with open(filename, 'w') as f:
        for index, row in df.iterrows():
            f.write(f"Time: {row['Time']}\n")
            f.write(f"  Mean Position - x: {row['MeanPositionX']}, y: {row['MeanPositionY']}\n")
            f.write(f"  Mean Orientation (radian) - yaw: {row['MeanYaw']}\n")
            f.write(f"  Variance Position - x: {row['VariancePositionX']}, y: {row['VariancePositionY']}\n")
            f.write(f"  Variance Orientation (radian) - yaw: {row['VarianceYaw']}\n")
            f.write("\n")
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'nv_data.bag'  # Ganti dengan path ke file rosbag Anda
    particlecloud_topic = '/particlecloud'  # Ganti dengan nama topik particlecloud Anda

    # Waktu spesifik (dalam detik) yang diinginkan dengan presisi tinggi
    target_time = 47.189  # Ganti dengan waktu detik yang Anda inginkan

    # Ekstrak data /particlecloud dari rosbag pada waktu spesifik
    df = extract_particlecloud_data_at_precise_time(bagfile, particlecloud_topic, target_time)
    
    # Simpan hasil mean dan varians posisi serta yaw ke file teks
    save_mean_variance_particlecloud_data_to_text(df, '47.189_mean_variance_particlecloud_data.txt')

