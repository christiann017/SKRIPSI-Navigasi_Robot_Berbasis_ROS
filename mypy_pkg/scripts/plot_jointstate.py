import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Buka file rosbag
bag = rosbag.Bag('sl_data.bag')

# Inisialisasi dictionary untuk menyimpan data joint_states
joint_data = {}

# Baca pesan dari topik /joint_states
for topic, msg, t in bag.read_messages(topics=['/joint_states']):
    for i, name in enumerate(msg.name):
        if name not in joint_data:
            joint_data[name] = {'time': [], 'position': []}
        joint_data[name]['time'].append(t.to_sec())
        joint_data[name]['position'].append(msg.position[i])


bag.close()

# Plot data joint_states untuk setiap joint
for joint, data in joint_data.items():
    time = np.array(data['time'])
    position = np.array(data['position'])

    
    plt.figure()
    
    plt.subplot(3, 1.5, 2)
    plt.plot(time, position, label='Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title(f'Joint: {joint} - Position')
    

    
    plt.tight_layout()
    plt.show()

