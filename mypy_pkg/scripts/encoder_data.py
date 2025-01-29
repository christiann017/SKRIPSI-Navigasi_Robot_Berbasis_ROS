import rosbag
import pandas as pd
from openpyxl import Workbook
from openpyxl.utils.dataframe import dataframe_to_rows
from openpyxl.worksheet.table import Table, TableStyleInfo

def extract_joint_states_data(bagfile, joint_states_topic):
    bag = rosbag.Bag(bagfile)
    joint_states_data = []
    last_time = None  # Variabel untuk menyimpan detik terakhir yang diproses

    for topic, msg, t in bag.read_messages(topics=[joint_states_topic]):
        time = int(t.to_sec())  # Convert time to integer (seconds)

        # Check if this second has already been processed
        if time != last_time:
            # Assuming the names of the wheel joints are 'left_wheel_joint' and 'right_wheel_joint'
            left_wheel_index = msg.name.index('wheel_left_joint')
            right_wheel_index = msg.name.index('wheel_right_joint')

            # Append data only for the first message in each second
            joint_states_data.append([
                time,
                msg.position[right_wheel_index],
                msg.position[left_wheel_index]
            ])
            last_time = time  # Update last_time to the current second

    bag.close()

    # Create a pandas DataFrame
    df = pd.DataFrame(joint_states_data, columns=[
        'Time', 
        'Right Joint State Position',
        'Left Joint State Position'
    ])
    return df

def save_joint_states_data_to_excel_with_filter(df, filename):
    # Create a new Excel workbook and worksheet
    wb = Workbook()
    ws = wb.active
    ws.title = "Joint States Data"

    # Write the DataFrame to the worksheet
    for r in dataframe_to_rows(df, index=False, header=True):
        ws.append(r)

    # Define the table range and add the table with filtering
    table = Table(displayName="JointStatesTable", ref=f"A1:C{len(df) + 1}")
    style = TableStyleInfo(
        name="TableStyleMedium9",
        showFirstColumn=False,
        showLastColumn=False,
        showRowStripes=True,
        showColumnStripes=True
    )
    table.tableStyleInfo = style
    ws.add_table(table)

    # Save the workbook
    wb.save(filename)
    print(f"Data has been saved to {filename}")

if __name__ == "__main__":
    bagfile = 'sl_data.bag'  # Ganti dengan path ke file rosbag Anda
    joint_states_topic = '/joint_states'  # Ganti dengan nama topik joint_states Anda

    df = extract_joint_states_data(bagfile, joint_states_topic)
    save_joint_states_data_to_excel_with_filter(df, 'joint_states_data_filtered2.xlsx')

