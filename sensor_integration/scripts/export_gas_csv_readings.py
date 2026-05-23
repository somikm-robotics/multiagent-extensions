import sys
import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import csv

def export_pm_csv(bag_name):
    # Build bag and output paths dynamically
    base_dir = os.path.expanduser("~/ros2_ws/experiments/bags")
    bag_path = os.path.join(base_dir, bag_name)
    out_dir = os.path.expanduser("~/ros2_ws/experiments/data")
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, f"{bag_name}.csv")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    msg_type = get_message('std_msgs/msg/String')

    with open(out_file, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["time", "voc_message"])
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic == "/air_quality/voc":
                msg = deserialize_message(data, msg_type)
                writer.writerow([t, msg.data])

    print(f"✅ Export complete: {out_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 export_gas_csv_readings.py <bag_name>")
        sys.exit(1)
    export_pm_csv(sys.argv[1])
