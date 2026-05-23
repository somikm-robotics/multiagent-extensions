import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import csv

bag_path = "bags/dust_sensor_readings_bag"
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

with open("pm_readings.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["time", "pm_message"])
    msg_type = get_message('std_msgs/msg/String')
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/air_quality/pm":
            msg = deserialize_message(data, msg_type)
            writer.writerow([t, msg.data])
