#!/usr/bin/env python3

import rosbag
from std_msgs.msg import Header
from livox_ros_driver.msg import CustomMsg

rosbag_file = "/media/burhan/D/all_tasks/FAST-LIO-MODIFIED/data/8_Shape_Path.bag"
output_file = "/media/burhan/D/all_tasks/FAST-LIO-MODIFIED/data/converted_odom.txt"

lidar_topic = "/livox/lidar"
imu_topic   = "/livox/imu"

bag = rosbag.Bag(rosbag_file)

def extract_lidar_data(msg):
    res = {}
    res['msg_type'] = 'lidar'
    res['header'] = msg.header.stamp.to_sec()
    res['height'] = msg.point_num
    
    res['fields'] = []
    for point_field in msg.points:
        res['fields'].append(point_field.x)
        res['fields'].append(point_field.y)
        res['fields'].append(point_field.z)
        res['fields'].append(point_field.reflectivity)
        res['fields'].append(point_field.offset_time*1e-9)

    return res

def extract_imu_data(msg):
    res = {}
    res['msg_type'] = 'imu'
    res['header'] = msg.header.stamp.to_sec()
    res['angular_velocity'] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    res['linear_acceleration'] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

    return res

with open(output_file, 'w') as file:
    for topic, msg, t in bag.read_messages(topics=[lidar_topic, imu_topic]):
        # print(topic + ': ' + str(t))

        if topic == lidar_topic:
            data_dict = extract_lidar_data(msg)
        elif topic == imu_topic:
            data_dict = extract_imu_data(msg)

        for key, value in data_dict.items():
            new_str = str(value).strip('[]()').replace(",", "").replace("\'", "")
            file.write(new_str + '\n')
            # file.write("{: <20}: {: >0} \n".format(str(key), str(value).strip('[]()')))
        # if topic == lidar_topic:
        #     break
    file.write('---\n')

bag.close()