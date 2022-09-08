#!/usr/bin/env python3

from re import ASCII
import rosbag

rosbag_file = "../data/kitti_2011_09_30_drive_0016_synced_correct.bag"
output_file = "../data/data.txt"

lidar_topic = "/points_raw"
imu_topic   = "/imu_correct"

bag = rosbag.Bag(rosbag_file)

def extract_lidar_data(msg):
    res = {}
    res['msg_type'] = 'lidar'
    res['header'] = [msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, msg.header.frame_id]
    res['height'] = msg.height
    res['width'] = msg.width
    res['fields_num'] = len(msg.fields)

    for point_field in msg.fields:
        res['f:'+point_field.name] = [point_field.name, point_field.offset, point_field.datatype, point_field.count]

    res['is_bigendian'] = 1 if msg.is_bigendian else 0
    res['point_step'] = msg.point_step
    res['row_step'] = msg.row_step
    res['data'] = msg.data
    res['is_dense'] = 1 if msg.is_dense else 0

    return res

def extract_imu_data(msg):
    res = {}
    res['msg_type'] = 'imu'
    res['header'] = [msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, msg.header.frame_id]
    res['orientation'] = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    res['orientation_covariance'] = msg.orientation_covariance
    res['angular_velocity'] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    res['angular_velocity_covariance'] = msg.angular_velocity_covariance
    res['linear_acceleration'] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
    res['linear_acceleration_covariance'] = msg.linear_acceleration_covariance

    return res

with open(output_file, 'w') as file:
    for topic, msg, t in bag.read_messages(topics=[lidar_topic, imu_topic]):
        # print(topic + ': ' + str(t))

        if topic == lidar_topic:
            data_dict = extract_lidar_data(msg)
        elif topic == imu_topic:
            data_dict = extract_imu_data(msg)

        for key, value in data_dict.items():
            if key == 'data':
                values = []
                for val in value:
                    values.append(val)
                file.write(str(values).strip('[]()').replace(",", "")+ '\n')
            else:
                file.write(str(value).strip('[]()').replace(",", "") + '\n')
            # file.write("{: <20}: {: >0} \n".format(str(key), str(value).strip('[]()')))
        # if topic == lidar_topic:
        #     break
    file.write('---\n')

bag.close()