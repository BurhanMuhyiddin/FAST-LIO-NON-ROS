from importlib.resources import path
import rosbag
import numpy as np
import pandas as pd
from std_msgs.msg import Time

BAG_PATH = "../data/8_Shape_Path.bag"
IMU_CSV_PATH = "../data/imu_8/imu.csv"
LIDAR_CSV_PATH = "../data/lidar_8/"

lidar_frame_counter = 0

imu_csv_header = ['Timesamp','Gyrox','Gyroy','Gyroz','Accx','Accy','Accz']
lidar_csv_header = ['Timestamp','Px','Py','Pz','Pi']

imu_buff = np.zeros((13976,7), dtype=np.float64)

bag = rosbag.Bag(BAG_PATH)

imu_ind = 0
lidar_ind = 0
point_line = -9999999
for topic, msg, t in bag.read_messages(topics=['/livox/imu', '/livox/lidar']):
    if topic == '/livox/imu':
        imu_buff[imu_ind, 0] = msg.header.stamp.secs*1.0 + msg.header.stamp.nsecs/1e9
        imu_buff[imu_ind, 1] = msg.angular_velocity.x
        imu_buff[imu_ind, 2] = msg.angular_velocity.y
        imu_buff[imu_ind, 3] = msg.angular_velocity.z
        imu_buff[imu_ind, 4] = msg.linear_acceleration.x
        imu_buff[imu_ind, 5] = msg.linear_acceleration.y
        imu_buff[imu_ind, 6] = msg.linear_acceleration.z
        imu_ind += 1
    elif topic == '/livox/lidar':
        lidar_buff = np.zeros((len(msg.points),5), dtype=np.float64)
        t_frame = msg.header.stamp.secs*1.0 + msg.header.stamp.nsecs/1e9
        for i, point in enumerate(msg.points):
            lidar_buff[i, 0] = t_frame + point.offset_time/1e9
            lidar_buff[i, 1] = point.x
            lidar_buff[i, 2] = point.y
            lidar_buff[i, 3] = point.z
            lidar_buff[i, 4] = point.reflectivity
        lidar_df = pd.DataFrame(lidar_buff, columns = lidar_csv_header)
        lidar_df.to_csv(LIDAR_CSV_PATH + ('0'*(6-len(str(lidar_frame_counter))) + str(lidar_frame_counter) + '.csv'), index=True)
        lidar_frame_counter += 1

imu_df = pd.DataFrame(imu_buff, columns = imu_csv_header)
imu_df.to_csv(IMU_CSV_PATH, index=True)

bag.close()