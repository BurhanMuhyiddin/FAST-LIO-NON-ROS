#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <sstream>
#include <Python.h>
#include <so3_math.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include "preprocess.h"
#include "msgs.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

void print_imu_data(const Imu& imu_msg)
{
   std::cout << imu_msg.header.seq << " " << imu_msg.header.stamp.sec << " " << imu_msg.header.stamp.nsecs << " " << imu_msg.header.frame_id << std::endl;
   std::cout << imu_msg.orientation.x << " ";
   std::cout << imu_msg.orientation.y << " ";
   std::cout << imu_msg.orientation.z << " ";
   std::cout << imu_msg.orientation.w << std::endl;
   for (int i = 0; i < 9; ++i)
      std::cout << imu_msg.orientation_covariance[i] << " ";
}

void print_lidar_data(PointCloud2 data_)
{
   std::cout << data_.height * data_.row_step << std::endl;
   for (int i = 0; i < data_.height * data_.row_step; ++i)
   {
      std::cout << data_.data[i] << "\n";
      if (i == 10)
         break;
   }
}

void read_data(const std::string& data_file)
{
   fstream file_stream;
   file_stream.open(data_file, ios::in);

   Imu imu_msg;
   PointCloud2 lidar_msg;
   
   if(file_stream.is_open())
   {
      std::string word;
      while (1)
      {
         file_stream >> word;

         if (word == "---")
            break;

         if (word == "imu")
         {
            file_stream >> imu_msg.header.seq;
            file_stream >> imu_msg.header.stamp.sec;
            file_stream >> imu_msg.header.stamp.nsecs;
            file_stream >> imu_msg.header.frame_id;
            file_stream >> imu_msg.orientation.x;
            file_stream >> imu_msg.orientation.y;
            file_stream >> imu_msg.orientation.z;
            file_stream >> imu_msg.orientation.w;
            for (int i = 0; i < 9; ++i)
               file_stream >> imu_msg.orientation_covariance[i];
            file_stream >> imu_msg.angular_velocity.x;
            file_stream >> imu_msg.angular_velocity.y;
            file_stream >> imu_msg.angular_velocity.z;
            for (int i = 0; i < 9; ++i)
               file_stream >> imu_msg.angular_velocity_covariance[i];
            file_stream >> imu_msg.linear_acceleration.x;
            file_stream >> imu_msg.linear_acceleration.y;
            file_stream >> imu_msg.linear_acceleration.z;
            for (int i = 0; i < 9; ++i)
               file_stream >> imu_msg.linear_acceleration_covariance[i];
         }
         else if (word == "lidar")
         {
            file_stream >> lidar_msg.header.seq;
            file_stream >> lidar_msg.header.stamp.sec;
            file_stream >> lidar_msg.header.stamp.nsecs;
            file_stream >> lidar_msg.header.frame_id;
            file_stream >> lidar_msg.height;
            file_stream >> lidar_msg.width;
            int field_num;
            file_stream >> field_num;
            lidar_msg.fields.clear();
            for (int i = 0; i < field_num; ++i)
            {
               PointField pf;
               file_stream >> pf.name;
               file_stream >> pf.offset;
               file_stream >> pf.datatype;
               file_stream >> pf.count;
               // std::cout << pf.name << pf.offset << pf.datatype << pf.count << std::endl;
               lidar_msg.fields.push_back(pf);
            }
            file_stream >> lidar_msg.is_bigendian;
            // std::cout << lidar_msg.is_bigendian << std::endl;
            file_stream >> lidar_msg.point_step;
            // std::cout << lidar_msg.point_step << std::endl;
            file_stream >> lidar_msg.row_step;
            // std::cout << lidar_msg.row_step << std::endl;
            // std::cout << lidar_msg.height * lidar_msg.row_step << std::endl;
            lidar_msg.data.clear();
            for (uint64_t i = 0; i < lidar_msg.height * lidar_msg.row_step; ++i)
            {
               unsigned short int lidar_data;
               file_stream >> lidar_data;
               lidar_msg.data.push_back(lidar_data);
            }
            file_stream >> lidar_msg.is_dense;
            // print_lidar_data(lidar_msg);
            // break;
         }
      }
      file_stream.close();
   }
}

int main(int argc, char** argv)
{
   read_data("../data/data.txt");
   return 0;
}
