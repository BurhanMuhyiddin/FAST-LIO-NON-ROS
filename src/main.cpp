#include <iostream>
#include <fstream>
#include <string>
#include "csv.h"
#include "fast_lio.hpp"

#define BASE_PATH   std::string("../")
#define DATA_PATH   std::string(BASE_PATH + std::string("data/"))
// #define DATA_FILE_HORIZON    std::string("../data/synch_data.txt") // change path based on your input file
#define DATA_FILE_HORIZON    std::string("../data/8_shape_path.txt")

mutex m_stop;

bool is_stop = false;

// read imu and lidar files
// convert them to the proper format
// feed them through interface
double msr_freq = 50.0;
const int imu_freq = 50;
const int lidar_freq = 10;
const int diff_freq = imu_freq / lidar_freq;

/* This is for parsing HITACHI dataset (or any dataset in the proper format from .csv)
bool parse_file(FastLio &fast_lio)
{
    const std::string imu_path = DATA_PATH + std::string("imu/imu.csv");
    const std::string lidar_path = DATA_PATH + std::string("lidar/");

    const double imu_period = double(1.0 / imu_freq); // in s
    const double lidar_period = double(1.0 / lidar_freq); // in s
    const double reading_period = 1000.0 / msr_freq; // in ms

    custom_messages::Imu imu_msg;
    custom_messages::CustomMsg lidar_msg;

    int lidar_file_number = 0, imu_file_counter = 0;

    bool is_first_imu = true;

    double imu_timestamp_corrected = 0.0; // in s

    try
    {
        io::CSVReader<7> imu_reader(imu_path);
        imu_reader.read_header(io::ignore_extra_column, "Timesamp", "Gyrox", "Gyroy", "Gyroz", "Accx", "Accy", "Accz");
        double imu_timestamp, gyroX, gyroY, gyroZ, accX, accY, accZ;
        double lidar_timestamp, pX, pY, pZ, pI;
        std::cout << "Started to read data..." << std::endl;
        while (imu_reader.read_row(imu_timestamp, gyroX, gyroY, gyroZ, accX, accY, accZ))
        {
            {
                const std::lock_guard<std::mutex> lock(m_stop);
                if (is_stop)    break;
            }
            auto start = std::chrono::high_resolution_clock::now();
            // std::cout << "IMU" << std::endl;
            imu_msg.header.stamp = imu_msg.header.stamp.fromSec(imu_timestamp_corrected);
            imu_msg.header.seq = 0;
            imu_msg.header.frame_id = "livox_frame";
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 1.0;
            for (int i = 0; i < 9; ++i)
                imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity.x = gyroX;
            imu_msg.angular_velocity.y = gyroY;
            imu_msg.angular_velocity.z = gyroZ;
            for (int i = 0; i < 9; ++i)
                imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration.x = accX;
            imu_msg.linear_acceleration.y = accY;
            imu_msg.linear_acceleration.z = accZ;
            for (int i = 0; i < 9; ++i)
                imu_msg.linear_acceleration_covariance[i] = 0.0;
            
            ImuConstPtr imu_msg_cptr(new custom_messages::Imu(imu_msg));
            fast_lio.feed_imu(imu_msg_cptr);

            if (imu_file_counter == 0)
            {
                // std::cout << "LIDAR" << std::endl;
                lidar_msg.header.seq = 0;
                lidar_msg.header.stamp = lidar_msg.header.stamp.fromSec(imu_timestamp_corrected);
                lidar_msg.header.frame_id = "livox_frame";
                lidar_msg.timebase = lidar_msg.header.stamp.toNsec();
                lidar_msg.lidar_id = 0;
                for (int i = 0; i < 3; i++)
                    lidar_msg.rsvd[i] = 0;
                unsigned int point_num = 0;
                io::CSVReader<5> lidar_reader(lidar_path + std::string(6 - ((lidar_file_number==0) ? 1:(int)floor(log10(lidar_file_number+1))), '0') + to_string(lidar_file_number) + std::string(".csv"));
                lidar_reader.read_header(io::ignore_extra_column, "Timestamp", "Px", "Py", "Pz", "Pi");
                double lidar_first_sample_timestamp = 0.0;
                int scan_num = 6;
                unsigned int scan_line_num = 0;
                lidar_msg.points.clear();
                while (lidar_reader.read_row(lidar_timestamp, pX, pY, pZ, pI))
                {
                    if (point_num == 0)
                    {
                        lidar_first_sample_timestamp = lidar_timestamp;
                    }
                    custom_messages::CustomPoint cp;
                    // std::cout << "x: " << pX << ", y: " << pY << ", z: " << pZ << ", i: " << pI << std::endl;
                    cp.x = pX;
                    cp.y = pY;
                    cp.z = pZ;
                    cp.reflectivity = pI;
                    cp.offset_time = (unsigned long int)(lidar_timestamp*1000000000.0 - lidar_first_sample_timestamp*1000000000.0);
                    cp.line = scan_line_num;
                    scan_line_num = (scan_line_num + 1)%scan_num;
                    cp.tag = 16;
                    lidar_msg.points.push_back(cp);
                    
                    point_num++;
                }
                lidar_msg.point_num = point_num;
                CstMsgConstPtr lidar_msg_cptr(new custom_messages::CustomMsg(lidar_msg));
                fast_lio.feed_lidar(lidar_msg_cptr);
            }

            imu_timestamp_corrected += imu_period;
            imu_file_counter = (imu_file_counter+1)%diff_freq;

            auto stop = std::chrono::high_resolution_clock::now();
            auto duration_ = std::chrono::duration<double, milli>(stop - start).count();
            while (duration_ < reading_period)
            {
                stop = std::chrono::high_resolution_clock::now();
                duration_ = std::chrono::duration<double, milli>(stop - start).count();
            }
        }
        std::cout << "Finished reading data..." << std::endl;
        {
            const std::lock_guard<std::mutex> lock(m_stop);
            is_stop = true;
        }
        return true;
    }
    catch(...)
    {
        std::cerr << "File couldn't be read successfully..." << std::endl;
        return false;
    }
}
*/

// This is for parsing 8_shape trajectory from .txt
// this is just for testing purposes
bool parse_file(FastLio &fast_lio)
{
    fstream file_stream;
    file_stream.open(DATA_FILE_HORIZON, ios::in);

    double reading_period = 1000.0 / msr_freq; // in ms

    custom_messages::Imu imu_msg;
    custom_messages::CustomMsg lidar_msg;

    if(file_stream.is_open())
    {
        std::cout << "Started to read the file..." << std::endl;
        std::string word;
        while (true)
        {
            {
                const std::lock_guard<std::mutex> lock(m_stop);
                if (is_stop)    break;
            }
            auto start = std::chrono::high_resolution_clock::now();

            file_stream >> word;

            if (word == "---")
            {
                break;
            }

            if (word == "imu")
            {
                long double sec;
                file_stream >> sec;
                imu_msg.header.stamp = imu_msg.header.stamp.fromSec(sec);
                imu_msg.header.seq = 0;
                imu_msg.header.frame_id = "livox_frame";
                imu_msg.orientation.x = 0.0;
                imu_msg.orientation.y = 0.0;
                imu_msg.orientation.z = 0.0;
                imu_msg.orientation.w = 1.0;
                // std::cout << imu_msg.orientation.x << ", " << imu_msg.orientation.y << ", " << imu_msg.orientation.z << ", " << imu_msg.orientation.w << std::endl;
                for (int i = 0; i < 9; ++i)
                    imu_msg.orientation_covariance[i] = 0.0;
                file_stream >> imu_msg.angular_velocity.x;
                file_stream >> imu_msg.angular_velocity.y;
                file_stream >> imu_msg.angular_velocity.z;
                // std::cout << imu_msg.angular_velocity.x << ", " << imu_msg.angular_velocity.y << ", " << imu_msg.angular_velocity.z << std::endl;
                for (int i = 0; i < 9; ++i)
                    imu_msg.angular_velocity_covariance[i] = 0.0;
                file_stream >> imu_msg.linear_acceleration.x;
                file_stream >> imu_msg.linear_acceleration.y;
                file_stream >> imu_msg.linear_acceleration.z;
                // std::cout << imu_msg.linear_acceleration.x << ", " << imu_msg.linear_acceleration.y << ", " << imu_msg.linear_acceleration.z << std::endl;
                for (int i = 0; i < 9; ++i)
                    imu_msg.linear_acceleration_covariance[i] = 0.0;
                // // call imu callback with the message
                ImuConstPtr imu_msg_cptr(new custom_messages::Imu(imu_msg));
                fast_lio.feed_imu(imu_msg_cptr);
            }
            else if (word == "lidar")
            {
                lidar_msg.header.seq = 0;
                double sec;
                file_stream >> sec;
                lidar_msg.header.stamp = lidar_msg.header.stamp.fromSec(sec);
                lidar_msg.header.frame_id = "livox_frame";
                // lidar_msg.timebase = sec * 1e9;
                lidar_msg.timebase = lidar_msg.header.stamp.toNsec();
                lidar_msg.lidar_id = 0;
                for (int i = 0; i < 3; i++)
                    lidar_msg.rsvd[i] = 0;
                unsigned long int len;
                file_stream >> len;
                lidar_msg.point_num = len;
                // std::cout << "len " << len << std::endl;

                lidar_msg.points.clear();
                for (int i = 0; i < len; ++i)
                {
                    custom_messages::CustomPoint cp;
                    file_stream >> cp.x;
                    file_stream >> cp.y;
                    file_stream >> cp.z;
                    file_stream >> cp.reflectivity;
                    file_stream >> cp.offset_time;
                    file_stream >> cp.line;
                    // std::cout << cp.offset_time << std::endl;
                    cp.tag = 16;
                    // if (i >= len - 20 && i < len)
                    //    std::cout << cp.x << " " << cp.y << " " << cp.z << " " << cp.reflectivity << " " << cp.offset_time << std::endl;
                    // std::cout << pf.name << pf.offset << pf.datatype << pf.count << std::endl;
                    lidar_msg.points.push_back(cp);
                }
                // call lidar callback with the message
                CstMsgConstPtr lidar_msg_cptr(new custom_messages::CustomMsg(lidar_msg));
                fast_lio.feed_lidar(lidar_msg_cptr);
                // print_lidar_data(lidar_msg_cptr);
                // break;
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration_ = std::chrono::duration<double, milli>(stop - start).count();
            while (duration_ < reading_period)
            {
                stop = std::chrono::high_resolution_clock::now();
                duration_ = std::chrono::duration<double, milli>(stop - start).count();
            }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    else
    {
        return false;
        std::cerr << "File couldn't be opened..." << std::endl;
    }
    {
        const std::lock_guard<std::mutex> lock(m_stop);
        is_stop = true;
    }
    file_stream.close();
    std::cout << "Finished reading file..." << std::endl;
    return true;
}

void SigHandle(int sig)
{
    const std::lock_guard<std::mutex> lock(m_stop);
    is_stop = true;
}

int main()
{
    FastLio fast_lio;

    std::vector<double> pose;

    // start the thread of parsing file
    std::thread data_reading_th(parse_file, std::ref(fast_lio));

    signal(SIGINT, SigHandle);

    const double reading_period = 1000.0 / 5000.0;

    // start to process in main thread
    while (true)
    {
        {
            const std::lock_guard<std::mutex> lock(m_stop);
            if (is_stop)    break;
        }
        auto start = std::chrono::high_resolution_clock::now();
        fast_lio.process();

        pose = std::move(fast_lio.get_pose());
        // fast_lio.write_to_file(pose);
        // std::cout << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration_ = std::chrono::duration<double, milli>(stop - start).count();
        // fast_lio.write_to_file(duration_);
        // std::cout << "Time to process: " << duration_ << " ms" << std::endl;
        while (duration_ < reading_period)
        {
            stop = std::chrono::high_resolution_clock::now();
            duration_ = std::chrono::duration<double, milli>(stop - start).count();
        }
    }
    
    if (data_reading_th.joinable())
    {
        data_reading_th.join();
    }

    std::cout << "Finished processing... Exiting..." << std::endl;

    return 0;
}