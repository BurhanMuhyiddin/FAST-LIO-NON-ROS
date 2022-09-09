#ifndef MSGS_H
#define MSGS_H

namespace custom_messages
{
    struct Time
    {
        unsigned long int sec;
        unsigned long int nsecs;

        unsigned long toNsec() const { return (unsigned long)sec*1000000000ull + (unsigned long)nsecs; };
        double toSec() const { return (double)sec + 1e-9*(double)nsecs; };
    };

    // This represents an orientation in free space in quaternion form.
    struct Quaternion
    {
        double x;
        double y;
        double z;
        double w;
    };

    struct Vector3
    {
        double x;
        double y;
        double z;
    };

    // This contains the position of a point in free space
    struct Point
    {
        double x;
        double y;
        double z;
    };

    struct PointField
    {
        const unsigned short int INT8 = 1;
        const unsigned short int UINT8 = 2;
        const unsigned short int INT16 = 3;
        const unsigned short int UINT16 = 4;
        const unsigned short int INT32 = 5;
        const unsigned short int UINT32 = 6;
        const unsigned short int FLOAT32 = 7;
        const unsigned short int FLOAT64 = 8;

        std::string name;
        unsigned long int offset;
        unsigned short int datatype;
        unsigned long int count;
    };

    // Standard metadata for higher-level stamped data types.
    struct Header
    {
        unsigned long int seq;
        Time stamp;
        std::string frame_id;
    };

    struct Pose
    {
        Point point;
        Quaternion orientation;
    };

    struct Twist
    {
        Vector3 linear;
        Vector3 angular;
    };

    // This represents a pose in free space with uncertainty.
    struct PoseWithCovariance
    {
        Pose pose;
        double covariance[36];
    };

    struct PoseStamped
    {
        Header header;
        Pose pose;
    };

    struct TwistWithCovariance
    {
        Twist twist;
        double covariance[36];
    };

    // An array of poses that represents a Path for a robot to follow
    struct Path
    {
        Header header;
        PoseStamped *poses;
    };

    struct Pose6D
    {
        double offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
        double acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
        double gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
        double vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
        double pos[3];       // the preintegrated position (global frame) at the Lidar origin
        double rot[9];       // the preintegrated rotation (global frame) at the Lidar origin
    };

    struct Imu
    {
        Header header;
        Quaternion orientation;
        double orientation_covariance[9];
        Vector3 angular_velocity;
        double angular_velocity_covariance[9];
        Vector3 linear_acceleration;
        double linear_acceleration_covariance[9];
    };

    struct Odometry
    {
        Header header;
        std::string child_frame_id;
        PoseWithCovariance pose;
        TwistWithCovariance twist;
    };

    // This message holds a collection of N-dimensional points, which may
    // contain additional information such as normals, intensity, etc. The
    // point data is stored as a binary blob, its layout described by the
    // contents of the "fields" array.
    struct PointCloud2
    {
        Header header;
        unsigned long int height;
        unsigned long int width;
        std::vector<PointField> fields;
        bool is_bigendian;
        unsigned long int point_step;
        unsigned long int row_step;
        std::vector<unsigned short int> data;
        bool is_dense;
    };
}

#endif