#ifndef MSGS_H
#define MSGS_H

struct Time
{
    uint32_t sec;
    uint32_t nsecs;
};

// This represents an orientation in free space in quaternion form.
struct Quaternion
{
    _Float64 x;
    _Float64 y;
    _Float64 z;
    _Float64 w;
};

struct Vector3
{
    _Float64 x;
    _Float64 y;
    _Float64 z;
};

// This contains the position of a point in free space
struct Point
{
    _Float64 x;
    _Float64 y;
    _Float64 z;
};

struct PointField
{
    const uint8_t INT8 = 1;
    const uint8_t UINT8 = 2;
    const uint8_t INT16 = 3;
    const uint8_t UINT16 = 4;
    const uint8_t INT32 = 5;
    const uint8_t UINT32 = 6;
    const uint8_t FLOAT32 = 7;
    const uint8_t FLOAT64 = 8;

    std::string name;
    uint32_t offset;
    uint8_t datatype;
    uint32_t count;
};

// Standard metadata for higher-level stamped data types.
struct Header
{
    uint32_t seq;
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
    _Float64 covariance[36];
};

struct PoseStamped
{
    Header header;
    Pose pose;
};

struct TwistWithCovariance
{
    Twist twist;
    _Float64 covariance[36];
};

// An array of poses that represents a Path for a robot to follow
struct Path
{
    Header header;
    PoseStamped *poses;
};

struct Pose6D
{
    _Float64 offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
    _Float64 acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
    _Float64 gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
    _Float64 vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
    _Float64 pos[3];       // the preintegrated position (global frame) at the Lidar origin
    _Float64 rot[9];       // the preintegrated rotation (global frame) at the Lidar origin
};

struct Imu
{
    Header header;
    Quaternion orientation;
    _Float64 orientation_covariance[9];
    Vector3 angular_velocity;
    _Float64 angular_velocity_covariance[9];
    Vector3 linear_acceleration;
    _Float64 linear_acceleration_covariance[9];
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
    uint32_t height;
    uint32_t width;
    std::vector<PointField> fields;
    bool is_bigendian;
    uint32_t point_step;
    uint32_t row_step;
    std::vector<unsigned short int> data;
    bool is_dense;
};

#endif