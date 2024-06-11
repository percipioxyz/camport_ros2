#include "percipio_video_mode.h"

namespace percipio_camera {
void VideoStream::reset()
{
    _left_ir.empty();
    _right_ir.empty();
    _depth.empty();
    _color.empty();
    _p3d.empty();
}

bool VideoStream::DepthInit(const cv::Mat& depth, const float* intr, const uint64_t& timestamp)
{
    if(depth.empty()) {
        return false;
    }

    _depth = depth.clone();
    _depth_timestamp = timestamp;
    _depth_cam_info = convertToCameraInfo(*(TY_CAMERA_INTRINSIC*)intr, depth.cols, depth.rows);
    return true;
}

bool VideoStream::ColorInit(const cv::Mat& color, const float* intr, const uint64_t& timestamp)
{
    if(color.empty()) {
        return false;
    }

    _color = color.clone();
    _color_timestamp = timestamp;
    _color_cam_info = convertToCameraInfo(*(TY_CAMERA_INTRINSIC*)intr, color.cols, color.rows);
    return true;
}

bool VideoStream::IRLeftInit(const cv::Mat& ir, const float* intr, const uint64_t& timestamp)
{
    if(ir.empty()) {
        return false;
    }

    _left_ir = ir.clone();
    _lir_timestamp = timestamp;
    _lir_cam_info = convertToCameraInfo(*(TY_CAMERA_INTRINSIC*)intr, ir.cols, ir.rows);
    return true;
}

bool VideoStream::IRRightInit(const cv::Mat& ir, const float* intr, const uint64_t& timestamp)
{
    if(ir.empty()) {
        return false;
    }

    _right_ir = ir.clone();
    _rir_timestamp = timestamp;
    _rir_cam_info = convertToCameraInfo(*(TY_CAMERA_INTRINSIC*)intr, ir.cols, ir.rows);
    return true;
}

bool VideoStream::PointCloudInit(const cv::Mat& p3d, const float* intr, const uint64_t& timestamp)
{
    if(p3d.empty()) {
        return false;
    }

    if(p3d.type() != CV_32FC3) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("VideoStream"), "Invalid p3d data type!");
        return false;
    }
    
    _p3d = p3d.clone();
    _p3d_timestamp = timestamp;
    _p3d_cam_info = convertToCameraInfo(*(TY_CAMERA_INTRINSIC*)intr, p3d.cols, p3d.rows);
    return true;
}

sensor_msgs::msg::CameraInfo VideoStream::convertToCameraInfo(const TY_CAMERA_INTRINSIC& intr, const int width, const int height)
{
    sensor_msgs::msg::CameraInfo ros_cam_info;
    ros_cam_info.width = width;
    ros_cam_info.height = height;

    //No distortion
    ros_cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    ros_cam_info.d.resize(12, 0.0);

    ros_cam_info.k.fill(0.0);
    ros_cam_info.k[0] = intr.data[0];   //fx
    ros_cam_info.k[2] = intr.data[2];   //cx
    ros_cam_info.k[4] = intr.data[4];   //fy;
    ros_cam_info.k[5] = intr.data[5];   //cy;
    ros_cam_info.k[8] = 1.0;

    ros_cam_info.r.fill(0.0);
    ros_cam_info.r[0] = 1;
    ros_cam_info.r[4] = 1;
    ros_cam_info.r[8] = 1;

    ros_cam_info.p.fill(0.0);
    ros_cam_info.p[0] = ros_cam_info.k[0];
    ros_cam_info.p[2] = ros_cam_info.k[2];
    ros_cam_info.p[5] = ros_cam_info.k[4];
    ros_cam_info.p[6] = ros_cam_info.k[5];
    ros_cam_info.p[10] = 1.0;
    
    return ros_cam_info;
}

}