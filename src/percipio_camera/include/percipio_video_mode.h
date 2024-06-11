#pragma once
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "TYApi.h"

namespace percipio_camera {

class VideoStream {
    public: 
        VideoStream() {}
        ~VideoStream() {}

        void reset();
        bool DepthInit(const cv::Mat& depth, const float* intr, const uint64_t& timestamp);
        bool ColorInit(const cv::Mat& color, const float* intr, const uint64_t& timestamp);
        bool IRLeftInit(const cv::Mat& ir, const float* intr, const uint64_t& timestamp);
        bool IRRightInit(const cv::Mat& ir, const float* intr, const uint64_t& timestamp);
        bool PointCloudInit(const cv::Mat& p3d, const float* intr, const uint64_t& timestamp);

                
        //////////////////////////////////////////////
        const cv::Mat& getDepthImage()                       {   return _depth;              }
        const uint64_t& getDepthStramTimestamp()             {   return _depth_timestamp;    }
        const sensor_msgs::msg::CameraInfo& getDepthInfo()   {   return _depth_cam_info;     }
        ////////////////////////////////////////////////////
        const cv::Mat& getColorImage()                       {   return _color;              }
        const uint64_t& getColorStramTimestamp()             {   return _color_timestamp;    }
        const sensor_msgs::msg::CameraInfo& getColorInfo()   {   return _color_cam_info;     }
        //////////////////////////////////////////////
        const cv::Mat& getLeftIRImage()                      {   return _left_ir;            }
        const uint64_t& getLeftIRStramTimestamp()            {   return _lir_timestamp;      }
        const sensor_msgs::msg::CameraInfo& getLeftIRInfo()  {   return _lir_cam_info;       }
        //////////////////////////////////////////////=
        const cv::Mat& getRightIRImage()                     {   return _right_ir;           }
        const uint64_t& getRightIRStramTimestamp()           {   return _rir_timestamp;      }
        const sensor_msgs::msg::CameraInfo& getRightIRInfo() {   return _rir_cam_info;       }
        //////////////////////////////////////////////
        const cv::Mat& getPointCloud()                       {   return _p3d;                }
        const uint64_t& getPointCloudStramTimestamp()        {   return _p3d_timestamp;      }

    private:
        uint64_t _lir_timestamp;
        uint64_t _rir_timestamp;
        uint64_t _depth_timestamp;
        uint64_t _color_timestamp;
        uint64_t _p3d_timestamp;

        sensor_msgs::msg::CameraInfo _lir_cam_info;
        sensor_msgs::msg::CameraInfo _rir_cam_info;
        sensor_msgs::msg::CameraInfo _depth_cam_info;
        sensor_msgs::msg::CameraInfo _color_cam_info;
        sensor_msgs::msg::CameraInfo _p3d_cam_info;

        cv::Mat _left_ir;
        cv::Mat _right_ir;
        cv::Mat _depth;
        cv::Mat _color;
        cv::Mat _p3d;

        sensor_msgs::msg::CameraInfo convertToCameraInfo(const TY_CAMERA_INTRINSIC& intr, const int width, const int height);
};

}