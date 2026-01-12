#pragma once
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "TYApi.h"

namespace percipio_camera {

class image_intrinsic;

class TYImage {
public:
    TYImage();
    TYImage(const int _width, const int _height, const TYPixFmt _fmt);
    TYImage(const int _width, const int _height, const TYPixFmt _fmt, void* data);
    TYImage(const TYImage& other);
    TYImage(TYImage&& other) noexcept;
    ~TYImage();

    TYImage& operator=(const TYImage& other);
    TYImage& operator=(TYImage&& other) noexcept;
    bool empty() const;
    TYImage clone() const;
    TYImage resize(int newWidth, int newHeight) const;
    void clear();
    void release();
    
    int width() const { return m_Width; }
    
    int height() const { return m_Height; }
    
    TYPixFmt format() const { return m_fmt; }
    
    void* data() const;
    size_t size() const;
    bool isExternalData() const;

    void setPixelFormat(const TYPixFmt fmt) { m_fmt = fmt; }
    void setWidth(const int w) { m_Width = w; }
    void setHeight(const int h) { m_Height = h; }

private:
    int m_Width;
    int m_Height;
    TYPixFmt m_fmt;
    bool m_ownsData;
    unsigned char* m_externalData;
    std::vector<unsigned char> m_data;
    
    void cleanup() {
        m_data.clear();
        m_externalData = nullptr;
    }
    
    size_t calculateRowSize(int width, TYPixFmt fmt) const; 
    
    size_t calculateDataSize(int width, int height, TYPixFmt fmt) const;
    
    size_t getPixelSize(TYPixFmt fmt) const;
    
    TY_STATUS resizeMono8To(int newWidth, int newHeight, TYImage& dest) const;
    TY_STATUS resizeMono16To(int newWidth, int newHeight, TYImage& dest) const;
    TY_STATUS resizeRGB8To(int newWidth, int newHeight, TYImage& dest) const;
    TY_STATUS resizeABC16To(int newWidth, int newHeight, TYImage& dest) const;
    TY_STATUS resizeABC32fTo(int newWidth, int newHeight, TYImage& dest) const;
};

class VideoStream {
    public: 
        VideoStream() {}
        ~VideoStream() {}

        void reset();
        bool DepthInit(const TYImage& depth, image_intrinsic& intr, const uint64_t& timestamp);
        bool ColorInit(const TYImage& color, image_intrinsic& intr, const uint64_t& timestamp);
        bool IRLeftInit(const TYImage& ir, image_intrinsic& intr, const uint64_t& timestamp);
        bool IRRightInit(const TYImage& ir, image_intrinsic& intr, const uint64_t& timestamp);
        bool PointCloudInit(const TYImage& p3d, image_intrinsic& intr, const uint64_t& timestamp);

                
        //////////////////////////////////////////////
        const TYImage& getDepthImage()                       {   return _depth;              }
        const uint64_t& getDepthStramTimestamp()             {   return _depth_timestamp;    }
        const sensor_msgs::msg::CameraInfo& getDepthInfo()   {   return _depth_cam_info;     }
        ////////////////////////////////////////////////////
        const TYImage& getColorImage()                       {   return _color;              }
        const uint64_t& getColorStramTimestamp()             {   return _color_timestamp;    }
        const sensor_msgs::msg::CameraInfo& getColorInfo()   {   return _color_cam_info;     }
        //////////////////////////////////////////////
        const TYImage& getLeftIRImage()                      {   return _left_ir;            }
        const uint64_t& getLeftIRStramTimestamp()            {   return _lir_timestamp;      }
        const sensor_msgs::msg::CameraInfo& getLeftIRInfo()  {   return _lir_cam_info;       }
        //////////////////////////////////////////////=
        const TYImage& getRightIRImage()                     {   return _right_ir;           }
        const uint64_t& getRightIRStramTimestamp()           {   return _rir_timestamp;      }
        const sensor_msgs::msg::CameraInfo& getRightIRInfo() {   return _rir_cam_info;       }
        //////////////////////////////////////////////
        const TYImage& getPointCloud()                       {   return _p3d;                }
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

        TYImage _left_ir;
        TYImage _right_ir;
        TYImage _depth;
        TYImage _color;
        TYImage _p3d;

        sensor_msgs::msg::CameraInfo convertToCameraInfo(const TY_CAMERA_INTRINSIC& intr, const int width, const int height);
};

}