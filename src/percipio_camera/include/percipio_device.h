#pragma once
#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>
#include <thread>
#include <map>
#include <condition_variable>
#include <mutex>

#include "TYParameter.h"
#include "DepthStreamProc.h"
#include "percipio_video_mode.h"

//#define IMAGE_DoUnsitortion_With_OpenCV     

namespace percipio_camera {

enum percipio_stream_type {
    DEPTH,
    COLOR,
    IR_LETF,
    IR_RIGHT
};

enum percipio_dev_workmode {
    CONTINUS = 0,
    SOFTTRIGGER,
    HARDTRIGGER,
};

enum percipio_dev_ros_event {
    TY_EVENT_DEVICE_CONNECT = 50000,
    TY_EVENT_DEVICE_TIMEOUT = 50001,
};

enum EncodingType : uint32_t  
{
  HUFFMAN = 0,
};

enum GigEVersion
{
    GigeE_2_0,
    GigeE_2_1,    
};

struct percipio_distortion_map_info
{
    public:
        int m_map_width = 0;
        int m_map_height = 0;
        std::vector<float> f_map_x;
        std::vector<float> f_map_y;

        bool IsValid() {
            if(m_map_width * m_map_height <= 0) return false;
            return true;
        }
};

class PercipioDevice;
typedef std::pair<percipio_stream_type, int> percipio_stream_index_pair;
typedef boost::function<void(VideoStream&)> FrameCallbackFunction;
typedef boost::function<void(PercipioDevice*, TY_EVENT_INFO*)> PercipioDeviceEventCallbackFunction;


struct percipio_stream_property
{
    percipio_stream_index_pair idx;
    std::string resolution;
    std::string format;
};

struct percipio_video_mode
{
    uint32_t fmt;
    uint32_t width;
    uint32_t height;
    uint32_t binning;
    std::string desc;
};

typedef std::map<uint32_t, std::vector<percipio_video_mode>> PercipioVideoMode;

class PercipioCameraNode;
class PercipioDevice
{
    public:
        PercipioDevice(const char* faceId, const char* deviceId);
        ~PercipioDevice();

        void set_workmode(percipio_dev_workmode mode) { workmode = mode; }

        TY_STATUS Reconnect();
        void Release();
        void register_node(void* para) { _node = (PercipioCameraNode*)para; }

        bool isAlive();
        PercipioDeviceEventCallbackFunction _event_callback;
        void registerCameraEventCallback(PercipioDeviceEventCallbackFunction callback);

        std::string serialNumber();
        std::string modelName();
        std::string buildHash();
        std::string configVersion();

        bool hasColor();
        bool hasDepth();
        bool hasLeftIR();
        bool hasRightIR();

        TY_STATUS dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes);
        TY_STATUS image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode);

        void enable_gvsp_resend(const bool en);

        void enable_offline_reconnect(const bool en);

        bool set_laser_power(const int power);
        
        bool set_tof_depth_quality(const std::string& qua);
        bool set_tof_modulation_threshold(const int threshold);
        bool set_tof_jitter_threshold(const int threshold);
        bool set_tof_filter_threshold(const int threshold);
        bool set_tof_channel(const int chan);
        bool set_tof_HDR_ratio(const int ratio);

        float getDepthValueScale();
        
        bool update_color_aec_roi(int x, int y , int w, int h);
        bool stream_open(const percipio_stream_index_pair& idx, const std::string& resolution, const std::string& format);
        bool stream_close(const percipio_stream_index_pair& idx);
        bool stream_start();
        bool stream_stop();
        void send_softtrigger();

        void setFrameCallback(FrameCallbackFunction callback);

        void topics_depth_stream_enable(bool enable);
        void topics_point_cloud_enable(bool enable);
        void topics_color_point_cloud_enable(bool enable);
        void topics_depth_registration_enable(bool enable);
        
        bool load_default_parameter();

        std::mutex softtrigger_mutex;

        std::mutex offline_detect_mutex;
        std::condition_variable offline_detect_cond;

        TY_EVENT_INFO device_ros_event;

    private:
        PercipioCameraNode* _node;

        GigEVersion  gige_version = GigeE_2_0;

        bool b_packet_resend_en = false;
        bool b_dev_auto_reconnect = false;
        bool reconnect = false;
        std::string strFaceId;
        std::string strDeviceId;
        std::vector<percipio_stream_property> m_streams;

        PercipioVideoMode mVideoMode;

        TY_AEC_ROI_PARAM ROI;
        bool enable_rgb_aec_roi = false;

        bool alive;
        TY_INTERFACE_HANDLE hIface;
        TY_DEV_HANDLE handle;

        TY_DEVICE_BASE_INFO base_info;
        TY_COMPONENT_ID allComps;

        percipio_dev_workmode workmode = CONTINUS;

        std::atomic_bool is_running_{false};

        FrameCallbackFunction  _callback;

#ifdef IMAGE_DoUnsitortion_With_OpenCV
        percipio_distortion_map_info depth_map;
        percipio_distortion_map_info color_map;
#else
        bool  has_depth_calib_data = false;
        bool  has_color_calib_data = false;
        TY_CAMERA_CALIB_INFO depth_calib_data;
        TY_CAMERA_CALIB_INFO color_calib_data;
        
#endif
        bool topics_depth_ = false;
        bool topics_p3d_ = false;
        bool topics_color_p3d_ = false;
        bool topics_d_registration_ = false;

        std::shared_ptr<std::thread> device_reconnect_thread = nullptr;
        void device_offline_reconnect();

        float f_scale_unit = 1.f;
        TY_CAMERA_CALIB_INFO cam_depth_calib_data;
        TY_CAMERA_CALIB_INFO cam_color_calib_data;

        std::vector<float> cam_depth_intrinsic;
        std::vector<float> cam_color_intrinsic;
        std::vector<float> cam_leftir_intrinsic;
        std::vector<float> cam_rightir_intrinsic;

        std::shared_ptr<std::thread> frame_recive_thread_ = nullptr;
        std::shared_ptr<VideoStream> VideoStreamPtr = nullptr;
        std::vector<unsigned char> frameBuffer[2];
        void frameDataReceive();

        TY_STATUS device_open(const char* faceId, const char* deviceId);

        bool resolveStreamResolution(const std::string& resolution_, int& width, int& height);
        std::string parseStreamFormat(const std::string& format);

        TY_STATUS color_stream_aec_roi_init();

        void StreamDistortionMapInit(TY_COMPONENT_ID comp, percipio_distortion_map_info& map);

        void colorStreamReceive(cv::Mat& color, uint64_t& timestamp);
        void leftIRStreamReceive(cv::Mat& ir,   uint64_t& timestamp);
        void rightIRStreamReceive(cv::Mat& ir,  uint64_t& timestamp);
        void depthStreamReceive(cv::Mat& depth, uint64_t& timestamp);
        void p3dStreamReceive(cv::Mat& depth,   uint64_t& timestamp);

};
}
