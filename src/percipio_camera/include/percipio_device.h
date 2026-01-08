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
#include "percipio_depth_algorithm.h"

#include "percipio_video_mode.h"

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

class image_intrinsic
{
    public:
        image_intrinsic() {}
        image_intrinsic(const int width, const int height, const float fx, const float fy, const float cx, const float cy);
        image_intrinsic(const int width, const int height, const TY_CAMERA_INTRINSIC& intr);
        ~image_intrinsic();

        int width()  { return m_width; }
        int height() { return m_height; }
        float fx() { return intrinsic[0]; }
        float cx() { return intrinsic[2]; }
        float fy() { return intrinsic[4]; }
        float cy() { return intrinsic[5]; }
 
        image_intrinsic resize(const float f_scale_x, const float f_scale_y);
        image_intrinsic resize(const int width, const int height);

        TY_CAMERA_INTRINSIC data();
    private:
        int32_t m_width;
        int32_t m_height;

        /// | fx|  0| cx|
        /// |  0| fy| cy|
        /// |  0|  0|  1|
        float intrinsic[9];
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

class GigEBase
{
public:
    GigEBase(const TY_DEV_HANDLE dev) : hDevice(dev){};
    virtual ~GigEBase() {};

    virtual TY_STATUS init() = 0;

    virtual void video_mode_init();

    virtual TY_STATUS dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes) = 0;

    virtual TY_STATUS image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode) = 0;

    virtual TY_STATUS stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data) = 0;

    virtual void depth_stream_distortion_check(bool& has_undist_data) = 0;

    virtual TY_STATUS depth_scale_unit_init(float& dept_scale_unit) = 0;

    virtual TY_STATUS color_stream_aec_roi_init(const TY_AEC_ROI_PARAM& ROI) = 0;

    virtual TY_STATUS set_tof_depth_quality(const std::string& qua) = 0;
    virtual TY_STATUS set_tof_modulation_threshold(int threshold) = 0;
    virtual TY_STATUS set_tof_jitter_threshold(int threshold) = 0;
    virtual TY_STATUS set_tof_filter_threshold(int threshold) = 0;
    virtual TY_STATUS set_tof_channel(int chan) = 0;
    virtual TY_STATUS set_tof_HDR_ratio(int ratio) = 0;


    uint32_t streams() { return allComps;}
public:
    PercipioVideoMode mVideoMode;
    
protected:
    TY_DEV_HANDLE hDevice;
    TY_COMPONENT_ID allComps = 0;
};

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

        std::unique_ptr<GigEBase> m_gige_dev;

        void enable_gvsp_resend(const bool en);

        void enable_offline_reconnect(const bool en);

        bool set_laser_power(const int power);
        
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

        void depth_speckle_filter_init(bool enable, int spec_size, int spec_diff, float phy_size);
        void dpeth_time_domain_filter_init(bool enable, int number);

        std::mutex softtrigger_mutex;

        std::mutex offline_detect_mutex;
        std::condition_variable offline_detect_cond;

        TY_EVENT_INFO device_ros_event;

        TY_STATUS set_tof_depth_quality(const std::string& qua) { return m_gige_dev->set_tof_depth_quality(qua); };
        TY_STATUS set_tof_modulation_threshold(int threshold) { return m_gige_dev->set_tof_modulation_threshold(threshold); };
        TY_STATUS set_tof_jitter_threshold(int threshold) { return m_gige_dev->set_tof_jitter_threshold(threshold); };
        TY_STATUS set_tof_filter_threshold(int threshold) { return m_gige_dev->set_tof_filter_threshold(threshold); };
        TY_STATUS set_tof_channel(int chan) { return m_gige_dev->set_tof_channel(chan); };
        TY_STATUS set_tof_HDR_ratio(int ratio) { return m_gige_dev->set_tof_HDR_ratio(ratio); };

    private:
        PercipioCameraNode* _node;

        GigEVersion  gige_version = GigeE_2_0;

        bool b_packet_resend_en = false;
        bool b_dev_auto_reconnect = false;
        bool reconnect = false;
        std::string strFaceId;
        std::string strDeviceId;
        std::vector<percipio_stream_property> m_streams;

        TY_AEC_ROI_PARAM ROI;
        bool enable_rgb_aec_roi = false;

        bool alive;
        TY_INTERFACE_HANDLE hIface;
        TY_DEV_HANDLE handle;

        TY_DEVICE_BASE_INFO base_info;
        
        percipio_dev_workmode workmode = CONTINUS;

        std::atomic_bool is_running_{false};

        FrameCallbackFunction  _callback;

        bool  b_need_do_depth_undistortion = false;
        bool  has_depth_calib_data = false;
        bool  has_color_calib_data = false;

        bool topics_depth_ = false;
        bool topics_p3d_ = false;
        bool topics_color_p3d_ = false;
        bool topics_d_registration_ = false;

        bool b_depth_spk_filter_en = false;
        int  m_depth_spk_size = 150;
        int  m_depth_spk_diff = 64;
        float f_depth_spk_phy_size = 20.f;

        bool b_depth_time_domain_en = false;
        int  m_depth_time_domain_frame_num = 3;
        std::unique_ptr<DepthTimeDomainMgr> DepthDomainTimeFilterMgrPtr;

        std::unique_ptr<std::thread> device_reconnect_thread = nullptr;
        void device_offline_reconnect();

        float f_scale_unit = 1.f;
        TY_CAMERA_CALIB_INFO cam_depth_calib_data;
        TY_CAMERA_CALIB_INFO cam_color_calib_data;

        image_intrinsic cam_depth_intrinsic;
        image_intrinsic cam_color_intrinsic;
        image_intrinsic cam_leftir_intrinsic;
        image_intrinsic cam_rightir_intrinsic;

        std::unique_ptr<std::thread> frame_recive_thread_ = nullptr;
        std::unique_ptr<VideoStream> VideoStreamPtr = nullptr;
        std::vector<unsigned char> frameBuffer[2];
        void frameDataReceive();

        TY_STATUS device_open(const char* faceId, const char* deviceId);

        bool resolveStreamResolution(const std::string& resolution_, uint32_t& width, uint32_t& height);
        std::string parseStreamFormat(const std::string& format);

        void colorStreamReceive(const TYImage& color, uint64_t& timestamp);
        void leftIRStreamReceive(const TYImage& ir,   uint64_t& timestamp);
        void rightIRStreamReceive(const TYImage& ir,  uint64_t& timestamp);
        void depthStreamReceive(TYImage& depth, uint64_t& timestamp);
        void p3dStreamReceive(const TYImage& depth,   uint64_t& timestamp);
};
}
