#include "percipio_device.h"
#include "TYCoordinateMapper.h"
#include "TYImageProc.h"

#include "percipio_camera_node.h"
#include "Utils.hpp"

#include "gige_2_0.h"
#include "gige_2_1.h"

namespace percipio_camera {

#define INVALID_COMPONENT_ID        (0xFFFFFFFF)

#define LOG_HEAD_PERCIPIO_DEVICE  "percipio_device"

static uint32_t StreamConvertComponent(const percipio_stream_index_pair& idx)
{
    switch(idx.first) {
    case DEPTH:
        return TY_COMPONENT_DEPTH_CAM;
    case COLOR:
        return TY_COMPONENT_RGB_CAM;
    case IR_LETF:
        return TY_COMPONENT_IR_CAM_LEFT;
    case IR_RIGHT:
        return TY_COMPONENT_IR_CAM_RIGHT;
    default:
        return INVALID_COMPONENT_ID;
    }
}

static const char* StreamConvertSourceDesc(const percipio_stream_index_pair& idx)
{
    switch(idx.first) {
    case DEPTH:
        return "Depth";
    case COLOR:
        return "Texture";
    case IR_LETF:
        return "Left";
    case IR_RIGHT:
        return "Right";
    default:
        return "Unknown";
    }
}

static TYImageInfo ty_image_info(const TY_IMAGE_DATA& image_data) {
    TYImageInfo info;
    info.width = image_data.width;
    info.height = image_data.height; 
    info.format = image_data.pixelFormat;
    info.dataSize = image_data.size;
    info.data = image_data.buffer; 
    return info;
}

class FPSCounter {
private:
    struct State {
        struct timeval start_time;
        int counter;
        bool initialized;
        
        State() : counter(0), initialized(false) {
            start_time.tv_sec = 0;
            start_time.tv_usec = 0;
        }
    };
    
    static std::shared_ptr<State> state_;

    static long timeval_to_ms(const struct timeval& tv) {
        return tv.tv_sec * 1000 + tv.tv_usec / 1000;
    }

public:
    static float get_fps() {
        if (!state_) {
            state_ = std::make_shared<State>();
        }
        
        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        
        if (!state_->initialized) {
            state_->start_time = current_time;
            state_->initialized = true;
            return -1.0f;
        }
        
        state_->counter++;
        
        long elapsed_ms = timeval_to_ms(current_time) - timeval_to_ms(state_->start_time);
        
        if (elapsed_ms < 5000) {
            return -1.0f;
        }
        
        float fps = 1000.0f * state_->counter / elapsed_ms;
        
        reset();
        
        return fps;
    }
    
    static void reset() {
        if (!state_) {
            state_ = std::make_shared<State>();
        } else {
            state_->counter = 0;
            state_->initialized = false;
        }
    }
};

std::shared_ptr<FPSCounter::State> FPSCounter::state_ = nullptr;
static FPSCounter fps_counter;

image_intrinsic::image_intrinsic(const int width, const int height, const float fx, const float fy, const float cx, const float cy)
{
    m_width = width;
    m_height = height;

    /// | fx|  0| cx|
    /// |  0| fy| cy|
    /// |  0|  0|  1|
    intrinsic[0] = fx;
    intrinsic[1] = 0;
    intrinsic[2] = cx;
    
    intrinsic[3] = 0;
    intrinsic[4] = fy;
    intrinsic[5] = cy;

    intrinsic[6] = 0;
    intrinsic[7] = 0;
    intrinsic[8] = 1;
}

image_intrinsic::image_intrinsic(const int width, const int height, const TY_CAMERA_INTRINSIC& intr)
{
    m_width = width;
    m_height = height;

    memcpy(intrinsic, intr.data, sizeof(intrinsic));
}

image_intrinsic::~image_intrinsic()
{

}

image_intrinsic image_intrinsic::resize(const float f_scale_x, const float f_scale_y)
{
    return image_intrinsic(
                width()  * f_scale_x, 
                height() * f_scale_y, 
                fx() * f_scale_x, 
                fy() * f_scale_y, 
                cx() * f_scale_x, 
                cy() * f_scale_y);
}

image_intrinsic image_intrinsic::resize(const int width, const int height)
{
    float f_scale_x = 1.f * width / m_width;
    float f_scale_y = 1.f * height / m_height;
    return resize(f_scale_x, f_scale_y);
}

TY_CAMERA_INTRINSIC image_intrinsic::data()
{
    TY_CAMERA_INTRINSIC intr;
    memcpy(intr.data, intrinsic, sizeof(intrinsic));
    return intr;
}

void GigEBase::video_mode_init()
{
    if(allComps & TY_COMPONENT_RGB_CAM) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_RGB_CAM, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "color stream: ");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                //uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_RGB_CAM] = image_mode_list;
        TYDisableComponents(hDevice, TY_COMPONENT_RGB_CAM);
    } else {
        mVideoMode[TY_COMPONENT_RGB_CAM].clear();
    }

    if(allComps & TY_COMPONENT_DEPTH_CAM) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_DEPTH_CAM, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "depth stream: ");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                //uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_DEPTH_CAM] = image_mode_list;
        TYDisableComponents(hDevice, TY_COMPONENT_DEPTH_CAM);
    } else {
        mVideoMode[TY_COMPONENT_DEPTH_CAM] .clear();
    }

    if(allComps & TY_COMPONENT_IR_CAM_LEFT) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_IR_CAM_LEFT, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "left-IR stream: ");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                //uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_IR_CAM_LEFT] = image_mode_list;
        TYDisableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT);
    } else {
        mVideoMode[TY_COMPONENT_IR_CAM_LEFT].clear();
    }

    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_IR_CAM_RIGHT, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "right-IR stream: ");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                //uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_IR_CAM_RIGHT] = image_mode_list;
        TYDisableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT);
    } else {
        mVideoMode[TY_COMPONENT_IR_CAM_RIGHT].clear();
    }
}

static std::string WrapXML(const std::string& xml) {
    return std::string("<root>") + xml + "</root>";
}

static inline std::string xml_key_trim(const std::string& str) {
    auto start = str.find_first_not_of(' ');
    auto end = str.find_last_not_of(' ');
    return str.substr(start, end - start + 1);
}

int GigEBase::parse_xml_parameters(const std::string& xml)
{
    parameters.clear();
    std::string wrappedXML = WrapXML(xml);
    tinyxml2::XMLError err = m_doc.Parse(wrappedXML.c_str());
    if( err != tinyxml2::XML_SUCCESS ){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "xml parse failed");
        return -1;
    }
  
    tinyxml2::XMLElement* m_root = m_doc.RootElement();
    if(!m_root){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "found no root element");
        return -1;
    }
  
    for(auto elemSource = m_root->FirstChildElement("source"); 
                elemSource != NULL; elemSource = elemSource->NextSiblingElement("source")) {
        auto str = elemSource->Attribute("name");
        if(!str) continue;
      
        std::string source = xml_key_trim(std::string(str));
        for(auto elemFeat = elemSource->FirstChildElement("feature");
                elemFeat != NULL; elemFeat = elemFeat->NextSiblingElement("feature")) {
            
            auto sub_str = elemFeat->Attribute("name");
            auto text = elemFeat->GetText();
            if(sub_str && text) {
                std::string feat_name = xml_key_trim(std::string(sub_str));
                std::string val = xml_key_trim(std::string(text));
                parameters[source].push_back({feat_name, val});
            }
        }
    }

    device_load_parameters();

    return 0;
}

//percipio camera 初始化，打开相机,配置参数,使能数据流
PercipioDevice::PercipioDevice(const char* faceId, const char* deviceId)
    : alive(false),
      hIface(nullptr),
      handle(nullptr)
{
    TY_STATUS status = device_open(faceId, deviceId);
    if(TY_STATUS_OK == status) {
        strFaceId = faceId;
        strDeviceId = deviceId;
    }

    DepthDomainTimeFilterMgrPtr = std::make_unique<DepthTimeDomainMgr>(m_depth_time_domain_frame_num);
}

//设备离校重连
//此功能只有在开启设备自动重连 且相机发生事实离线问题时会被主动调用，
TY_STATUS  PercipioDevice::Reconnect()
{
    TY_STATUS status = device_open(strFaceId.c_str(), strDeviceId.c_str());
    if(TY_STATUS_OK != status) 
        return status;

    device_ros_event.eventId = (TY_EVENT)TY_EVENT_DEVICE_CONNECT;

    if(_event_callback)
        _event_callback(this, &device_ros_event);

    return TY_STATUS_OK;
}

TY_STATUS PercipioDevice::device_open(const char* faceId, const char* deviceId)
{
    TY_STATUS status;
    status = TYOpenInterface(faceId, &hIface);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Open interface fail : " << status);
        return status;
    }
  
    status = TYOpenDevice(hIface, deviceId, &handle);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Open device fail : " << status);
        TYCloseInterface(hIface);
        return status;
    }

    status = TYGetDeviceInfo(handle, &base_info);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid device handle  : " << status);
        TYCloseDevice(handle);
        TYCloseInterface(hIface);
        return status;
    }

    std::string str_gige_version;
    bool isNetDev = TYIsNetworkInterface(base_info.iface.type);
    if(isNetDev) {
        str_gige_version = base_info.netInfo.tlversion;
        if(str_gige_version == "Gige_2_1") {
            gige_version = GigeE_2_1;
        }
    } else {
        str_gige_version = base_info.usbInfo.tlversion;
        if(str_gige_version == "USB3Vision_1_2") {
            gige_version = GigeE_2_1;
        }
    }

    if(gige_version == GigeE_2_1)
        m_gige_dev = std::make_unique<GigE_2_1>(handle);
    else
        m_gige_dev = std::make_unique<GigE_2_0>(handle);

    m_gige_dev->init();

    



    device_ros_event.eventId = (TY_EVENT)TY_EVENT_DEVICE_CONNECT;

    if(_event_callback)
        _event_callback(this, &device_ros_event);
    
    alive = true;

    return TY_STATUS_OK;
}

//相机释放，停止拍照取图，关闭相机
void PercipioDevice::Release()
{
    stream_stop();

    TYCloseDevice(handle);
    handle = nullptr;

    TYCloseInterface(hIface);
    hIface = nullptr;
}

//析构，释放资源
PercipioDevice::~PercipioDevice()
{
    is_running_.store(false);
    if (frame_rate_ctrl_thread_ && frame_rate_ctrl_thread_->joinable()) {
        frame_rate_ctrl_thread_->join();
        frame_rate_ctrl_thread_ = nullptr;
    }

    if (frame_recive_thread_ && frame_recive_thread_->joinable()) {
        frame_recive_thread_->join();
        frame_recive_thread_ = nullptr;
    }

    alive = false;
    if (device_reconnect_thread && device_reconnect_thread->joinable()) {//
        device_reconnect_thread->join();
        device_reconnect_thread = nullptr;
    }

    TYCloseDevice(handle);
    handle = nullptr;

    TYCloseInterface(hIface);
    hIface = nullptr;
}

bool PercipioDevice::isAlive()
{
    return alive;
}

//SDK 事件回调函数，相机离线等问题时 会被SDK调用
static void eventCallback(TY_EVENT_INFO *event_info, void *userdata) {
    PercipioDevice* handle = (PercipioDevice*)userdata;

    handle->device_ros_event.eventId = event_info->eventId;
    if(handle->_event_callback)
        handle->_event_callback(handle, &handle->device_ros_event);

    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        handle->offline_detect_cond.notify_one();
    }
}

void PercipioDevice::registerCameraEventCallback(PercipioDeviceEventCallbackFunction callback)
{
    if(alive) {
       _event_callback = callback;
        TYRegisterEventCallback(handle, eventCallback, this);
   }
}

void PercipioDevice::setDeviceConfig(const std::string& config_xml)
{
    //TODO..
    m_gige_dev->parse_xml_parameters(config_xml);
}

//相机 serial number
std::string PercipioDevice::serialNumber()
{
    return std::string(base_info.id);
}

//相机model name
std::string PercipioDevice::modelName()
{
    return std::string(base_info.modelName);
}

//相机固件hash
std::string PercipioDevice::buildHash()
{
    return std::string(base_info.buildHash);
}

//相机config 版本
std::string PercipioDevice::configVersion()
{
    return std::string(base_info.configVersion);
}

//相机组件查询
bool PercipioDevice::hasColor()
{
    return (m_gige_dev->streams() & TY_COMPONENT_RGB_CAM) == 
            TY_COMPONENT_RGB_CAM;
}

bool PercipioDevice::hasDepth()
{
    return (m_gige_dev->streams() & TY_COMPONENT_DEPTH_CAM) == 
            TY_COMPONENT_DEPTH_CAM;
}

bool PercipioDevice::hasLeftIR()
{
    return (m_gige_dev->streams() & TY_COMPONENT_IR_CAM_LEFT) == 
            TY_COMPONENT_IR_CAM_LEFT;
}

bool PercipioDevice::hasRightIR()
{
    return (m_gige_dev->streams() & TY_COMPONENT_IR_CAM_RIGHT) == 
            TY_COMPONENT_IR_CAM_RIGHT;
}

//创建相机离线重现监测函数
void PercipioDevice::enable_offline_reconnect(const bool en) 
{ 
    b_dev_auto_reconnect = en; 
    if(!b_dev_auto_reconnect)
        return;
    if(device_reconnect_thread)
        return;
    device_reconnect_thread = std::make_unique<std::thread>([this]() { device_offline_reconnect(); });
}

void PercipioDevice::frame_rate_init(const bool en, const float fps)
{
    b_dev_frame_rate_ctrl_en = en;
    f_dev_frame_rate = fps;
}

//laser亮度
bool PercipioDevice::set_laser_power(const int power)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;
    status = TYSetInt(handle, TY_COMPONENT_LASER, TY_INT_LASER_POWER, power);
    if(status != TY_STATUS_OK) return false;
    return true;
}

//解析用户设置的stream 数据流
bool PercipioDevice::resolveStreamResolution(const std::string& resolution_, uint32_t& width, uint32_t& height)
{
  size_t pos = resolution_.find('x');
  if((pos != 0) && (pos != std::string::npos))
  {
    std::string str_width = resolution_.substr(0, pos);
    std::string str_height = resolution_.substr(pos+1, resolution_.length());
    width = static_cast<uint32_t>(atoi(str_width.c_str()));
    height = static_cast<uint32_t>(atoi(str_height.c_str()));
    return true;
  }
  return false;
}

std::string PercipioDevice::parseStreamFormat(const std::string& format)
{
    std::string fmt = format;
    std::transform(fmt.begin(), fmt.end(), fmt.begin(),[](unsigned char c) { return std::tolower(c); });
    if(fmt.find("mono") != std::string::npos) return "mono";
    if(fmt.find("bayer") != std::string::npos) return "bayer";

    if(fmt.find("yuv") != std::string::npos) return "yuv";
    if(fmt.find("ycbcr") != std::string::npos) return "yuv";
    if(fmt.find("yuyv") != std::string::npos) return "yuv";
    if(fmt.find("yvyu") != std::string::npos) return "yuv";
    
    if(fmt.find("jpeg") != std::string::npos) return "jpeg";

    if(fmt.find("depth") != std::string::npos) return "depth16";
    if(fmt.find("coord3d_c16") != std::string::npos) return "depth16";
    
    if(fmt.find("xyz48") != std::string::npos) return "xyz48";
    if(fmt.find("coord3d_abc16") != std::string::npos) return "xyz48";

    if(fmt.find("coord3d_abc32f") != std::string::npos) return "point3d";

    if(fmt.find("bgr") != std::string::npos) return "bgr";
    if(fmt.find("rgb") != std::string::npos) return "rgb";
    
    return std::string();
}

//读取相机深度图的单位
float PercipioDevice::getDepthValueScale()
{
    return f_scale_unit;
}

//开启指定数据流
bool PercipioDevice::stream_open(const percipio_stream_index_pair& idx, const std::string& resolution, const std::string& format)
{
    TY_STATUS status;
    uint32_t m_comp = StreamConvertComponent(idx);
    if(INVALID_COMPONENT_ID == m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid stream!");
        return false;
    }

    if((m_comp & m_gige_dev->streams()) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported component: " << m_comp);
        return false;
    }

    uint32_t img_width, img_height;
    std::vector<percipio_video_mode> video_mode_val_list(0);
    bool valid_resolution = resolveStreamResolution(resolution, img_width, img_height);
    auto VideoModeList = m_gige_dev->mVideoMode[m_comp];
    if(VideoModeList.size()) {
        for(auto video_mode : VideoModeList) {
            if(valid_resolution) {
                if(img_width != video_mode.width || img_height != video_mode.height) {
                    continue;
                }
            }

            if(format.length()) {
                if(format != parseStreamFormat(video_mode.desc)) {
                    continue;
                }
            }

            video_mode_val_list.push_back(video_mode);
        }
    }

    if(valid_resolution) {
        if(video_mode_val_list.size()) {
            status = m_gige_dev->image_mode_cfg(m_comp, video_mode_val_list[0]);
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Stream mode(" << StreamConvertSourceDesc(idx) << ") init error: " << status);
            } else {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set stream mode(" << StreamConvertSourceDesc(idx) << "): " << video_mode_val_list[0].desc << " " 
                        << video_mode_val_list[0].width << "x" << video_mode_val_list[0].height);
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported stream mode(" << StreamConvertSourceDesc(idx) << "): " << resolution);
        }
    }

    status = TYEnableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Stream(" << StreamConvertSourceDesc(idx) << ") open error: " << status);
        return false;
    }

    switch (m_comp) {
        case TY_COMPONENT_DEPTH_CAM: {
            m_gige_dev->depth_scale_unit_init(f_scale_unit);
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth stream scale unit: " << f_scale_unit);
            status = m_gige_dev->stream_calib_data_init(TY_COMPONENT_DEPTH_CAM, cam_depth_calib_data);
            if(status != TY_STATUS_OK) {
                has_depth_calib_data = false;
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Got depth stream calib data error: " << status);
            } else {
                has_depth_calib_data = true;
                cam_depth_intrinsic = image_intrinsic(cam_depth_calib_data.intrinsicWidth, cam_depth_calib_data.intrinsicHeight, cam_depth_calib_data.intrinsic);
            }

            m_gige_dev->depth_stream_distortion_check(b_need_do_depth_undistortion);
            break;
        }
        case TY_COMPONENT_RGB_CAM: {
            status = m_gige_dev->stream_calib_data_init(TY_COMPONENT_RGB_CAM, cam_color_calib_data);
            if(status != TY_STATUS_OK) {
                has_color_calib_data = false;
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Got color stream calib data error:" << status);
            } else {
                has_color_calib_data = true;
                cam_color_intrinsic = image_intrinsic(cam_color_calib_data.intrinsicWidth, cam_color_calib_data.intrinsicHeight, cam_color_calib_data.intrinsic);
            }
            break;
        }
        case TY_COMPONENT_IR_CAM_LEFT: {
            TY_CAMERA_CALIB_INFO calib_data;
            m_gige_dev->stream_calib_data_init(TY_COMPONENT_IR_CAM_LEFT, calib_data);
            cam_leftir_intrinsic = image_intrinsic(calib_data.intrinsicWidth, calib_data.intrinsicHeight, calib_data.intrinsic);
            break;
        }
        case TY_COMPONENT_IR_CAM_RIGHT: {
            TY_CAMERA_CALIB_INFO calib_data;
            m_gige_dev->stream_calib_data_init(TY_COMPONENT_IR_CAM_RIGHT, calib_data);
            cam_rightir_intrinsic = image_intrinsic(calib_data.intrinsicWidth, calib_data.intrinsicHeight, calib_data.intrinsic);
            break;
        }
        default:
            break;
    }

    if(!reconnect) m_streams.push_back({idx, resolution, format});
    VideoStreamPtr = std::make_unique<VideoStream>();
    return true;
}

//关闭指定数据流
bool PercipioDevice::stream_close(const percipio_stream_index_pair& idx)
{
    TY_STATUS status;
    uint32_t m_comp = StreamConvertComponent(idx);
    if(INVALID_COMPONENT_ID == m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid stream!");
        return false;
    }

    if((m_comp & m_gige_dev->streams()) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported component: " << m_comp);
        return false;
    }

    status = TYDisableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Stream close error: " << status);
        return false;
    }

    return true;
}

//
void PercipioDevice::colorStreamReceive(const TYImage& color, uint64_t& timestamp)
{
    TYImage targetRGB;

    if(color.empty()) return;
    if(VideoStreamPtr) {
        if(has_color_calib_data) {
            if(color.format() == TYPixelFormatBGR8) {
                targetRGB = color.clone();

                TY_IMAGE_DATA src;
                src.width = color.width();
                src.height = color.height();
                src.size = color.width() * color.height() * 3;
                src.pixelFormat = TYPixelFormatBGR8;
                src.buffer = (void*)color.data();

                TY_IMAGE_DATA dst;
                dst.width = color.width();
                dst.height = color.height();
                dst.size = color.width() * color.height() * 3;
                dst.pixelFormat = TYPixelFormatBGR8;
                dst.buffer = (void*)targetRGB.data();
                TY_STATUS err = TYUndistortImage(&cam_color_calib_data, &src, NULL, &dst);
                if(err) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Faild to do color undistortImage, error: " << err);
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid color stream fmt: " << color.format());
                return;
            }
        } else {
            targetRGB = color;
        }
        VideoStreamPtr->ColorInit(targetRGB, cam_color_intrinsic, timestamp);
    }
}

void PercipioDevice::leftIRStreamReceive(const TYImage& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRLeftInit(ir, cam_leftir_intrinsic, timestamp);
}

void PercipioDevice::rightIRStreamReceive(const TYImage& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRRightInit(ir, cam_rightir_intrinsic, timestamp);
}

void PercipioDevice::depthStreamReceive(TYImage& depth, uint64_t& timestamp)
{
    TYImage targetDepth;
    if(depth.empty()) return;
    if(!VideoStreamPtr) return;

    if(depth.format() == TYPixelFormatCoord3D_C16) {
        if(b_need_do_depth_undistortion) {
            targetDepth = depth.clone();

            TY_IMAGE_DATA src;
            src.width = depth.width();
            src.height = depth.height();
            src.size = depth.width() * depth.height() * 2;
            src.pixelFormat = TYPixelFormatCoord3D_C16;
            src.buffer = depth.data();

            TY_IMAGE_DATA dst;
            dst.width = depth.width();
            dst.height = depth.height();
            dst.size = depth.width() * depth.height() * 2;
            dst.pixelFormat = TYPixelFormatCoord3D_C16;
            dst.buffer = targetDepth.data();
            
            TY_STATUS err = TYUndistortImage(&cam_depth_calib_data, &src, NULL, &dst);
            if(err) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Faild to do depth undistortImage, error: " << err);
        } else {
            targetDepth = depth;
        }

        if(topics_d_registration_) {
            TYImage out = TYImage(targetDepth.width(), targetDepth.height(), TYPixelFormatCoord3D_C16);
            TYMapDepthImageToColorCoordinate(&cam_depth_calib_data,
                targetDepth.width(), targetDepth.height(), (const uint16_t*)targetDepth.data(),
                &cam_color_calib_data,
                out.width(), out.height(), (uint16_t*)out.data(), f_scale_unit);
            
            targetDepth = out.clone();
            VideoStreamPtr->DepthInit(targetDepth, cam_color_intrinsic, timestamp);
        } else if(topics_depth_) {
            VideoStreamPtr->DepthInit(targetDepth, cam_depth_intrinsic, timestamp);
        }
    } else if(depth.format() == TYPixelFormatCoord3D_ABC16) {
        if(topics_d_registration_) {
            TYImage p3d = TYImage(depth.width(), depth.height(), TYPixelFormatCoord3D_ABC32f);
            int16_t* src = (int16_t*)depth.data();
            float* dst = (float*)p3d.data();
            for(int i = 0; i < depth.height(); i++) {
                for(int j = 0; j < depth.width(); j++) {
                    if(src[3*i*depth.width() + 3*j + 2]) {
                        dst[3*i*depth.width() + 3*j + 0] = src[3*i*depth.width() + 3*j + 0];
                        dst[3*i*depth.width() + 3*j + 1] = src[3*i*depth.width() + 3*j + 1];
                        dst[3*i*depth.width() + 3*j + 2] = src[3*i*depth.width() + 3*j + 2];
                    } else {
                        dst[3*i*depth.width() + 3*j + 0] = std::numeric_limits<float>::quiet_NaN();
                        dst[3*i*depth.width() + 3*j + 1] = std::numeric_limits<float>::quiet_NaN();
                        dst[3*i*depth.width() + 3*j + 2] = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }

            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data(), p3d.width() * p3d.height(), (TY_VECT_3F*)p3d.data());

            targetDepth = TYImage(depth.width(), depth.height(), TYPixelFormatCoord3D_C16);
            TYMapPoint3dToDepthImage(&cam_color_calib_data, (const TY_VECT_3F*)(p3d.data()), p3d.width() * p3d.height(), p3d.width(), p3d.height(), (uint16_t*)(targetDepth.data()));
            VideoStreamPtr->DepthInit(targetDepth, cam_color_intrinsic,timestamp);
        } else if(topics_depth_) {
            targetDepth = depth;
            VideoStreamPtr->DepthInit(targetDepth, cam_depth_intrinsic, timestamp);
        }
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid depth stream fmt: " << depth.format());
        return;
    }

    depth = targetDepth.clone();
    return;
}

void PercipioDevice::p3dStreamReceive(const TYImage& depth, uint64_t& timestamp) {
    if(depth.empty()) return;
    if(!topics_p3d_  && !topics_color_p3d_) return;
    if(!VideoStreamPtr) return;

    TYImage p3d = TYImage(depth.width(), depth.height(), TYPixelFormatCoord3D_ABC32f);
    if(depth.format() == TYPixelFormatCoord3D_C16) {
        TYImage targetDepth;
        if(b_need_do_depth_undistortion && !topics_d_registration_) {
            targetDepth = depth.clone();
    
            TY_IMAGE_DATA src;
            src.width = depth.width();
            src.height = depth.height();
            src.size = depth.width() * depth.height() * 2;
            src.pixelFormat = TYPixelFormatCoord3D_C16;
            src.buffer = depth.data();
    
            TY_IMAGE_DATA dst;
            dst.width = depth.width();
            dst.height = depth.height();
            dst.size = depth.width() * depth.height() * 2;
            dst.pixelFormat = TYPixelFormatCoord3D_C16;
            dst.buffer = targetDepth.data();
            TY_STATUS err = TYUndistortImage(&cam_depth_calib_data, &src, NULL, &dst);
            if(err) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Faild to do depth undistortImage, error: " << err);
            }
        } else {
            targetDepth = depth;
        }
        
        if(topics_color_p3d_) {
            TYMapDepthImageToPoint3d(&cam_color_calib_data, targetDepth.width(), targetDepth.height(), (const uint16_t*)targetDepth.data(), (TY_VECT_3F*)p3d.data(), f_scale_unit);
            VideoStreamPtr->PointCloudInit(p3d, cam_color_intrinsic, timestamp);
        } else if(topics_p3d_) {
            TYMapDepthImageToPoint3d(&cam_depth_calib_data, targetDepth.width(), targetDepth.height(), (const uint16_t*)targetDepth.data(), (TY_VECT_3F*)p3d.data(), f_scale_unit);
            VideoStreamPtr->PointCloudInit(p3d, cam_depth_intrinsic, timestamp);
        }
    } else if(depth.format() == TYPixelFormatCoord3D_ABC16) {
        int16_t* src = (int16_t*)depth.data();
        float* dst = (float*)p3d.data();
        for(int i = 0; i < depth.height(); i++) {
            for(int j = 0; j < depth.width(); j++) {
                if(src[3*i*depth.width() + 3*j + 2]) {
                    dst[3*i*depth.width() + 3*j + 0] = src[3*i*depth.width() + 3*j + 0];
                    dst[3*i*depth.width() + 3*j + 1] = src[3*i*depth.width() + 3*j + 1];
                    dst[3*i*depth.width() + 3*j + 2] = src[3*i*depth.width() + 3*j + 2];
                } else {
                    dst[3*i*depth.width() + 3*j + 0] = std::numeric_limits<float>::quiet_NaN();
                    dst[3*i*depth.width() + 3*j + 1] = std::numeric_limits<float>::quiet_NaN();
                    dst[3*i*depth.width() + 3*j + 2] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        if(topics_p3d_) {
            VideoStreamPtr->PointCloudInit(p3d, cam_depth_intrinsic, timestamp);
        } else if(topics_color_p3d_) {
            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data(), p3d.width() * p3d.height(), (TY_VECT_3F*)p3d.data());
            VideoStreamPtr->PointCloudInit(p3d, cam_color_intrinsic, timestamp);
        }
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid depth stream fmt: " << depth.format());
    }
}

//相机离线监测
void PercipioDevice::device_offline_reconnect() {
    while(isAlive()) {
        //TODO
        std::unique_lock<std::mutex> lck(offline_detect_mutex);
        offline_detect_cond.wait(lck);
        reconnect = true;
        Release();
        while(true) {
            TY_STATUS status = Reconnect();
            if(status == TY_STATUS_OK) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device Offline, reconnect ok, now restart stream!");
                _node->setupDevices();
                reconnect = false;
                break;
            }
            MSLEEP(1000);
        }
    }
}

void PercipioDevice::softTriggerSend() {
    const float fps = m_gige_dev->PeriodicSoftTriggerFpS();

    while (rclcpp::ok() && is_running_.load()) {
        int delay = (int)(1000 / fps);
        uint64_t trig_before = getSystemTime();
        TY_STATUS rc = m_gige_dev->send_soft_trigger_signal();
        if(rc) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to send soft trigger signal!");
        }
        uint64_t trig_after = getSystemTime();

        int trig_time = static_cast<int>(trig_after - trig_before);
        int delt = delay > trig_time ? (delay - trig_time) : 0;

        if(delt) {
            if(delt<60)
            {
                delt=60;
            }
            MSLEEP(delt);
        } else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Trigger signal timeout!");
        }
    }
}

void PercipioDevice::frameDataReceive() {
    TY_STATUS status;
    //m_softtrigger_ready = false;

    switch(workmode) {
        case CONTINUS:
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device now work at continuous mode.");
            break;
        case SOFTTRIGGER:
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device now work at soft trigger mode.");
            break;
        case HARDTRIGGER:
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device now work at hard trigger mode.");
            break;
        default:
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device now work at invalid workmode.");
            break;
    }

    fps_counter.reset();
    while (rclcpp::ok() && is_running_.load()) {
        TY_FRAME_DATA frame;
        if(workmode == CONTINUS || workmode == HARDTRIGGER) {
            status = TYFetchFrame(handle, &frame, 2000);
        } else if(workmode == SOFTTRIGGER) {
            status = TYFetchFrame(handle, &frame, 200);
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid workmode: " << workmode);
            status = TYFetchFrame(handle, &frame, 2000);
        }

        if(status == TY_STATUS_OK) {
            float fps = fps_counter.get_fps();
            if(fps > 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "fps = " << fps);
            }

            for (int i = 0; i < frame.validCount; i++){
                if (frame.image[i].status != TY_STATUS_OK) continue;

                if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
                    TYImage depth;
                    if(frame.image[i].pixelFormat == TYPixelFormatCoord3D_C16) {
                        uint16_t* ptrDepth = static_cast<uint16_t*>(frame.image[i].buffer);
                        int32_t PixsCnt = frame.image[i].width * frame.image[i].height;
                        for(int32_t i = 0; i < PixsCnt; i++) {
                            if(ptrDepth[i] == 0xFFFF) ptrDepth[i] = 0;
                        }

                        if(b_depth_spk_filter_en) {
                            DepthSpeckleFilterParameters param = {m_depth_spk_size, m_depth_spk_diff, f_depth_spk_phy_size};
                            TYDepthSpeckleFilter(&frame.image[i], &param, &cam_depth_calib_data, f_scale_unit);
                        }

                        if(b_depth_time_domain_en) {
                            DepthDomainTimeFilterMgrPtr->add_frame(frame.image[i]);
                            if(!DepthDomainTimeFilterMgrPtr->do_time_domain_process(frame.image[i])) {
                                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Do Time-domain filter, drop frame!");
                                continue;
                            }
                        }
                        depth = TYImage(frame.image[i].width, frame.image[i].height, TYPixelFormatCoord3D_C16, frame.image[i].buffer);
                    } else if((uint32_t)frame.image[i].pixelFormat == TYPixelFormatCoord3D_ABC16) {
                        depth = TYImage(frame.image[i].width, frame.image[i].height, TYPixelFormatCoord3D_ABC16, frame.image[i].buffer);
                    }
                    depthStreamReceive(depth, frame.image[i].timestamp);
                    p3dStreamReceive(depth, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_RGB_CAM) {
                    uint32_t destSize;
                    TYImage color;
                    std::vector<uint8_t> image_data;
                    TYImageInfo image_info = ty_image_info(frame.image[i]);
                    TYDecodeError err = TYGetDecodeBufferSize(&image_info, &destSize, TY_OUTPUT_FORMAT_AUTO);
                    if(err == TY_DECODE_SUCCESS) {
                        TYDecodeResult retInfo;
                        image_data.resize(destSize);
                        TYDecodeImage(&image_info,  TY_OUTPUT_FORMAT_AUTO, (void*)&image_data[0], destSize, &retInfo);
                        color = TYImage(frame.image[i].width, frame.image[i].height, retInfo.format, &image_data[0]);
                    } else {
                        color = TYImage(frame.image[i].width, frame.image[i].height, frame.image[i].pixelFormat, frame.image[i].buffer);
                    }
                    
                    colorStreamReceive(color, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT) {
                    uint32_t destSize;
                    TYImage leftIR;
                    std::vector<uint8_t> image_data;
                    TYImageInfo image_info = ty_image_info(frame.image[i]);
                    TYDecodeError err = TYGetDecodeBufferSize(&image_info, &destSize, TY_OUTPUT_FORMAT_AUTO);
                    if(err == TY_DECODE_SUCCESS) {
                        TYDecodeResult retInfo;
                        image_data.resize(destSize);
                        TYDecodeImage(&image_info,  TY_OUTPUT_FORMAT_AUTO, (void*)&image_data[0], destSize, &retInfo);
                        leftIR = TYImage(frame.image[i].width, frame.image[i].height, retInfo.format, &image_data[0]);
                    } else {
                        leftIR = TYImage(frame.image[i].width, frame.image[i].height, frame.image[i].pixelFormat, frame.image[i].buffer);
                    }

                    leftIRStreamReceive(leftIR, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT) {
                    uint32_t destSize;
                    TYImage rightIR;
                    std::vector<uint8_t> image_data;
                    TYImageInfo image_info = ty_image_info(frame.image[i]);
                    TYDecodeError err = TYGetDecodeBufferSize(&image_info, &destSize, TY_OUTPUT_FORMAT_AUTO);
                    if(err == TY_DECODE_SUCCESS) {
                        TYDecodeResult retInfo;
                        image_data.resize(destSize);
                        TYDecodeImage(&image_info,  TY_OUTPUT_FORMAT_AUTO, (void*)&image_data[0], destSize, &retInfo);
                        rightIR = TYImage(frame.image[i].width, frame.image[i].height, retInfo.format, &image_data[0]);
                    } else {
                        rightIR = TYImage(frame.image[i].width, frame.image[i].height, frame.image[i].pixelFormat, frame.image[i].buffer);
                    }
                    rightIRStreamReceive(rightIR, frame.image[i].timestamp);
                }
            }

            if(_callback)
               _callback(*VideoStreamPtr.get());

            TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
        }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "frameDataReceive exit...");
}

//开启数据流
bool PercipioDevice::stream_start()
{
    if(b_dev_frame_rate_ctrl_en) {
        if(workmode != CONTINUS) {
            workmode = CONTINUS;
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The device has enabled the fixed frame rate output mode, the operating mode is force-switched to continuous output mode.");
        }
    }
    m_gige_dev->work_mode_init(workmode, b_dev_frame_rate_ctrl_en, f_dev_frame_rate);
    
    uint32_t frameSize;
    TY_STATUS status = TYGetFrameBufferSize(handle, &frameSize);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to get frame buffer size, error: " << status);
        return false;
    }
    frameBuffer[0].resize(frameSize);
    frameBuffer[1].resize(frameSize);
    
    TYEnqueueBuffer(handle, frameBuffer[0].data(), frameSize);
    TYEnqueueBuffer(handle, frameBuffer[1].data(), frameSize);

    status = TYStartCapture(handle);
    if(status != TY_STATUS_OK) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed  to Start capture, error: " << status);
      return false;
    }

    is_running_.store(true);
    if(m_gige_dev->PeriodicSoftTriggerEnable()) {
        frame_rate_ctrl_thread_ = std::make_unique<std::thread>([this]() { softTriggerSend(); });
    }

    frame_recive_thread_ = std::make_unique<std::thread>([this]() { frameDataReceive(); });

    return true;
}

bool PercipioDevice::stream_stop()
{
    if(!is_running_.load()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The camera's not on yet!");
        return false;
    }

    is_running_.store(false);
    if (frame_rate_ctrl_thread_ && frame_rate_ctrl_thread_->joinable()) {
        frame_rate_ctrl_thread_->join();
        frame_rate_ctrl_thread_ = nullptr;
    }

    if (frame_recive_thread_ && frame_recive_thread_->joinable()) {
        frame_recive_thread_->join();
        frame_recive_thread_ = nullptr;
    }

    TYStopCapture(handle);
    TYClearBufferQueue(handle);
    frameBuffer[0].clear();
    frameBuffer[1].clear();

    return true;
}

void PercipioDevice::send_softtrigger()
{
    if(workmode != SOFTTRIGGER) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The camera is not working in soft trigger mode, ignore trigger signal");
        return;
    }
    m_gige_dev->send_soft_trigger_signal();
}

void PercipioDevice::setFrameCallback(FrameCallbackFunction callback)
{
    _callback = callback;
}

void PercipioDevice::topics_depth_stream_enable(bool enable)
{
    topics_depth_ = enable;
}

void PercipioDevice::topics_point_cloud_enable(bool enable)
{
    topics_p3d_= enable;
}

void PercipioDevice::topics_color_point_cloud_enable(bool enable)
{
    topics_color_p3d_= enable;
}

void PercipioDevice::topics_depth_registration_enable(bool enable)
{
    topics_d_registration_= enable;
}

void PercipioDevice::depth_speckle_filter_init(bool enable, int spec_size, int spec_diff, float phy_size)
{
    b_depth_spk_filter_en = enable;
    m_depth_spk_size = spec_size;
    m_depth_spk_diff = spec_diff;
    f_depth_spk_phy_size = phy_size;
}

void PercipioDevice::dpeth_time_domain_filter_init(bool enable, int number)
{
    b_depth_time_domain_en = enable;
    m_depth_time_domain_frame_num = number;
    DepthDomainTimeFilterMgrPtr->reset(m_depth_time_domain_frame_num);
}

}
