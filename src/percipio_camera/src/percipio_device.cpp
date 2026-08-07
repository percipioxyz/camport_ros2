#include "percipio_device.h"
#include "TYCoordinateMapper.h"
#include "TYImageProc.h"

#include "percipio_camera_node.h"
#include "Utils.hpp"

#include "gige_2_0.h"
#include "gige_2_1.h"

#include "percipio_image_process.hpp"

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
    case IR_LEFT:
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
    case IR_LEFT:
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

void GigEBase::init_component_video_mode(TY_COMPONENT_ID comp, const char* stream_desc)
{
    if(!(allComps & comp)) {
        mVideoMode[comp].clear();
        return;
    }

    std::vector<percipio_video_mode> image_mode_list;
    dump_image_mode_list(comp, image_mode_list);
    if(!image_mode_list.empty()) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Available video modes for " << stream_desc << " stream:");
        for(const auto& mode : image_mode_list) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE),
                "        " << mode.desc << " " << mode.width << "x" << mode.height);
        }
    }

    mVideoMode[comp] = image_mode_list;
    TYDisableComponents(hDevice, comp);
}

void GigEBase::video_mode_init()
{
    init_component_video_mode(TY_COMPONENT_RGB_CAM,      "color");
    init_component_video_mode(TY_COMPONENT_DEPTH_CAM,    "depth");
    init_component_video_mode(TY_COMPONENT_IR_CAM_LEFT,  "left-IR");
    init_component_video_mode(TY_COMPONENT_IR_CAM_RIGHT, "right-IR");
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
    percipio_ros2_tinyxml2::XMLError err = m_doc.Parse(wrappedXML.c_str());
    if( err != percipio_ros2_tinyxml2::XML_SUCCESS ){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "XML configuration parse error");
        return -1;
    }
  
    percipio_ros2_tinyxml2::XMLElement* m_root = m_doc.RootElement();
    if(!m_root){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "XML configuration missing root element");
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to open interface: " << status);
        return status;
    }

    status = TYUpdateDeviceList(hIface);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to update device list: " << status);
        return status;
    }
  
    status = TYOpenDevice(hIface, deviceId, &handle);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Open device fail : " << status);
        TYCloseInterface(hIface);
        return status;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device opened successfully: " << deviceId);

    status = TYGetDeviceInfo(handle, &base_info);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to retrieve device info: " << status);
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

    auto comps = m_gige_dev->streams();
    if(comps & TY_COMPONENT_DEPTH_CAM) {
        status = m_gige_dev->stream_calib_data_init(TY_COMPONENT_DEPTH_CAM, cam_depth_calib_data);
        if(status != TY_STATUS_OK) {
            has_depth_calib_data = false;
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to read depth calibration data: " << status);
        } else {
            has_depth_calib_data = true;
            cam_depth_intrinsic = image_intrinsic(cam_depth_calib_data.intrinsicWidth, cam_depth_calib_data.intrinsicHeight, cam_depth_calib_data.intrinsic);
        }

        m_gige_dev->depth_stream_distortion_check(b_need_do_depth_undistortion);
    }

    if(comps & TY_COMPONENT_RGB_CAM) {
        status = m_gige_dev->stream_calib_data_init(TY_COMPONENT_RGB_CAM, cam_color_calib_data);
        if(status != TY_STATUS_OK) {
            has_color_calib_data = false;
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to read color calibration data: " << status);
        } else {
            has_color_calib_data = true;
            cam_color_intrinsic = image_intrinsic(cam_color_calib_data.intrinsicWidth, cam_color_calib_data.intrinsicHeight, cam_color_calib_data.intrinsic);
        }
    }

    if(comps & TY_COMPONENT_IR_CAM_LEFT) {
        m_gige_dev->stream_calib_data_init(TY_COMPONENT_IR_CAM_LEFT, cam_left_ir_calib_data);
        cam_leftir_intrinsic = image_intrinsic(cam_left_ir_calib_data.intrinsicWidth, cam_left_ir_calib_data.intrinsicHeight, cam_left_ir_calib_data.intrinsic);
    }

    if(comps & TY_COMPONENT_IR_CAM_RIGHT) {
        m_gige_dev->stream_calib_data_init(TY_COMPONENT_IR_CAM_RIGHT, cam_right_ir_calib_data);
        cam_rightir_intrinsic = image_intrinsic(cam_right_ir_calib_data.intrinsicWidth, cam_right_ir_calib_data.intrinsicHeight, cam_right_ir_calib_data.intrinsic);
    }

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
        handle->b_offline_event_pending.store(true);
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

void PercipioDevice::reset()
{
    m_gige_dev->reset();
    MSLEEP(2000);
    offline_detect_cond.notify_one();
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid stream identifier");
        return false;
    }

    if((m_comp & m_gige_dev->streams()) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Component not supported by device: " << m_comp);
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
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to configure " << StreamConvertSourceDesc(idx) << " stream mode: " << status);
            } else {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Stream mode configured — " << StreamConvertSourceDesc(idx) << ": " << video_mode_val_list[0].desc << " " 
                        << video_mode_val_list[0].width << "x" << video_mode_val_list[0].height);
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported " << StreamConvertSourceDesc(idx) << " stream resolution: " << resolution);
        }
    }

    status = TYEnableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to enable " << StreamConvertSourceDesc(idx) << " stream: " << status);
        return false;
    }

    if(m_comp == TY_COMPONENT_DEPTH_CAM) {
        m_gige_dev->depth_scale_unit_init(f_scale_unit);
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth scale unit: " << f_scale_unit);
    }

    if((m_comp == TY_COMPONENT_IR_CAM_LEFT) || (m_comp == TY_COMPONENT_IR_CAM_RIGHT)) {
        if(b_do_ir_undist) {
            if(m_comp == TY_COMPONENT_IR_CAM_LEFT) cam_leftir_intrinsic = cam_depth_intrinsic;
            if(m_comp == TY_COMPONENT_IR_CAM_RIGHT) cam_rightir_intrinsic = cam_depth_intrinsic;
            
            status = m_gige_dev->EnableHwIRUndistortion();
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The device does not support self-rectification of IR images.");

                m_gige_dev->getIRLensType(ir_len_type);
                m_gige_dev->getIRRectificationMode(ir_rectificatio_mode);

                if(ir_rectificatio_mode == EPIPOLAR_RECTIFICATION) {
                    m_gige_dev->getLeftIRRotation(left_ir_rotation);
                    m_gige_dev->getRightIRRotation(right_ir_rotation);
                    m_gige_dev->getLeftIRRectifiedIntr(left_ir_rectified_intr);
                    m_gige_dev->getRightIRRectifiedIntr(right_ir_rectified_intr);
                }
                b_enable_sw_ir_undistortion = true;
            } else {
                b_enable_sw_ir_undistortion = false;
            }
        } else {
            b_enable_sw_ir_undistortion = false;
        }
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid stream identifier");
        return false;
    }

    if((m_comp & m_gige_dev->streams()) != m_comp) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Component not supported by device: 0x" << std::hex << m_comp << std::dec);
        return false;
    }

    status = TYDisableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to disable stream: " << status);
        return false;
    }

    return true;
}

//
void PercipioDevice::colorStreamReceive(const TYImage& color, uint64_t& timestamp, const TY_CAMERA_CALIB_INFO& calib, image_intrinsic& intr)
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
                TY_STATUS err = TYUndistortImage(&calib, &src, NULL, &dst);
                if(err) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Color undistortion failed: " << err);
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported color pixel format: " << color.format());
                return;
            }
        } else {
            targetRGB = color;
        }
        VideoStreamPtr->ColorInit(targetRGB, intr, timestamp);
    }
}

TY_CAMERA_CALIB_INFO PercipioDevice::adjustCalibByBinning(const TY_CAMERA_CALIB_INFO& src_calib, const TY_COMPONENT_ID comp)
{
    TY_CAMERA_CALIB_INFO dst_calib;
    float binning = m_gige_dev->get_stream_binning(comp);
    if(binning < 0) binning = 1.f;
    int32_t binX = static_cast<int32_t>(binning);
    int32_t binY = static_cast<int32_t>(binning);

    int32_t cropX = 0;
    int32_t cropY = 0;
    int32_t width = static_cast<int32_t>(src_calib.intrinsicWidth  / binning);
    int32_t height = static_cast<int32_t>(src_calib.intrinsicHeight / binning);
    TY_STATUS status = TYAdjustCalibInfoByBinningCrop(
        &src_calib, binX, binY,
        cropX, cropY,
        width, height, &dst_calib);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE),
            "Calibration binning adjustment failed (comp=0x" << std::hex << comp << std::dec
            << ", bin=" << binX << "x" << binY
            << ", crop=" << cropX << "," << cropY
            << ", size=" << width << "x" << height
            << "): " << status << "), using unadjusted calibration");
        return src_calib;
    }
    return dst_calib;
}


TY_CAMERA_CALIB_INFO PercipioDevice::adjustCalibByBinningCrop(const TY_CAMERA_CALIB_INFO& src_calib, const TY_COMPONENT_ID comp, const TY_IMAGE_DATA& image_data)
{
    TY_CAMERA_CALIB_INFO dst_calib;
    float binning = m_gige_dev->get_stream_binning(comp);
    if(binning < 0) binning = 1;
    int32_t binX = static_cast<int32_t>(binning);
    int32_t binY = static_cast<int32_t>(binning);

    int32_t cropX = image_data.cropOffsetX;
    int32_t cropY = image_data.cropOffsetY;
    int32_t width = image_data.width;
    int32_t height = image_data.height;

    TY_STATUS status = TYAdjustCalibInfoByBinningCrop(
        &src_calib, binX, binY,
        cropX, cropY,
        width, height, &dst_calib);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE),
            "Calibration binning/crop adjustment failed (comp=0x" << std::hex << comp << std::dec
            << ", bin=" << binX << "x" << binY
            << ", crop=" << cropX << "," << cropY
            << ", size=" << width << "x" << height
            << "): " << status << "), using unadjusted calibration");
        return src_calib;
    }
    return dst_calib;
}

TY_CAMERA_INTRINSIC PercipioDevice::adjustIntrinsicByBinningCrop(const TY_CAMERA_INTRINSIC& src_intr, TY_COMPONENT_ID comp, const TY_IMAGE_DATA& image_data, const bool crop)
{
    float binning = m_gige_dev->get_stream_binning(comp);
    if(binning < 0) binning = 1.f;
    float fcropX = crop ? static_cast<float>(image_data.cropOffsetX) : 0.0f;
    float fcropY = crop ? static_cast<float>(image_data.cropOffsetY) : 0.0f;
    TY_CAMERA_INTRINSIC dst_intr;
    memcpy(dst_intr.data, src_intr.data, sizeof(dst_intr.data));
    dst_intr.data[0] = src_intr.data[0] / binning;
    dst_intr.data[2] = src_intr.data[2] / binning - fcropX;
    dst_intr.data[4] = src_intr.data[4] / binning;
    dst_intr.data[5] = src_intr.data[5] / binning - fcropY;
    return dst_intr;
}

TY_STATUS PercipioDevice::IREnhancement(TYImage& IR)
{
    switch(enhance_mode) {
        case IREnhanceOFF:                  return TY_STATUS_OK;
        case IREnhanceLinearStretch:        return GrayIR_linearStretch(IR);
        case IREnhanceLinearStretch_Multi:  return GrayIR_linearStretch_multi(IR, m_enhance_coeff);
        case IREnhanceLinearStretch_STD:    return GrayIR_linearStretch_std(IR, m_enhance_coeff);
        case IREnhanceLinearStretch_LOG2:   return GrayIR_nonlinearStretch_log2(IR, m_enhance_coeff);
        case IREnhanceLinearStretch_Hist:   return GrayIR_nonlinearStretch_hist(IR);
        default: return TY_STATUS_INVALID_PARAMETER;
    }
}

TY_STATUS PercipioDevice::IRUndistortion(TYImage& IR, const TY_CAMERA_CALIB_INFO *calib_info, const TY_CAMERA_ROTATION *cameraRotation, const TY_CAMERA_INTRINSIC *cameraNewIntrinsic, const TYLensOpticalType type)
{
    if (!calib_info ) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "IR undistortion failed: calibration data unavailable");
        return TY_STATUS_INVALID_PARAMETER;
    }
        
    //Check if IR data is valid
    if (!IR.size() || IR.width() <= 0 || IR.height() <= 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "IR undistortion failed: invalid image data");
        return TY_STATUS_INVALID_PARAMETER;
    }
        
    //Get current image properties
    int32_t width = IR.width();
    int32_t height = IR.height();
    int32_t dataSize = (int32_t)IR.size();
    uint32_t pixelFormat = IR.format();
        
    //Allocate buffer for rectified image (same size as original)
    std::vector<uint8_t> rectifiedBuffer(dataSize);
        
    //Prepare source image data structure
    TY_IMAGE_DATA srcImage;
    srcImage.width = width;
    srcImage.height = height;
    srcImage.size = dataSize;
    srcImage.pixelFormat = pixelFormat;
    srcImage.buffer = IR.data();
    
    //Prepare destination image data structure
    TY_IMAGE_DATA dstImage;
    dstImage.width = width;
    dstImage.height = height;
    dstImage.size = dataSize;
    dstImage.pixelFormat = pixelFormat;
    dstImage.buffer = rectifiedBuffer.data();
    
    //Apply distortion correction
    TY_STATUS status = TYUndistortImage2(calib_info, &srcImage, cameraRotation, 
                                        cameraNewIntrinsic, &dstImage, type);
    if (status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "IR undistortion failed: " << status);
        return status;
    }
    
    //Copy rectified data to VideoFrameData buffer
    memcpy(IR.data(), rectifiedBuffer.data(), dataSize);
    return TY_STATUS_OK;
}

void PercipioDevice::leftIRStreamReceive(TYImage& ir, uint64_t& timestamp, const TY_CAMERA_CALIB_INFO& calib, image_intrinsic& intr, const TY_CAMERA_INTRINSIC* rectified_intr)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) {
        IREnhancement(ir);
        if(b_enable_sw_ir_undistortion) {
            if(DISTORTION_CORRECTION == ir_rectificatio_mode) {
                IRUndistortion(ir, &calib, nullptr, nullptr, ir_len_type);
            } else {
                IRUndistortion(ir, &calib, &left_ir_rotation, rectified_intr, ir_len_type);
            }
        }
        VideoStreamPtr->IRLeftInit(ir, intr, timestamp);
    }
}

void PercipioDevice::rightIRStreamReceive(TYImage& ir, uint64_t& timestamp, const TY_CAMERA_CALIB_INFO& calib, image_intrinsic& intr, const TY_CAMERA_INTRINSIC* rectified_intr)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) {
        IREnhancement(ir);
        if(b_enable_sw_ir_undistortion) {
            if(DISTORTION_CORRECTION == ir_rectificatio_mode) {
                IRUndistortion(ir, &calib, nullptr, nullptr, ir_len_type);
            } else {
                IRUndistortion(ir, &calib, &right_ir_rotation, rectified_intr, ir_len_type);
            }
        }
        VideoStreamPtr->IRRightInit(ir, intr, timestamp);
    }
}

void PercipioDevice::depthStreamReceive(TYImage& depth, uint64_t& timestamp, int32_t  target_width, int32_t target_height, const TY_CAMERA_CALIB_INFO& depth_calib, image_intrinsic& depth_intr, const TY_CAMERA_CALIB_INFO& color_calib, image_intrinsic& color_intr)
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
            
            TY_STATUS err = TYUndistortImage(&depth_calib, &src, NULL, &dst);
            if(err) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth undistortion failed: " << err);
        } else {
            targetDepth = depth;
        }

        if(topics_d_registration_) {
            //TYImage out = TYImage(targetDepth.width(), targetDepth.height(), TYPixelFormatCoord3D_C16);
            TYImage out = TYImage(target_width, target_height, TYPixelFormatCoord3D_C16);
            TYMapDepthImageToColorCoordinate(&depth_calib,
                targetDepth.width(), targetDepth.height(), (const uint16_t*)targetDepth.data(),
                &color_calib,
                out.width(), out.height(), (uint16_t*)out.data(), f_scale_unit);
            
            targetDepth = out.clone();
            VideoStreamPtr->DepthInit(targetDepth, color_intr, timestamp);
        } else if(topics_depth_) {
            VideoStreamPtr->DepthInit(targetDepth, depth_intr, timestamp);
        }
    } else if(depth.format() == TYPixelFormatCoord3D_ABC16) {
        if(topics_d_registration_) {
            TYImage p3d = convertABC16ToABC32f(depth);

            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&color_calib.extrinsic, &extri_inv);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data(), p3d.width() * p3d.height(), (TY_VECT_3F*)p3d.data());

            targetDepth = TYImage(depth.width(), depth.height(), TYPixelFormatCoord3D_C16);
            TYMapPoint3dToDepthImage(&color_calib, (const TY_VECT_3F*)(p3d.data()), p3d.width() * p3d.height(), p3d.width(), p3d.height(), (uint16_t*)(targetDepth.data()));
            VideoStreamPtr->DepthInit(targetDepth, color_intr,timestamp);
        } else if(topics_depth_) {
            targetDepth = depth;
            VideoStreamPtr->DepthInit(targetDepth, depth_intr, timestamp);
        }
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid depth stream fmt: " << depth.format());
        return;
    }

    depth = std::move(targetDepth);
    return;
}

void PercipioDevice::p3dStreamReceive(const TYImage& depth, uint64_t& timestamp, const TY_CAMERA_CALIB_INFO& depth_calib, image_intrinsic& depth_intr, const TY_CAMERA_CALIB_INFO& color_calib, image_intrinsic& color_intr) {
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
            TY_STATUS err = TYUndistortImage(&depth_calib, &src, NULL, &dst);
            if(err) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth undistortion failed: " << err);
            }
        } else {
            targetDepth = depth;
        }
        
        if(topics_color_p3d_) {
            TYMapDepthImageToPoint3d(&color_calib, targetDepth.width(), targetDepth.height(), (const uint16_t*)targetDepth.data(), (TY_VECT_3F*)p3d.data(), f_scale_unit);
            VideoStreamPtr->PointCloudInit(p3d, color_intr, timestamp);
        } else if(topics_p3d_) {
            TYMapDepthImageToPoint3d(&depth_calib, targetDepth.width(), targetDepth.height(), (const uint16_t*)targetDepth.data(), (TY_VECT_3F*)p3d.data(), f_scale_unit);
            VideoStreamPtr->PointCloudInit(p3d, depth_intr, timestamp);
        }
    } else if(depth.format() == TYPixelFormatCoord3D_ABC16) {
        p3d = convertABC16ToABC32f(depth);
        if(topics_p3d_) {
            VideoStreamPtr->PointCloudInit(p3d, depth_intr, timestamp);
        } else if(topics_color_p3d_) {
            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&color_calib.extrinsic, &extri_inv);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data(), p3d.width() * p3d.height(), (TY_VECT_3F*)p3d.data());
            VideoStreamPtr->PointCloudInit(p3d, color_intr, timestamp);
        }
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported depth pixel format: " << depth.format());
    }
}

//相机离线监测
void PercipioDevice::device_offline_reconnect() {
    while(isAlive()) {
        //TODO
        std::unique_lock<std::mutex> lck(offline_detect_mutex);
        offline_detect_cond.wait(lck);
        b_offline_event_pending.store(false);
        reconnect = true;
        Release();
        while(true) {
            TY_STATUS status = Reconnect();
            if(status == TY_STATUS_OK) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device reconnected, restarting streams");
                bool setup_ok = _node->setupDevices();
                if(b_offline_event_pending.exchange(false)) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device went offline during reconnection setup, retrying...");
                    Release();
                    continue;
                }
                if(!setup_ok) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device setup failed during reconnection, retrying...");
                    Release();
                    continue;
                }
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
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to send soft trigger signal");
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
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Soft trigger deadline missed");
        }
    }
}

TYImage PercipioDevice::decodeFrameImage(const TY_IMAGE_DATA& image_data)
{
    uint32_t dest_size = 0;
    TYImageInfo image_info = ty_image_info(image_data);
    TYDecodeError err = TYGetDecodeBufferSize(&image_info, &dest_size, TY_OUTPUT_FORMAT_AUTO);
    if(err == TY_DECODE_SUCCESS) {
        std::vector<uint8_t> image_buffer(dest_size);
        TYDecodeResult ret_info;
        TYDecodeImage(&image_info, TY_OUTPUT_FORMAT_AUTO, image_buffer.data(), dest_size, &ret_info);
        TYImage decoded(image_data.width, image_data.height, ret_info.format);
        if(static_cast<size_t>(dest_size) <= decoded.size()) {
            memcpy(decoded.data(), image_buffer.data(), dest_size);
        }
        return decoded;
    }
    return TYImage(image_data.width, image_data.height, image_data.pixelFormat, image_data.buffer);
}

TYImage PercipioDevice::convertABC16ToABC32f(const TYImage& depth)
{
    TYImage p3d(depth.width(), depth.height(), TYPixelFormatCoord3D_ABC32f);
    const int16_t* src = static_cast<const int16_t*>(depth.data());
    float* dst = static_cast<float*>(p3d.data());
    const int width = depth.width();
    const int height = depth.height();
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            int idx = 3 * (i * width + j);
            if(src[idx + 2]) {
                dst[idx + 0] = src[idx + 0];
                dst[idx + 1] = src[idx + 1];
                dst[idx + 2] = src[idx + 2];
            } else {
                dst[idx + 0] = std::numeric_limits<float>::quiet_NaN();
                dst[idx + 1] = std::numeric_limits<float>::quiet_NaN();
                dst[idx + 2] = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return p3d;
}

void PercipioDevice::frameDataReceive() {
    TY_STATUS status;
    //m_softtrigger_ready = false;

    switch(workmode) {
        case CONTINUOUS:
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Operating mode: continuous streaming");
            break;
        case SOFTTRIGGER:
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Operating mode: soft trigger");
            break;
        case HARDTRIGGER:
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Operating mode: hardware trigger");
            break;
        default:
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Operating mode: unknown, defaulting to continuous");
            break;
    }

    fps_counter.reset();
    while (rclcpp::ok() && is_running_.load()) {
        TY_FRAME_DATA frame;
        /*
        if(workmode == CONTINUOUS || workmode == HARDTRIGGER) {
            status = TYFetchFrame(handle, &frame, 2000);
        } else if(workmode == SOFTTRIGGER) {
            status = TYFetchFrame(handle, &frame, 200);
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported workmode: " << workmode);
            status = TYFetchFrame(handle, &frame, 2000);
        }
        */
        int32_t retry_count = 10;
        while(retry_count-- > 0) {
            status = TYFetchFrame(handle, &frame, 200);
            if(status == TY_STATUS_OK) break;
            else if(status == TY_STATUS_TIMEOUT) continue;
            else {
                if((!rclcpp::ok()) || (!is_running_.load())) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Frame fetch interrupted, exiting...");
                    return;
                }

                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Frame fetch failed: " << status);
                MSLEEP(500);
            }
        }

        if(status == TY_STATUS_OK) {
            float fps = fps_counter.get_fps();
            if(fps > 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Frame rate: " << fps << " FPS");
            }

            
            for (int i = 0; i < frame.validCount; i++){
                if (frame.image[i].status != TY_STATUS_OK) continue;

                if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
                    TYImage depth;
                    TY_CAMERA_CALIB_INFO adj_depth_calib = adjustCalibByBinningCrop(cam_depth_calib_data, TY_COMPONENT_DEPTH_CAM, frame.image[i]);
                    image_intrinsic adj_depth_intrinsic(adj_depth_calib.intrinsicWidth, adj_depth_calib.intrinsicHeight, adj_depth_calib.intrinsic);

                    TY_CAMERA_CALIB_INFO adj_color_calib = cam_color_calib_data;
                    image_intrinsic adj_color_intrinsic = cam_color_intrinsic;
                    int32_t m_color_width = adj_color_calib.intrinsicWidth;
                    int32_t m_color_height = adj_color_calib.intrinsicHeight;
                    if(has_color_calib_data) {
                        adj_color_calib = adjustCalibByBinning(cam_color_calib_data, TY_COMPONENT_RGB_CAM);
                        adj_color_intrinsic = image_intrinsic(adj_color_calib.intrinsicWidth, adj_color_calib.intrinsicHeight, adj_color_calib.intrinsic);
                        m_color_width = adj_color_calib.intrinsicWidth;
                        m_color_height = adj_color_calib.intrinsicHeight;
                    }

                    if(frame.image[i].pixelFormat == TYPixelFormatCoord3D_C16) {
                        uint16_t* ptrDepth = static_cast<uint16_t*>(frame.image[i].buffer);
                        int32_t PixsCnt = frame.image[i].width * frame.image[i].height;
                        for(int32_t i = 0; i < PixsCnt; i++) {
                            if(ptrDepth[i] == 0xFFFF) ptrDepth[i] = 0;
                        }

                        if(b_depth_spk_filter_en) {
                            DepthSpeckleFilterParameters param = {m_depth_spk_size, m_depth_spk_diff, f_depth_spk_phy_size};
                            if(f_depth_spk_phy_size <= 0)
                                TYDepthSpeckleFilter(&frame.image[i], &param, nullptr, f_scale_unit);
                            else
                                TYDepthSpeckleFilter(&frame.image[i], &param, &adj_depth_calib, f_scale_unit);
                        }

                        if(b_depth_time_domain_en) {
                            DepthDomainTimeFilterMgrPtr->add_frame(frame.image[i]);
                            if(!DepthDomainTimeFilterMgrPtr->do_time_domain_process(frame.image[i])) {
                                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Time-domain filter applied, frame dropped");
                                continue;
                            }
                        }
                        depth = TYImage(frame.image[i].width, frame.image[i].height, TYPixelFormatCoord3D_C16, frame.image[i].buffer);
                    } else if((uint32_t)frame.image[i].pixelFormat == TYPixelFormatCoord3D_ABC16) {
                        depth = TYImage(frame.image[i].width, frame.image[i].height, TYPixelFormatCoord3D_ABC16, frame.image[i].buffer);
                    }
                    depthStreamReceive(depth, frame.image[i].timestamp, m_color_width, m_color_height, adj_depth_calib, adj_depth_intrinsic, adj_color_calib, adj_color_intrinsic);
                    p3dStreamReceive(depth, frame.image[i].timestamp, adj_depth_calib, adj_depth_intrinsic, adj_color_calib, adj_color_intrinsic);
                }

                if (frame.image[i].componentID == TY_COMPONENT_RGB_CAM) {
                    TYImage color = decodeFrameImage(frame.image[i]);
                    TY_CAMERA_CALIB_INFO adj_color_calib = adjustCalibByBinningCrop(cam_color_calib_data, TY_COMPONENT_RGB_CAM, frame.image[i]);
                    image_intrinsic adj_color_intrinsic(adj_color_calib.intrinsicWidth, adj_color_calib.intrinsicHeight, adj_color_calib.intrinsic);
                    colorStreamReceive(color, frame.image[i].timestamp, adj_color_calib, adj_color_intrinsic);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT) {
                    TYImage left_ir = decodeFrameImage(frame.image[i]);
                    TY_CAMERA_CALIB_INFO adj_lir_calib = adjustCalibByBinningCrop(cam_left_ir_calib_data, TY_COMPONENT_IR_CAM_LEFT, frame.image[i]);
                    image_intrinsic adj_lir_intrinsic(adj_lir_calib.intrinsicWidth, adj_lir_calib.intrinsicHeight, adj_lir_calib.intrinsic);
                    TY_CAMERA_INTRINSIC adj_lir_rectified_intr = adjustIntrinsicByBinningCrop(left_ir_rectified_intr, TY_COMPONENT_IR_CAM_LEFT, frame.image[i]);
                    leftIRStreamReceive(left_ir, frame.image[i].timestamp, adj_lir_calib, adj_lir_intrinsic, &adj_lir_rectified_intr);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT) {
                    TYImage right_ir = decodeFrameImage(frame.image[i]);
                    TY_CAMERA_CALIB_INFO adj_rir_calib = adjustCalibByBinningCrop(cam_right_ir_calib_data, TY_COMPONENT_IR_CAM_RIGHT, frame.image[i]);
                    image_intrinsic adj_rir_intrinsic(adj_rir_calib.intrinsicWidth, adj_rir_calib.intrinsicHeight, adj_rir_calib.intrinsic);
                    TY_CAMERA_INTRINSIC adj_rir_rectified_intr = adjustIntrinsicByBinningCrop(right_ir_rectified_intr, TY_COMPONENT_IR_CAM_RIGHT, frame.image[i]);
                    rightIRStreamReceive(right_ir, frame.image[i].timestamp, adj_rir_calib, adj_rir_intrinsic, &adj_rir_rectified_intr);
                }
            }

            if(_callback)
               _callback(*VideoStreamPtr.get());

            TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
        } else if(status != TY_STATUS_TIMEOUT) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Frame fetch failed: " << status);
            MSLEEP(500);
        }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Frame receive thread terminated");
}

//开启数据流
bool PercipioDevice::stream_start()
{
    if(b_dev_frame_rate_ctrl_en) {
        if(workmode != CONTINUOUS) {
            workmode = CONTINUOUS;
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Fixed frame rate enabled: operating mode overridden to continuous");
        }
    }
    m_gige_dev->work_mode_init(workmode, b_dev_frame_rate_ctrl_en, f_dev_frame_rate);

    m_gige_dev->stream_base_info_init();
    
    uint32_t frameSize;
    TY_STATUS status = TYGetFrameBufferSize(handle, &frameSize);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to obtain frame buffer size: " << status);
        return false;
    }
    frameBuffer[0].resize(frameSize);
    frameBuffer[1].resize(frameSize);
    
    TYEnqueueBuffer(handle, frameBuffer[0].data(), frameSize);
    TYEnqueueBuffer(handle, frameBuffer[1].data(), frameSize);

    status = TYStartCapture(handle);
    if(status != TY_STATUS_OK) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to start stream capture: " << status);
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
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Camera is not streaming");
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Camera is not in soft trigger mode, trigger signal ignored");
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

void PercipioDevice::depth_time_domain_filter_init(bool enable, int number)
{
    b_depth_time_domain_en = enable;
    m_depth_time_domain_frame_num = number;
    DepthDomainTimeFilterMgrPtr->reset(m_depth_time_domain_frame_num);
}

void PercipioDevice::ir_enhance_mode_init(ir_enhance_model mode, int coeff)
{
    enhance_mode = mode;
    m_enhance_coeff = coeff;
}

void PercipioDevice::ir_undistortion_enable(bool en)
{
    b_do_ir_undist = en;
}

}
