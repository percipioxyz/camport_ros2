#include "percipio_device.h"
#include "common.hpp"
#include "TYCoordinateMapper.h"
#include "TYImageProc.h"
#include "crc32.hpp"
#include "ParametersParse.hpp"
#include "huffman.h"

#include "percipio_camera_node.h"

namespace percipio_camera {

#define INVALID_COMPONENT_ID        (0xFFFFFFFF)

#define MAX_STORAGE_SIZE    (10*1024*1024)

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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Open interface fail : " << status);
        return status;
    }
  
    status = TYOpenDevice(hIface, deviceId, &handle);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Open device fail : " << status);
        TYCloseInterface(hIface);
        return status;
    }

    status = TYGetDeviceInfo(handle, &base_info);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Invalid device handle  : " << status);
        TYCloseDevice(handle);
        TYCloseInterface(hIface);
        return status;
    }

#if 1
    status = TYSetEnum(handle, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_HOST);
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Set time sync type(host) err : " << status);
    }
#else
    uint8_t ip[4]= {192,168,1,100};
    uint32_t IP = TYIPv4ToInt(ip);
    status = TYSetInt(handle, TY_COMPONENT_DEVICE, TY_INT_NTP_SERVER_IP, IP)
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Set ntp server ip err : " << status);
    }

    status = TYSetEnum(handle, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_NTP);
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Set time sync type(host) err : " << status);
    }

    int m_retry = 20;
    while(m_retry-- ) {
        bool b_ready = false;
        status = TYGetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_TIME_SYNC_READY, &b_ready);
        if(status == TY_STATUS_OK && b_ready) {
            break;
        }
        MSLEEP(100);
    }
    if(!m_retry) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Wait NTP time sync ready signal err");
    }
#endif

    load_default_parameter();

    TYGetComponentIDs(handle, &allComps);

    if(allComps & TY_COMPONENT_RGB_CAM) {
        std::vector<TY_ENUM_ENTRY> image_mode_list(0);
        get_feature_enum_list(handle, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "color stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                std::string format_desc;
                uint32_t mode = image_mode_list[i].value;
                int m_width = TYImageWidth(mode);
                int m_height = TYImageHeight(mode);
                uint32_t m_fmt = TYPixelFormat(mode);
                if(nominateStreamFormat(m_fmt, format_desc))
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        TYDisableComponents(handle, TY_COMPONENT_RGB_CAM);
    }

    if(allComps & TY_COMPONENT_DEPTH_CAM) {
        std::vector<TY_ENUM_ENTRY> image_mode_list(0);
        get_feature_enum_list(handle, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "depth stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                std::string format_desc;
                uint32_t mode = image_mode_list[i].value;
                int m_width = TYImageWidth(mode);
                int m_height = TYImageHeight(mode);
                uint32_t m_fmt = TYPixelFormat(mode);
                if(nominateStreamFormat(m_fmt, format_desc))
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }
        TYDisableComponents(handle, TY_COMPONENT_DEPTH_CAM);
    }

    if(allComps & TY_COMPONENT_IR_CAM_LEFT) {
        std::vector<TY_ENUM_ENTRY> image_mode_list(0);
        get_feature_enum_list(handle, TY_COMPONENT_IR_CAM_LEFT, TY_ENUM_IMAGE_MODE, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "left-IR stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                std::string format_desc;
                uint32_t mode = image_mode_list[i].value;
                int m_width = TYImageWidth(mode);
                int m_height = TYImageHeight(mode);
                uint32_t m_fmt = TYPixelFormat(mode);
                if(nominateStreamFormat(m_fmt,  format_desc))
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }
        TYDisableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
    }

    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) {
        std::vector<TY_ENUM_ENTRY> image_mode_list(0);
        get_feature_enum_list(handle, TY_COMPONENT_IR_CAM_RIGHT, TY_ENUM_IMAGE_MODE, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "right-IR stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                std::string format_desc;
                uint32_t mode = image_mode_list[i].value;
                int m_width = TYImageWidth(mode);
                int m_height = TYImageHeight(mode);
                uint32_t m_fmt = TYPixelFormat(mode);
                if(nominateStreamFormat(m_fmt, format_desc))
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }
        TYDisableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
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
    is_running_.store(false);

    if(workmode == SOFTTRIGGER) {
        std::unique_lock<std::mutex> lck( softtrigger_mutex);
        softtrigger_detect_cond.notify_one();
    }

    if (frame_recive_thread_ && frame_recive_thread_->joinable()) {
        frame_recive_thread_->join();
        frame_recive_thread_ = nullptr;
    }

    TYCloseDevice(handle);
    handle = nullptr;

    TYCloseInterface(hIface);
    hIface = nullptr;
}

//析构，释放资源
PercipioDevice::~PercipioDevice()
{
    is_running_.store(false);

    if(workmode == SOFTTRIGGER) {
        std::unique_lock<std::mutex> lck( softtrigger_mutex);
        softtrigger_detect_cond.notify_one();
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

static bool isValidJsonString(const char* code)
{
    std::string err;
    const auto json = Json::parse(code, err);
    if(json.is_null()) return false;
    return true;
}

bool PercipioDevice::load_default_parameter()
{
    TY_STATUS status = TY_STATUS_OK;
    uint32_t block_size;
    uint8_t* blocks = new uint8_t[MAX_STORAGE_SIZE] ();
    status = TYGetByteArraySize(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, &block_size);
    if(status != TY_STATUS_OK) {
        delete []blocks;
        return false;
    } 
    
    status = TYGetByteArray(handle, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, blocks,  block_size);
    if(status != TY_STATUS_OK) {
        delete []blocks;
        return false;
    }
    
    uint32_t crc_data = *(uint32_t*)blocks;
    if(0 == crc_data || 0xffffffff == crc_data) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "The CRC check code is empty.");
        delete []blocks;
        return false;
    } 
    
    uint32_t crc;
    std::string js_string;
    uint8_t* js_code = blocks + 4;
    crc = crc32_bitwise(js_code, strlen((const char*)js_code));
    if((crc != crc_data) || !isValidJsonString((const char*)js_code)) {
        EncodingType type = *(EncodingType*)(blocks + 4);
        switch(type) {
            case HUFFMAN:
            {
                uint32_t huffman_size = *(uint32_t*)(blocks + 8);
                uint8_t* huffman_ptr = (uint8_t*)(blocks + 12);
                if(huffman_size > (MAX_STORAGE_SIZE - 8)) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Storage data length error.");
                    delete []blocks;
                    return false;
                }
                
                crc = crc32_bitwise(huffman_ptr, huffman_size);
                if(crc_data != crc) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Storage area data check failed (check code error).");
                    delete []blocks;
                    return false;
                }

                std::string huffman_string(huffman_ptr, huffman_ptr + huffman_size);
                if(!TextHuffmanDecompression(huffman_string, js_string)) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Huffman decompression error.");
                    delete []blocks;
                    return false;
                }
                break;
            }
            default:
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Unsupported encoding format.");
                delete []blocks;
                return false;
            }
        }
    } else {
        js_string = std::string((const char*)js_code);
    }

    if(!isValidJsonString(js_string.c_str())) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Incorrect json data.");
        delete []blocks;
        return false;
    }

    bool ret =json_parse(handle, js_string.c_str());
    if(ret)  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "Loading default parameters successfully!");
    else
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Failed to load default parameters, some parameters cannot be loaded properly!");

    delete []blocks;
    return ret;
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
        std::unique_lock<std::mutex> lck( handle->offline_detect_mutex);
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
    return (allComps & TY_COMPONENT_RGB_CAM) == 
            TY_COMPONENT_RGB_CAM;
}

bool PercipioDevice::hasDepth()
{
    return (allComps & TY_COMPONENT_DEPTH_CAM) == 
            TY_COMPONENT_DEPTH_CAM;
}

bool PercipioDevice::hasLeftIR()
{
    return (allComps & TY_COMPONENT_IR_CAM_LEFT) == 
            TY_COMPONENT_IR_CAM_LEFT;
}

bool PercipioDevice::hasRightIR()
{
    return (allComps & TY_COMPONENT_IR_CAM_RIGHT) == 
            TY_COMPONENT_IR_CAM_RIGHT;
}

//packet resend enable
void PercipioDevice::enable_gvsp_resend(const bool en)
{
    b_packet_resend_en = en;
}

//创建相机离线重现监测函数
void PercipioDevice::enable_offline_reconnect(const bool en) 
{ 
    b_dev_auto_reconnect = en; 
    if(!b_dev_auto_reconnect)
        return;
    if(device_reconnect_thread)
        return;
    device_reconnect_thread = std::make_shared<std::thread>([this]() { device_offline_reconnect(); });
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

static uint32_t depth_qua_desc_to_enum(const std::string& qua)
{
    if(qua == "basic") return TY_DEPTH_QUALITY_BASIC;
    else if(qua == "medium") return TY_DEPTH_QUALITY_MEDIUM;
    else if(qua == "high") return TY_DEPTH_QUALITY_HIGH;
    else return TY_DEPTH_QUALITY_MEDIUM;
}

bool PercipioDevice::set_tof_depth_quality(const std::string& qua)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;

    uint32_t m_qua = depth_qua_desc_to_enum(qua);
    status = TYSetEnum(handle, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, m_qua);
    if(status != TY_STATUS_OK) return false;
    return true;
}

bool PercipioDevice::set_tof_modulation_threshold(int threshold)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_MODULATION_THRESHOLD, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;
    status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_MODULATION_THRESHOLD, threshold);
    if(status != TY_STATUS_OK) return false;
    return true;
}

bool PercipioDevice::set_tof_jitter_threshold(int threshold)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_JITTER_THRESHOLD, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;
    status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_JITTER_THRESHOLD, threshold);
    if(status != TY_STATUS_OK) return false;
    return true;
}

bool PercipioDevice::set_tof_filter_threshold(int threshold)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_FILTER_THRESHOLD, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;
    status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_FILTER_THRESHOLD, threshold);
    if(status != TY_STATUS_OK) return false;
    return true;
}

bool PercipioDevice::set_tof_channel(int chan)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_CHANNEL, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;
    status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_CHANNEL, chan);
    if(status != TY_STATUS_OK) return false;
    return true;
}

bool PercipioDevice::set_tof_HDR_ratio(int ratio)
{
    TY_STATUS status;
    bool has = false;
    status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_HDR_RATIO, &has);
    if(status != TY_STATUS_OK) return false;
    if(!has) return false;
    status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_HDR_RATIO, ratio);
    if(status != TY_STATUS_OK) return false;
    return true;
}

//node 的 stream idx 和 camport sdk 的stream idx转换
uint32_t PercipioDevice::StreamConvertComponent(const percipio_stream_index_pair& idx)
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

//解析用户设置的stream 数据流
bool PercipioDevice::resolveStreamResolution(const std::string& resolution_, int& width, int& height)
{
  size_t pos = resolution_.find('x');
  if((pos != 0) && (pos != std::string::npos))
  {
    std::string str_width = resolution_.substr(0, pos);
    std::string str_height = resolution_.substr(pos+1, resolution_.length());
    width = atoi(str_width.c_str());
    height = atoi(str_height.c_str());
    return true;
  }
  return false;
}

struct StreamFormatMapper
{
    std::string str_format;
    uint32_t    m_format;
};

static StreamFormatMapper format_mapper_list[] = {
    {"mono",            TY_PIXEL_FORMAT_MONO        },
    {"bayer8gb",        TY_PIXEL_FORMAT_BAYER8GB    },
    {"bayer8bg",        TY_PIXEL_FORMAT_BAYER8BG    },
    {"bayer8gr",        TY_PIXEL_FORMAT_BAYER8GR    },
    {"bayer8rg",        TY_PIXEL_FORMAT_BAYER8RG    },
    {"bayer8grbg",      TY_PIXEL_FORMAT_BAYER8GRBG  },
    {"bayer8rggb",      TY_PIXEL_FORMAT_BAYER8RGGB  },
    {"bayer8gbrg",      TY_PIXEL_FORMAT_BAYER8GBRG  },
    {"bayer8bggr",      TY_PIXEL_FORMAT_BAYER8BGGR  },

    {"mono10",          TY_PIXEL_FORMAT_CSI_MONO10      },
    {"bayer10grbg",     TY_PIXEL_FORMAT_CSI_BAYER10GRBG },
    {"bayer10rggb",     TY_PIXEL_FORMAT_CSI_BAYER10RGGB },
    {"bayer10gbrg",     TY_PIXEL_FORMAT_CSI_BAYER10GBRG },
    {"bayer10bggr",     TY_PIXEL_FORMAT_CSI_BAYER10BGGR },

    {"mono12",          TY_PIXEL_FORMAT_CSI_MONO12      },
    {"bayer12grbg",     TY_PIXEL_FORMAT_CSI_BAYER12GRBG },
    {"bayer12rggb",     TY_PIXEL_FORMAT_CSI_BAYER12RGGB },
    {"bayer12gbrg",     TY_PIXEL_FORMAT_CSI_BAYER12GBRG },
    {"bayer12bggr",     TY_PIXEL_FORMAT_CSI_BAYER12BGGR },

    {"depth16",         TY_PIXEL_FORMAT_DEPTH16         },
    {"yvyu",            TY_PIXEL_FORMAT_YVYU            },
    {"yuyv",            TY_PIXEL_FORMAT_YUYV            },
    {"mono16",          TY_PIXEL_FORMAT_MONO16          },
    {"tof_ir_mono16",   TY_PIXEL_FORMAT_TOF_IR_MONO16   },
    {"rgb",             TY_PIXEL_FORMAT_RGB             },
    {"bgr",             TY_PIXEL_FORMAT_BGR             },
    {"jpeg",            TY_PIXEL_FORMAT_JPEG            },
    {"mjpg",            TY_PIXEL_FORMAT_MJPG            },
    {"rgb48",           TY_PIXEL_FORMAT_RGB48           },
    {"bgr48",           TY_PIXEL_FORMAT_BGR48           },
    {"xyz48",           TY_PIXEL_FORMAT_XYZ48           },
};

bool PercipioDevice::resolveStreamFormat(const std::string& format, uint32_t& fmt)
{
    if(format.empty()) return false;
    for(auto _map : format_mapper_list){
        if(_map.str_format == format) {
            fmt = _map.m_format;
            return true;
        }
    }
    return false;
}

bool PercipioDevice::nominateStreamFormat(const uint32_t& fmt,  std::string& format)
{
    for(auto _map : format_mapper_list) {
        if(_map.m_format == fmt) {
            format = _map.str_format;
            return true;
        }
    }
    return false;
}


///stream 畸变map 初始化
void PercipioDevice::StreamDistortionMapInit(TY_COMPONENT_ID comp, percipio_distortion_map_info& map)
{
    TY_STATUS status;
    int m_width, m_height;
    TY_CAMERA_INTRINSIC intrinsic;
    TY_CAMERA_DISTORTION distortion;

    cv::Mat intrinsic_mat, distortion_mat;
    cv::Mat mapX, mapY;

    status = TYGetInt(handle, comp, TY_INT_WIDTH, &m_width);
    if(status) goto INIT_FAIL;
    status = TYGetInt(handle, comp, TY_INT_HEIGHT, &m_height);
    if(status) goto INIT_FAIL;
    status = TYGetStruct(handle, comp, TY_STRUCT_CAM_INTRINSIC, &intrinsic, sizeof(intrinsic));
    if(status) goto INIT_FAIL;
    status = TYGetStruct(handle, comp, TY_STRUCT_CAM_DISTORTION, &distortion, sizeof(distortion));
    if(status) goto INIT_FAIL;

    map.m_map_width = m_width;
    map.m_map_height = m_height;
    map.f_map_x.resize(m_width * m_height);
    map.f_map_y.resize(m_width * m_height);
    intrinsic_mat = cv::Mat(3, 3, CV_32F, intrinsic.data);
    distortion_mat = cv::Mat(12, 1, CV_32F, distortion.data);
    mapX = cv::Mat(cv::Size(m_width, m_height), CV_32F, map.f_map_x.data());
    mapY = cv::Mat(cv::Size(m_width, m_height), CV_32F, map.f_map_y.data());
    cv::initUndistortRectifyMap(intrinsic_mat,
            distortion_mat, cv::Mat(), 
            intrinsic_mat, 
            cv::Size(m_width, m_height), 
            CV_32FC1, 
            mapX, 
            mapY);
    return;

INIT_FAIL:
    map.m_map_width = 0;
    map.m_map_height = 0;
    map.f_map_x.clear();
    map.f_map_y.clear();
    return ;
}

//读取相机深度图的单位
float PercipioDevice::getDepthValueScale()
{
    return f_scale_unit;
}

//更新color roi aec信息
bool PercipioDevice::update_color_aec_roi(int x, int y , int w, int h)
{
    ROI.x = x;
    ROI.y = y;
    ROI.w = w;
    ROI.h = h;
    enable_rgb_aec_roi = true;
    return true;
}

//开启指定数据流
bool PercipioDevice::stream_open(const percipio_stream_index_pair& idx, const std::string& resolution, const std::string& format)
{
    TY_STATUS status;
    uint32_t m_comp = StreamConvertComponent(idx);
    if(INVALID_COMPONENT_ID == m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Invalid stream!");
        return false;
    }

    if((m_comp & allComps) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Unsupported component : 0x" << std::hex <<  m_comp);
        return false;
    }

    std::vector<TY_ENUM_ENTRY> image_mode_list(0);
    std::vector<uint32_t> image_mode_val_list(0);
    std::vector<std::string> image_mode_str_list(0);

    uint32_t fmt;
    int img_width, img_height;
    bool valid_resolution = resolveStreamResolution(resolution, img_width, img_height);
    bool valid_format = resolveStreamFormat(format, fmt);
    get_feature_enum_list(handle, m_comp, TY_ENUM_IMAGE_MODE, image_mode_list);
    if(image_mode_list.size()) {
        for(auto image_mode : image_mode_list) {
            uint32_t current_image_mode = image_mode.value;
            if(valid_resolution) {
                if(img_width != TYImageWidth(current_image_mode) ||
                   img_height != TYImageHeight(current_image_mode)) {
                    continue;
                }
            }
            if(valid_format) {
                if(fmt != TYPixelFormat(current_image_mode)) {
                    continue;
                }
            }

            image_mode_val_list.push_back(current_image_mode);
            image_mode_str_list.push_back(image_mode.description);
        }
    }

    if(valid_resolution) {
        if(image_mode_val_list.size()) {
            status = TYSetEnum(handle, m_comp, TY_ENUM_IMAGE_MODE, image_mode_val_list[0]);
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Stream mode init error :" << status);
            } else {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "Set stream mode : " << image_mode_str_list[0]);
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Unsupported stream mode!");
        }
    }

    status = TYEnableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Stream open error :" << status);
        return false;
    }

    TYGetFloat(handle, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &f_scale_unit);
    TYGetStruct(handle, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &cam_depth_calib_data, sizeof(cam_depth_calib_data));
    TYGetStruct(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &cam_color_calib_data, sizeof(cam_color_calib_data));

    switch (m_comp) {
        case TY_COMPONENT_DEPTH_CAM:
            cam_depth_intrinsic.resize(9);
            TYGetStruct(handle, m_comp, TY_STRUCT_CAM_INTRINSIC, &cam_depth_intrinsic[0], cam_depth_intrinsic.size() * sizeof(float));
            break;
        case TY_COMPONENT_RGB_CAM:
            cam_color_intrinsic.resize(9);
            TYGetStruct(handle, m_comp, TY_STRUCT_CAM_INTRINSIC, &cam_color_intrinsic[0], cam_color_intrinsic.size() * sizeof(float));
            break;
        case TY_COMPONENT_IR_CAM_LEFT:
            cam_leftir_intrinsic.resize(9);
            TYGetStruct(handle, m_comp, TY_STRUCT_CAM_INTRINSIC, &cam_leftir_intrinsic[0], cam_leftir_intrinsic.size() * sizeof(float));
            break;
        case TY_COMPONENT_IR_CAM_RIGHT:
            cam_rightir_intrinsic.resize(9);
            TYGetStruct(handle, m_comp, TY_STRUCT_CAM_INTRINSIC, &cam_rightir_intrinsic[0], cam_rightir_intrinsic.size() * sizeof(float));
            break;
        default:
            break;
    }

#ifdef IMAGE_DoUnsitortion_With_OpenCV
    StreamDistortionMapInit(TY_COMPONENT_DEPTH_CAM, depth_map);
    StreamDistortionMapInit(TY_COMPONENT_RGB_CAM, color_map);
#else
    status = TYGetStruct(handle, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &depth_calib_data, sizeof(depth_calib_data));
    if(status != TY_STATUS_OK) {
        has_depth_calib_data = false;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Got depth stream calib dataerror :" << status);
    } else {
        has_depth_calib_data = true;
    }
    status = TYGetStruct(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &color_calib_data, sizeof(color_calib_data));
    if(status != TY_STATUS_OK) {
        has_color_calib_data = false;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Got color stream calib dataerror :" << status);
    } else {
        has_color_calib_data = true;
    }
#endif

    if(!reconnect) m_streams.push_back({idx, resolution, format});
    VideoStreamPtr = std::make_shared<VideoStream>();
    return true;
}

//关闭指定数据流
bool PercipioDevice::stream_close(const percipio_stream_index_pair& idx)
{
    TY_STATUS status;
    uint32_t m_comp = StreamConvertComponent(idx);
    if(INVALID_COMPONENT_ID == m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Invalid stream!");
        return false;
    }

    if((m_comp & allComps) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Unsupported component : 0x" << std::hex <<  m_comp);
        return false;
    }

    status = TYDisableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Stream close error :" << status);
        return false;
    }

    return true;
}

//
void PercipioDevice::colorStreamReceive(cv::Mat& color, uint64_t& timestamp)
{
    cv::Mat mapX, mapY;
    cv::Mat targetRGB;

    if(color.empty()) return;
    if(VideoStreamPtr) {
#ifdef IMAGE_DoUnsitortion_With_OpenCV
        if(color_map.IsValid()) {
            mapX = cv::Mat(cv::Size(color_map.m_map_width, color_map.m_map_height), CV_32F, color_map.f_map_x.data());
            mapY = cv::Mat(cv::Size(color_map.m_map_width, color_map.m_map_height), CV_32F, color_map.f_map_y.data());
            cv::remap(color, targetRGB, mapX, mapY, 0);
        } else {
            targetRGB = color;
        }
#else
        if(has_color_calib_data) {
            if(color.type() == CV_8UC3) {
                targetRGB = color.clone();

                TY_IMAGE_DATA src;
                src.width = color.cols;
                src.height = color.rows;
                src.size = color.size().area() * 3;;
                src.pixelFormat = TY_PIXEL_FORMAT_BGR;
                src.buffer = color.data;

                TY_IMAGE_DATA dst;
                dst.width = color.cols;
                dst.height = color.rows;
                dst.size = color.size().area() * 3;;
                dst.pixelFormat = TY_PIXEL_FORMAT_BGR;
                dst.buffer = targetRGB.data;
                TY_STATUS err = TYUndistortImage(&color_calib_data, &src, NULL, &dst);
                if(err) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Color TYUndistortImage ret = :" << err);
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Invalid color stream fmt :" << color.type());
                return;
            }
        } else {
            targetRGB = color;
        }
#endif
        VideoStreamPtr->ColorInit(targetRGB, &cam_color_intrinsic[0], timestamp);
    }
}

void PercipioDevice::leftIRStreamReceive(cv::Mat& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRLeftInit(ir, &cam_leftir_intrinsic[0], timestamp);
}

void PercipioDevice::rightIRStreamReceive(cv::Mat& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRRightInit(ir, &cam_rightir_intrinsic[0], timestamp);
}

void PercipioDevice::depthStreamReceive(cv::Mat& depth, uint64_t& timestamp)
{
    cv::Mat mapX, mapY;
    cv::Mat targetDepth;

    if(depth.empty()) return;
    if(VideoStreamPtr) {
#ifdef IMAGE_DoUnsitortion_With_OpenCV
        if(depth_map.IsValid()) {
            mapX = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_x.data());
            mapY = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_y.data());
            cv::remap(depth, targetDepth, mapX, mapY, 0);
        } else {
            targetDepth = depth;
        }
#else
        if(has_depth_calib_data) {
            if(depth.type() == CV_16U) {
                targetDepth = depth.clone();

                TY_IMAGE_DATA src;
                src.width = depth.cols;
                src.height = depth.rows;
                src.size = depth.size().area() * 2;
                src.pixelFormat = TY_PIXEL_FORMAT_DEPTH16;
                src.buffer = depth.data;

                TY_IMAGE_DATA dst;
                dst.width = depth.cols;
                dst.height = depth.rows;
                dst.size = depth.size().area() * 2;
                dst.pixelFormat = TY_PIXEL_FORMAT_DEPTH16;
                dst.buffer = targetDepth.data;
                TY_STATUS err = TYUndistortImage(&depth_calib_data, &src, NULL, &dst);
                if(err) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Depth TYUndistortImage ret = :" << err);
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Invalid depth stream fmt :" << depth.type());
                return;
            }

        } else {
            targetDepth = depth;
        }
#endif
        if(topics_d_registration_) {
            cv::Mat out = cv::Mat::zeros(targetDepth.size(), CV_16U);
            TYMapDepthImageToColorCoordinate(&cam_depth_calib_data,
                targetDepth.cols, targetDepth.rows, targetDepth.ptr<uint16_t>(),
                &cam_color_calib_data,
                out.cols, out.rows, out.ptr<uint16_t>(), f_scale_unit);
                targetDepth = out.clone();
            VideoStreamPtr->DepthInit(targetDepth, &cam_color_intrinsic[0], timestamp);
        } else if(topics_depth_) {
            VideoStreamPtr->DepthInit(targetDepth, &cam_depth_intrinsic[0],timestamp);
        }
    }
    return;
}

void PercipioDevice::p3dStreamReceive(cv::Mat& depth, uint64_t& timestamp) {
    if(depth.empty()) return;
    if(!topics_p3d_  && !topics_color_p3d_) return;

    if(VideoStreamPtr) {
        cv::Mat p3d = cv::Mat(depth.size(), CV_32FC3);
        if(depth.type() == CV_16U) {
            cv::Mat targetDepth;
#ifdef IMAGE_DoUnsitortion_With_OpenCV            
            if(depth_map.IsValid()) {
                cv::Mat mapX = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_x.data());
                cv::Mat mapY = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_y.data());
                cv::remap(depth, targetDepth, mapX, mapY, 0);
            } else {
                targetDepth = depth;
            }
#else
            if(has_depth_calib_data) {
                if(depth.type() == CV_16U) {
                    targetDepth = depth.clone();
    
                    TY_IMAGE_DATA src;
                    src.width = depth.cols;
                    src.height = depth.rows;
                    src.size = depth.size().area() * 2;
                    src.pixelFormat = TY_PIXEL_FORMAT_DEPTH16;
                    src.buffer = depth.data;
    
                    TY_IMAGE_DATA dst;
                    dst.width = depth.cols;
                    dst.height = depth.rows;
                    dst.size = depth.size().area() * 2;
                    dst.pixelFormat = TY_PIXEL_FORMAT_DEPTH16;
                    dst.buffer = targetDepth.data;
                    TY_STATUS err = TYUndistortImage(&depth_calib_data, &src, NULL, &dst);
                    if(err) {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Depth TYUndistortImage ret = :" << err);
                    }
                } else {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Invalid depth stream fmt :" << depth.type());
                    return;
                }
            } else {
                targetDepth = depth;
            }
#endif
            if(topics_p3d_) {
                TYMapDepthImageToPoint3d(&cam_depth_calib_data, targetDepth.cols, targetDepth.rows, (const uint16_t*)targetDepth.data, (TY_VECT_3F*)p3d.data, f_scale_unit);
                VideoStreamPtr->PointCloudInit(p3d, &cam_depth_intrinsic[0], timestamp);
            } else if(topics_color_p3d_) {
                TY_CAMERA_EXTRINSIC extri_inv;
                TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
                TYMapDepthImageToPoint3d(&cam_depth_calib_data, targetDepth.cols, targetDepth.rows, (const uint16_t*)targetDepth.data, (TY_VECT_3F*)p3d.data, f_scale_unit);
                TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data, p3d.cols * p3d.rows, (TY_VECT_3F*)p3d.data);
                VideoStreamPtr->PointCloudInit(p3d, &cam_color_intrinsic[0], timestamp);
            }
        } else if(depth.type() == CV_16SC3) {
            for(int i = 0; i < depth.rows; i++) {
                for(int j = 0; j < depth.cols; j++) {
                    p3d.at<cv::Vec3f>(j, i) = static_cast<cv::Vec3f>(p3d.at<cv::Vec3s>(i, j));
                }
            }

            if(topics_p3d_) {
                VideoStreamPtr->PointCloudInit(p3d, &cam_depth_intrinsic[0], timestamp);
            } else if(topics_color_p3d_) {
                TY_CAMERA_EXTRINSIC extri_inv;
                TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
                TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data, p3d.cols * p3d.rows, (TY_VECT_3F*)p3d.data);
                VideoStreamPtr->PointCloudInit(p3d, &cam_color_intrinsic[0], timestamp);
            }
        }
    } 
}

static void PercipioXYZ48ToDepth(cv::Mat& p3d, cv::Mat& depth)
{
    if(p3d.type() != CV_16SC3) return;
    depth = cv::Mat(p3d.size(), CV_16U);
    for(int i = 0; i < p3d.rows; i++) {
        for(int j = 0; j < p3d.cols; j++) {
            depth.at<unsigned short>(i, j) = static_cast<unsigned short>(p3d.at<cv::Vec3s>(i, j)[2]);
        }
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
                RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Device Offline, reconnect ok, now restart stream!");
                _node->setupDevices();
                reconnect = false;
                break;
            }
            MSLEEP(1000);
        }
    }
}

void PercipioDevice::frameDataReceive() {
    TY_STATUS status;
    m_softtrigger_ready = false;

    switch(workmode) {
        case CONTINUS:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "Device now work at continuous mode.");
            break;
        case SOFTTRIGGER:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "Device now work at soft trigger mode.");
            break;
        case HARDTRIGGER:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "Device now work at hard trigger mode.");
            break;
        default:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "Device now work at invalid workmode.");
            break;
    }

    if(enable_rgb_aec_roi) {
        status = TYSetBool(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, true);
        if(status != TY_STATUS_OK) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Enable color aec failed : " << status);
        } else {
            status = TYSetStruct(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_AEC_ROI, &ROI, sizeof(ROI));
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Set color aec roi failed : " << status);
            } else {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "Set color aec roi: " << ROI.x << ", " << ROI.y << ", " << ROI.w << ", " << ROI.h);
            }
        }
    }

    while (rclcpp::ok() && is_running_.load()) {
        TY_FRAME_DATA frame;
        if(workmode == CONTINUS || workmode == HARDTRIGGER) {
            status = TYFetchFrame(handle, &frame, 2000);
        } else if(workmode == SOFTTRIGGER) {
            std::unique_lock<std::mutex> lck(softtrigger_mutex);
            while((!m_softtrigger_ready) && is_running_.load()) {
                softtrigger_detect_cond.wait(lck);
            }
            if(!is_running_.load()) {
                //RCLCPP_INFO(rclcpp::get_logger("percipio_device"), "Stream fecth exit, is_running_ = ." << is_running_);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_camera"), "Stream fecth exit, is_running_ = " << is_running_);
                return;
            }
            m_softtrigger_ready = false;
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "Send soft trigger signal!");
            while(TY_STATUS_BUSY == TYSendSoftTrigger(handle));
            status = TYFetchFrame(handle, &frame, 5000);
            if(status != TY_STATUS_OK) {
                 device_ros_event.eventId = (TY_EVENT)TY_EVENT_DEVICE_TIMEOUT;
                 if(_event_callback)
                    _event_callback(this, &device_ros_event);
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Invalid workmode error :" << workmode);
            status = TYFetchFrame(handle, &frame, 2000);
        }
        if(status == TY_STATUS_OK) {

            int fps = get_fps();
            if(fps > 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "fps = " << fps);
            }

            
            for (int i = 0; i < frame.validCount; i++){
                if (frame.image[i].status != TY_STATUS_OK) continue;

                if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
                    cv::Mat depth;
                    if(frame.image[i].pixelFormat == TY_PIXEL_FORMAT_DEPTH16) {
                        depth = cv::Mat(frame.image[i].height, frame.image[i].width, CV_16U, frame.image[i].buffer);
                        p3dStreamReceive(depth, frame.image[i].timestamp);
                    } else if((uint32_t)frame.image[i].pixelFormat == TY_PIXEL_FORMAT_XYZ48) {
                        cv::Mat p3d = cv::Mat(frame.image[i].height, frame.image[i].width
                          , CV_16SC3, frame.image[i].buffer);
                        p3dStreamReceive(p3d, frame.image[i].timestamp);
                        PercipioXYZ48ToDepth(p3d, depth);
                    }
                    depthStreamReceive(depth, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_RGB_CAM) {
                    cv::Mat color;
                    parseColorFrame(&frame.image[i], &color);
                    colorStreamReceive(color, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT) {
                    cv::Mat IR;
                    parseIrFrame(&frame.image[i], &IR);
                    leftIRStreamReceive(IR, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT) {
                    cv::Mat IR;
                    parseIrFrame(&frame.image[i], &IR);
                    rightIRStreamReceive(IR, frame.image[i].timestamp);
                }
            }

            if(_callback)
               _callback(*VideoStreamPtr.get());

            TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
        }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "frameDataReceive exit...");
}

//开启数据流
bool PercipioDevice::stream_start()
{
    TY_STATUS status;

    TY_TRIGGER_PARAM_EX trigger;
    if(workmode != CONTINUS) {
        TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, true);

        trigger.mode = TY_TRIGGER_MODE_SLAVE;
        status = TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
        if(status != TY_STATUS_OK) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Set device to trigger mode error :" << status);
            return false;
        } else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "Enable device trigger mode.");
        }
    } else {
        TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, b_packet_resend_en);

        //Clear trigger mdoe status
        trigger.mode = TY_TRIGGER_MODE_OFF;
        TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
    }

    uint32_t frameSize;
    status = TYGetFrameBufferSize(handle, &frameSize);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Get frame buffer size error :" << status);
        return false;
    }
    frameBuffer[0].resize(frameSize);
    frameBuffer[1].resize(frameSize);
    
    TYEnqueueBuffer(handle, frameBuffer[0].data(), frameSize);
    TYEnqueueBuffer(handle, frameBuffer[1].data(), frameSize);

    

    status = TYStartCapture(handle);
    if(status != TY_STATUS_OK) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Start capture error :" << status);
      return false;
    }
    is_running_.store(true);
    frame_recive_thread_ = std::make_shared<std::thread>([this]() { frameDataReceive(); });

    return true;
}

bool PercipioDevice::stream_stop()
{
    is_running_.store(false);

    if(workmode == SOFTTRIGGER) {
        std::unique_lock<std::mutex> lck( softtrigger_mutex);
        softtrigger_detect_cond.notify_one();
    }

    if (frame_recive_thread_ && frame_recive_thread_->joinable()) {
        frame_recive_thread_->join();
        TYStopCapture(handle);
        TYClearBufferQueue(handle);
        frameBuffer[0].clear();
        frameBuffer[1].clear();
    }
    return true;
}

void PercipioDevice::send_softtrigger()
{
    if(workmode != SOFTTRIGGER) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "The camera is not working in soft trigger mode, ignore trigger signal");
        return;
    }

    std::unique_lock<std::mutex> lck( softtrigger_mutex);
    m_softtrigger_ready = true;
    softtrigger_detect_cond.notify_one();
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

}
