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

#define LOG_HEAD_PERCIPIO_DEVICE  "percipio_device"

const static int32_t m_Source_Range = 0;
const static int32_t m_Source_Color = 1;
const static int32_t m_Source_LeftIR = 2;
const static int32_t m_Source_RightIR = 3;
static int CamComponentIDToSourceIdx(const TY_COMPONENT_ID comp)
{
    int32_t index = -1;
    switch(comp) {
    case TY_COMPONENT_DEPTH_CAM:
        index = m_Source_Range;
        break;
    case TY_COMPONENT_RGB_CAM:
        index = m_Source_Color;
        break;
    case TY_COMPONENT_IR_CAM_LEFT:
        index = m_Source_LeftIR;
        break;
    case TY_COMPONENT_IR_CAM_RIGHT:
        index = m_Source_RightIR;
        break;
    default:
        break;
    }
    return index;
}

static const char* StreamCompID2GenICamSource(const TY_COMPONENT_ID comp)
{
    const char* genICamSource[] = {
        "Range",
        "Intensity",
        "BinocularLeft",
        "BinocularRight",
    };
    switch(comp){
        case TY_COMPONENT_DEPTH_CAM:
            return genICamSource[0];
        case TY_COMPONENT_RGB_CAM:
            return genICamSource[1];
        case TY_COMPONENT_IR_CAM_LEFT:
            return genICamSource[2];
        case TY_COMPONENT_IR_CAM_RIGHT:
            return genICamSource[3];
        default:
            return nullptr;
    }
}

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

    bool isNetDev = TYIsNetworkInterface(base_info.iface.type);
    if(isNetDev) {
        std::string str_gige_version = base_info.netInfo.tlversion;
        if(str_gige_version == "Gige_2_1") {
            gige_version = GigeE_2_1;
        }
    }

    if(gige_version == GigeE_2_0) {
        status = TYSetEnum(handle, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_HOST);
        if(status != TY_STATUS_OK) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set time sync type(host) err : " << status);
        }
        load_default_parameter();
    } else {
        status = TYEnumSetString(handle, "DeviceTimeSyncMode", "SyncTypeHost");
        if(status != TY_STATUS_OK) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set time sync type(host) err : " << status);
        }
    }

    TYGetComponentIDs(handle, &allComps);

    if(allComps & TY_COMPONENT_RGB_CAM) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_RGB_CAM, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "color stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_RGB_CAM] = image_mode_list;
        TYDisableComponents(handle, TY_COMPONENT_RGB_CAM);
    } else {
        mVideoMode[TY_COMPONENT_RGB_CAM].clear();
    }

    if(allComps & TY_COMPONENT_DEPTH_CAM) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_DEPTH_CAM, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "depth stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_DEPTH_CAM] = image_mode_list;
        TYDisableComponents(handle, TY_COMPONENT_DEPTH_CAM);
    } else {
        mVideoMode[TY_COMPONENT_DEPTH_CAM] .clear();
    }

    if(allComps & TY_COMPONENT_IR_CAM_LEFT) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_IR_CAM_LEFT, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "left-IR stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_IR_CAM_LEFT] = image_mode_list;
        TYDisableComponents(handle, TY_COMPONENT_IR_CAM_LEFT);
    } else {
        mVideoMode[TY_COMPONENT_IR_CAM_LEFT].clear();
    }

    if(allComps & TY_COMPONENT_IR_CAM_RIGHT) {
        std::vector<percipio_video_mode> image_mode_list(0);
        dump_image_mode_list(TY_COMPONENT_IR_CAM_RIGHT, image_mode_list);
        if(image_mode_list.size()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "right-IR stream:");
            for(size_t i = 0; i < image_mode_list.size(); i++) {
                int m_width = image_mode_list[i].width;
                int m_height = image_mode_list[i].height;
                uint32_t m_fmt = image_mode_list[i].fmt;
                std::string format_desc = image_mode_list[i].desc;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "        " << format_desc << " " << m_width << "x" << m_height);
            }
        }

        mVideoMode[TY_COMPONENT_IR_CAM_RIGHT] = image_mode_list;
        TYDisableComponents(handle, TY_COMPONENT_IR_CAM_RIGHT);
    } else {
        mVideoMode[TY_COMPONENT_IR_CAM_RIGHT].clear();
    }

    device_ros_event.eventId = (TY_EVENT)TY_EVENT_DEVICE_CONNECT;

    if(_event_callback)
        _event_callback(this, &device_ros_event);
    
    alive = true;

    return TY_STATUS_OK;
}

TY_STATUS PercipioDevice::dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes)
{
    if(GigeE_2_1 == gige_version) {
        int source = CamComponentIDToSourceIdx(comp);
        if(source < 0) return TY_STATUS_INVALID_COMPONENT;

        TY_STATUS ret = TYEnumSetValue(handle, "SourceSelector", source);
        if(ret) return ret;

        int64_t m_sensor_w, m_sensor_h;
        ret = TYIntegerGetValue(handle, "SensorWidth", &m_sensor_w);
        if(ret) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Read sensor width failed: " << ret);
            return ret;
        }

        ret = TYIntegerGetValue(handle, "SensorHeight", &m_sensor_h);
        if(ret) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Read sensor height failed: " << ret);
            return ret;
        }
        
        uint32_t PixFmtCnt = 0;
        std::vector<TYEnumEntry> PixelFormatList;
        TYEnumGetEntryCount(handle, "PixelFormat", &PixFmtCnt);
        if(PixFmtCnt > 0) {
            PixelFormatList.resize(PixFmtCnt);
            TYEnumGetEntryInfo(handle, "PixelFormat", &PixelFormatList[0], PixFmtCnt, &PixFmtCnt);
        }

        for(size_t i = 0; i < PixFmtCnt; i++) {
            uint32_t Fmt = PixelFormatList[i].value;
            std::string FmtDesc = std::string(PixelFormatList[i].name);
            ret = TYEnumSetValue(handle, "PixelFormat", Fmt);
            if(ret) continue;

            uint32_t BinningCnt = 0;
            std::vector<TYEnumEntry> BinningList;
            TYEnumGetEntryCount(handle, "BinningHorizontal", &BinningCnt);
            if(BinningCnt > 0) {
                BinningList.resize(BinningCnt);
                TYEnumGetEntryInfo( handle, "BinningHorizontal", &BinningList[0], BinningCnt, &BinningCnt);
            }

            for(size_t j = 0; j < BinningCnt; j++) {
                uint32_t binning = BinningList[j].value;
                uint32_t img_width = static_cast<uint32_t>(m_sensor_w / binning);
                uint32_t img_height = static_cast<uint32_t>(m_sensor_h / binning);
                modes.push_back({Fmt, img_width, img_height, BinningList[j].value, FmtDesc});
            }
        }
        return TY_STATUS_OK;

    } else {
        std::vector<TY_ENUM_ENTRY> entrys;
        TY_STATUS ret = get_feature_enum_list(handle, comp, TY_ENUM_IMAGE_MODE, entrys);
        if(ret) return ret;

        for(size_t i = 0; i < entrys.size(); i++) {
            uint32_t fmt = TYPixelFormat(entrys[i].value);
            uint32_t width = TYImageWidth(entrys[i].value);
            uint32_t height = TYImageHeight(entrys[i].value);
            uint32_t binning = 0;
            std::string desc = std::string(entrys[i].description);
            modes.push_back({fmt, width, height, binning, desc});
        }
        return TY_STATUS_OK;
    }
}

TY_STATUS PercipioDevice::image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode)
{
    if(GigeE_2_1 == gige_version) {
        int source = CamComponentIDToSourceIdx(comp);
        if(source < 0) return TY_STATUS_INVALID_COMPONENT;

        TY_STATUS ret = TYEnumSetValue(handle, "SourceSelector", source);
        if(ret) return ret;

        ret = TYEnumSetValue(handle, "PixelFormat", mode.fmt);
        if(ret) return ret;

        return TYEnumSetValue(handle, "BinningHorizontal", mode.binning);
    } else {
        TY_IMAGE_MODE image_enum_mode = TYImageMode2(mode.fmt, mode.width, mode.height);
        return TYSetEnum(handle, comp, TY_ENUM_IMAGE_MODE, image_enum_mode);
    }
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

    //if(workmode == SOFTTRIGGER) {
    //    softtrigger_detect_cond.notify_one();
    //}

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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The CRC check code is empty.");
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
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Storage data length error.");
                    delete []blocks;
                    return false;
                }
                
                crc = crc32_bitwise(huffman_ptr, huffman_size);
                if(crc_data != crc) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Storage area data check failed (check code error).");
                    delete []blocks;
                    return false;
                }

                std::string huffman_string(huffman_ptr, huffman_ptr + huffman_size);
                if(!TextHuffmanDecompression(huffman_string, js_string)) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Huffman decompression error.");
                    delete []blocks;
                    return false;
                }
                break;
            }
            default:
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Unsupported encoding format.");
                delete []blocks;
                return false;
            }
        }
    } else {
        js_string = std::string((const char*)js_code);
    }

    if(!isValidJsonString(js_string.c_str())) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Incorrect json data.");
        delete []blocks;
        return false;
    }

    bool ret =json_parse(handle, js_string.c_str());
    if(ret)  
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Loading default parameters successfully!");
    else
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Failed to load default parameters, some parameters cannot be loaded properly!");

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
    device_reconnect_thread = std::make_unique<std::thread>([this]() { device_offline_reconnect(); });
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
    switch(gige_version) {
        case GigeE_2_1:
        {
            break;
        }
        default:
        {
            bool has = false;
            status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, &has);
            if(status != TY_STATUS_OK) return false;
            if(!has) return false;

            uint32_t m_qua = depth_qua_desc_to_enum(qua);
            status = TYSetEnum(handle, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, m_qua);
            if(status != TY_STATUS_OK) return false;
            break;
        }
    }
    return true;
}

bool PercipioDevice::set_tof_modulation_threshold(int threshold)
{
    TY_STATUS status;
    switch(gige_version) {
        case GigeE_2_1:
        {
            break;
        }
        default:
        {
            bool has = false;
            status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_MODULATION_THRESHOLD, &has);
            if(status != TY_STATUS_OK) return false;
            if(!has) return false;
            status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_MODULATION_THRESHOLD, threshold);
            if(status != TY_STATUS_OK) return false;
            break;
        }
    }
    return true;
}

bool PercipioDevice::set_tof_jitter_threshold(int threshold)
{
    TY_STATUS status;
    switch(gige_version) {
        case GigeE_2_1:
        {
            break;
        }
        default:
        {
            bool has = false;
            status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_JITTER_THRESHOLD, &has);
            if(status != TY_STATUS_OK) return false;
            if(!has) return false;
            status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_JITTER_THRESHOLD, threshold);
            if(status != TY_STATUS_OK) return false;
            break;
        }
    }
    return true;
}

bool PercipioDevice::set_tof_filter_threshold(int threshold)
{
    TY_STATUS status;
    switch(gige_version) {
        case GigeE_2_1:
        {
            break;
        }
        default:
        {
            bool has = false;
            status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_FILTER_THRESHOLD, &has);
            if(status != TY_STATUS_OK) return false;
            if(!has) return false;
            status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_FILTER_THRESHOLD, threshold);
            if(status != TY_STATUS_OK) return false;
            break;
        }
    }
    return true;
}

bool PercipioDevice::set_tof_channel(int chan)
{
    TY_STATUS status;
    switch(gige_version) {
        case GigeE_2_1:
        {
            break;
        }
        default:
        {
            bool has = false;
            status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_CHANNEL, &has);
            if(status != TY_STATUS_OK) return false;
            if(!has) return false;
            status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_CHANNEL, chan);
            if(status != TY_STATUS_OK) return false;
            break;
        }
    }
    return true;
}

bool PercipioDevice::set_tof_HDR_ratio(int ratio)
{
    TY_STATUS status;
    switch(gige_version) {
        case GigeE_2_1:
        {
            break;
        }
        default:
        {
            bool has = false;
            status = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_HDR_RATIO, &has);
            if(status != TY_STATUS_OK) return false;
            if(!has) return false;
            status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_HDR_RATIO, ratio);
            if(status != TY_STATUS_OK) return false;
            break;
        }
    }
    return true;
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

std::string PercipioDevice::parseStreamFormat(const std::string& format)
{
    std::string fmt = format;
    std::transform(fmt.begin(), fmt.end(), fmt.begin(),[](unsigned char c) { return std::tolower(c); });
    if(fmt.find("mono") != std::string::npos) return "mono";
    if(fmt.find("bayer") != std::string::npos) return "bayer";

    if(fmt.find("yuv") != std::string::npos) return "yuv";
    if(fmt.find("ycbcr") != std::string::npos) return "yuv";

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

TY_STATUS PercipioDevice::color_stream_aec_roi_init()
{
    TY_STATUS status;
    if(!enable_rgb_aec_roi) return TY_STATUS_OK;
    if(GigeE_2_1 == gige_version) {
        auto source = StreamCompID2GenICamSource(TY_COMPONENT_RGB_CAM);
        if(!source) return TY_STATUS_INVALID_PARAMETER;

        status = TYEnumSetString(handle, "SourceSelector", source);
        if(status) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "SourceSelector init failed: " << status);
            return status;
        }

        TY_ACCESS_MODE _Access;
        status = TYParamGetAccess(handle, "ExposureAuto", &_Access);
        if(status) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Get ExposureAuto access failed: " << status);
            return status;
        }
        
        if(!(_Access & TY_ACCESS_WRITABLE)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "RGB camera does not support AEC!");
            return TY_STATUS_OK;
        }

        status = TYBooleanSetValue(handle, "ExposureAuto", true);
        if(status) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Enable color aec failed: " << status);
            return status;
        }

        status = TYIntegerSetValue(handle, "AutoFunctionAOIOffsetX", ROI.x);
        if(status) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "AutoFunctionAOIOffsetX write failed: " << status);

        status = TYIntegerSetValue(handle, "AutoFunctionAOIOffsetY", ROI.x);
        if(status) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "AutoFunctionAOIOffsetY write failed: " << status);

        status = TYIntegerSetValue(handle, "AutoFunctionAOIWidth", ROI.x);
        if(status) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "AutoFunctionAOIWidth write failed: " << status);

        status = TYIntegerSetValue(handle, "AutoFunctionAOIHeight", ROI.x);
        if(status) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "AutoFunctionAOIHeight write failed: " << status);
        
        return TY_STATUS_OK;
    }
    
    bool has = false;
    TYHasFeature(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, &has);
    if(!has) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "RGB camera does not support AEC!");
        return TY_STATUS_OK;
    }

    status = TYSetBool(handle, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, true);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Enable color aec failed : " << status);
        return status;
    } else {
        has = false;
        TYHasFeature(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_AEC_ROI, &has);
        if(!has) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The RGB camera does not support AEC ROI metering!");
            return TY_STATUS_OK;
        }

        status = TYSetStruct(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_AEC_ROI, &ROI, sizeof(ROI));
        if(status != TY_STATUS_OK) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set color aec roi failed : " << status);
        } else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set color aec roi: " << ROI.x << ", " << ROI.y << ", " << ROI.w << ", " << ROI.h);
        }
    }
    
    return status;
}

///stream 畸变map 初始化
void PercipioDevice::StreamDistortionMapInit(TY_COMPONENT_ID comp, percipio_distortion_map_info& map)
{
    TY_STATUS status;
    const char* source;
    int64_t m_sensor_w, m_sensor_h;
    int64_t m_sensor_intr_w, m_sensor_intr_h;
    int32_t m_sensor_b;

    int32_t m_width, m_height;
    TY_CAMERA_INTRINSIC intrinsic;
    TY_CAMERA_DISTORTION distortion;

    cv::Mat intrinsic_mat, distortion_mat;
    cv::Mat mapX, mapY;

    if(GigeE_2_0 == gige_version) {
        status = TYGetInt(handle, comp, TY_INT_WIDTH, &m_width);
        if(status) goto INIT_FAIL;

        status = TYGetInt(handle, comp, TY_INT_HEIGHT, &m_height);
        if(status) goto INIT_FAIL;
        
        status = TYGetStruct(handle, comp, TY_STRUCT_CAM_INTRINSIC, &intrinsic, sizeof(intrinsic));
        if(status) goto INIT_FAIL;
        
        status = TYGetStruct(handle, comp, TY_STRUCT_CAM_DISTORTION, &distortion, sizeof(distortion));
        if(status) goto INIT_FAIL;
    } else {
        source = StreamCompID2GenICamSource(comp);
        if(!source) goto INIT_FAIL;

        status = TYEnumSetString(handle, "SourceSelector", source);
        if(status) goto INIT_FAIL;

        status = TYIntegerGetValue(handle, "SensorWidth", &m_sensor_w);
        if(!source) goto INIT_FAIL;

        status = TYIntegerGetValue(handle, "SensorHeight", &m_sensor_h);
        if(!source) goto INIT_FAIL;

        status = TYEnumGetValue(handle, "BinningHorizontal", &m_sensor_b);
        if(!source) goto INIT_FAIL;

        status = TYIntegerGetValue(handle, "IntrinsicWidth", &m_sensor_intr_w);
        if(!source) goto INIT_FAIL;

        status = TYIntegerGetValue(handle, "IntrinsicHeight", &m_sensor_intr_h);
        if(!source) goto INIT_FAIL;

        double f_intrinsic[9];
        status = TYByteArrayGetValue(handle, "Intrinsic", (uint8_t*)f_intrinsic, sizeof(f_intrinsic));
        if(!source) goto INIT_FAIL;

        double f_distortion[12];
        status = TYByteArrayGetValue(handle, "Distortion", (uint8_t*)f_distortion, sizeof(f_distortion));
        if(!source) goto INIT_FAIL;

        m_width = m_sensor_w / m_sensor_b;
        m_height = m_sensor_h / m_sensor_b;

        /// | fx|  0| cx|
        /// |  0| fy| cy|
        /// |  0|  0|  1|
        intrinsic.data[0] = f_intrinsic[0] / m_sensor_b;
        intrinsic.data[1] = 0;
        intrinsic.data[2] = f_intrinsic[2] / m_sensor_b;

        intrinsic.data[3] = 0;
        intrinsic.data[4] = f_intrinsic[4] / m_sensor_b;
        intrinsic.data[5] = f_intrinsic[5] / m_sensor_b;

        intrinsic.data[6] = 0;
        intrinsic.data[7] = 0;
        intrinsic.data[8] = 1;
        
        for(size_t i = 0; i < 12; i++) {
            distortion.data[i] = f_distortion[i];
        }
    }

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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid stream!");
        return false;
    }

    auto desc = StreamCompID2GenICamSource(m_comp);
    if((m_comp & allComps) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << "Unsupported component!");
        return false;
    }

    int img_width, img_height;
    std::vector<percipio_video_mode> video_mode_val_list(0);
    bool valid_resolution = resolveStreamResolution(resolution, img_width, img_height);
    auto VideoModeList = mVideoMode[m_comp];
    if(VideoModeList.size()) {
        for(auto video_mode : VideoModeList) {
            if(valid_resolution) {
                if(img_width != video_mode.width ||
                   img_height != video_mode.height) {
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
            status = image_mode_cfg(m_comp, video_mode_val_list[0]);
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Stream mode init error: " << status);
            } else {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Set stream mode: " << video_mode_val_list[0].desc << " " 
                        << video_mode_val_list[0].width << "x" << video_mode_val_list[0].height);
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Unsupported stream mode: " << resolution);
        }
    }

    status = TYEnableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Stream open error: " << status);
        return false;
    }

    switch (m_comp) {
        case TY_COMPONENT_DEPTH_CAM: {
            TYGetFloat(handle, m_comp, TY_FLOAT_SCALE_UNIT, &f_scale_unit);
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth stream scale unit: " << f_scale_unit);

            status = TYGetStruct(handle, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &cam_depth_calib_data, sizeof(cam_depth_calib_data));
            if(status != TY_STATUS_OK) {
                has_depth_calib_data = false;
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Got stream calib data error: " << status);
            } else {
                has_depth_calib_data = true;
                cam_depth_intrinsic = image_intrinsic(cam_depth_calib_data.intrinsicWidth, cam_depth_calib_data.intrinsicHeight, cam_depth_calib_data.intrinsic);
            }

            depth_stream_distortion_check();
            break;
        }
        case TY_COMPONENT_RGB_CAM: {
            status = TYGetStruct(handle, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &cam_color_calib_data, sizeof(cam_color_calib_data));
            if(status != TY_STATUS_OK) {
                has_color_calib_data = false;
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Got stream calib data error:" << status);
            } else {
                has_color_calib_data = true;
                cam_color_intrinsic = image_intrinsic(cam_color_calib_data.intrinsicWidth, cam_color_calib_data.intrinsicHeight, cam_color_calib_data.intrinsic);
            }
            break;
        }
        case TY_COMPONENT_IR_CAM_LEFT: {
            TY_CAMERA_CALIB_INFO calib_data;
            TYGetStruct(handle, m_comp, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(calib_data));
            cam_leftir_intrinsic = image_intrinsic(calib_data.intrinsicWidth, calib_data.intrinsicHeight, calib_data.intrinsic);
            break;
        }
        case TY_COMPONENT_IR_CAM_RIGHT: {
            TY_CAMERA_CALIB_INFO calib_data;
            TYGetStruct(handle, m_comp, TY_STRUCT_CAM_CALIB_DATA,&calib_data, sizeof(calib_data));
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

    auto desc = StreamCompID2GenICamSource(m_comp);
    if((m_comp & allComps) != m_comp) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Unsupported component!");
        return false;
    }

    status = TYDisableComponents(handle, m_comp);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), desc << ": Stream close error: " << status);
        return false;
    }

    return true;
}

//
void PercipioDevice::colorStreamReceive(const cv::Mat& color, uint64_t& timestamp)
{
    cv::Mat mapX, mapY;
    cv::Mat targetRGB;

    if(color.empty()) return;
    if(VideoStreamPtr) {
        if(has_color_calib_data) {
            if(color.type() == CV_8UC3) {
                targetRGB = color.clone();

                TY_IMAGE_DATA src;
                src.width = color.cols;
                src.height = color.rows;
                src.size = color.size().area() * 3;;
                src.pixelFormat = TYPixelFormatBGR8;
                src.buffer = color.data;

                TY_IMAGE_DATA dst;
                dst.width = color.cols;
                dst.height = color.rows;
                dst.size = color.size().area() * 3;;
                dst.pixelFormat = TYPixelFormatBGR8;
                dst.buffer = targetRGB.data;
                TY_STATUS err = TYUndistortImage(&cam_color_calib_data, &src, NULL, &dst);
                if(err) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Color TYUndistortImage ret = :" << err);
                }
            } else {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid color stream fmt :" << color.type());
                return;
            }
        } else {
            targetRGB = color;
        }
        VideoStreamPtr->ColorInit(targetRGB, cam_color_intrinsic, timestamp);
    }
}

void PercipioDevice::leftIRStreamReceive(const cv::Mat& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRLeftInit(ir, cam_leftir_intrinsic, timestamp);
}

void PercipioDevice::rightIRStreamReceive(const cv::Mat& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRRightInit(ir, cam_rightir_intrinsic, timestamp);
}

void PercipioDevice::depthStreamReceive(const cv::Mat& depth, uint64_t& timestamp)
{
    cv::Mat targetDepth;
    if(depth.empty()) return;

    if(depth.type() != CV_16U && depth.type() != CV_16SC3) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid depth stream fmt :" << depth.type());
        return;
    }
    if(!VideoStreamPtr) return;

    if(depth.type() == CV_16U) {
        if(b_need_do_depth_undistortion) {
            targetDepth = depth.clone();

            TY_IMAGE_DATA src;
            src.width = depth.cols;
            src.height = depth.rows;
            src.size = depth.size().area() * 2;
            src.pixelFormat = TYPixelFormatCoord3D_C16;
            src.buffer = depth.data;

            TY_IMAGE_DATA dst;
            dst.width = depth.cols;
            dst.height = depth.rows;
            dst.size = depth.size().area() * 2;
            dst.pixelFormat = TYPixelFormatCoord3D_C16;
            dst.buffer = targetDepth.data;
            
            TY_STATUS err = TYUndistortImage(&cam_depth_calib_data, &src, NULL, &dst);
            if(err) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth TYUndistortImage err:" << err);
        } else {
            targetDepth = depth;
        }

        if(topics_d_registration_) {
            cv::Mat out = cv::Mat::zeros(targetDepth.size(), CV_16U);
            TYMapDepthImageToColorCoordinate(&cam_depth_calib_data,
                targetDepth.cols, targetDepth.rows, targetDepth.ptr<uint16_t>(),
                &cam_color_calib_data,
                out.cols, out.rows, out.ptr<uint16_t>(), f_scale_unit);
                targetDepth = out.clone();
            
            VideoStreamPtr->DepthInit(targetDepth, cam_color_intrinsic, timestamp);
        } else if(topics_depth_) {
            VideoStreamPtr->DepthInit(targetDepth, cam_depth_intrinsic, timestamp);
        }
    } else if(depth.type() == CV_16SC3) {
        if(topics_d_registration_) {
            cv::Point3f point(
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()
                );
            cv::Mat p3d = cv::Mat(depth.size(), CV_32FC3);
            for(int i = 0; i < depth.rows; i++) {
                for(int j = 0; j < depth.cols; j++) {
                    if(depth.at<cv::Vec3s>(i, j)[2]) {
                        p3d.at<cv::Vec3f>(i, j) = static_cast<cv::Vec3f>(depth.at<cv::Vec3s>(i, j));
                    } else {
                        p3d.at<cv::Vec3f>(i, j) = point;
                    }
                }
            }

            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data, p3d.cols * p3d.rows, (TY_VECT_3F*)p3d.data);

            targetDepth = cv::Mat::zeros(depth.size(), CV_16U);
            TYMapPoint3dToDepthImage(&cam_color_calib_data, (const TY_VECT_3F*)(p3d.data), p3d.cols * p3d.rows, p3d.cols, p3d.rows, (uint16_t*)(targetDepth.data));
            VideoStreamPtr->DepthInit(targetDepth, cam_color_intrinsic,timestamp);
        } else if(topics_depth_) {
            targetDepth = depth;
            VideoStreamPtr->DepthInit(targetDepth, cam_depth_intrinsic, timestamp);
        }
    } else 
        ;

    return;
}

void PercipioDevice::p3dStreamReceive(const cv::Mat& depth, uint64_t& timestamp) {
    if(depth.empty()) return;
    if(!topics_p3d_  && !topics_color_p3d_) return;
    if(!VideoStreamPtr) return;

    cv::Mat p3d = cv::Mat(depth.size(), CV_32FC3);
    if(depth.type() == CV_16U) {
        cv::Mat targetDepth;
        if(b_need_do_depth_undistortion) {
            targetDepth = depth.clone();
    
            TY_IMAGE_DATA src;
            src.width = depth.cols;
            src.height = depth.rows;
            src.size = depth.size().area() * 2;
            src.pixelFormat = TYPixelFormatCoord3D_C16;
            src.buffer = depth.data;
    
            TY_IMAGE_DATA dst;
            dst.width = depth.cols;
            dst.height = depth.rows;
            dst.size = depth.size().area() * 2;
            dst.pixelFormat = TYPixelFormatCoord3D_C16;
            dst.buffer = targetDepth.data;
            TY_STATUS err = TYUndistortImage(&cam_depth_calib_data, &src, NULL, &dst);
            if(err) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth TYUndistortImage ret = :" << err);
            }
        } else {
            targetDepth = depth;
        }
            
        if(topics_p3d_) {
            TYMapDepthImageToPoint3d(&cam_depth_calib_data, targetDepth.cols, targetDepth.rows, (const uint16_t*)targetDepth.data, (TY_VECT_3F*)p3d.data, f_scale_unit);
            VideoStreamPtr->PointCloudInit(p3d, cam_depth_intrinsic, timestamp);
        } else if(topics_color_p3d_) {
            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
            TYMapDepthImageToPoint3d(&cam_depth_calib_data, targetDepth.cols, targetDepth.rows, (const uint16_t*)targetDepth.data, (TY_VECT_3F*)p3d.data, f_scale_unit);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data, p3d.cols * p3d.rows, (TY_VECT_3F*)p3d.data);
            VideoStreamPtr->PointCloudInit(p3d, cam_color_intrinsic, timestamp);
        }
    } else if(depth.type() == CV_16SC3) {
        for(int i = 0; i < depth.rows; i++) {
            for(int j = 0; j < depth.cols; j++) {
                if(depth.at<cv::Vec3s>(i, j)[2]) {
                    p3d.at<cv::Vec3f>(i, j) = static_cast<cv::Vec3f>(depth.at<cv::Vec3s>(i, j));
                } else {
                    p3d.at<cv::Vec3f>(i, j) = cv::Point3f(
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN()
                    );
                }
            }
        }
        if(topics_p3d_) {
            VideoStreamPtr->PointCloudInit(p3d, cam_depth_intrinsic, timestamp);
        } else if(topics_color_p3d_) {
            TY_CAMERA_EXTRINSIC extri_inv;
            TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
            TYMapPoint3dToPoint3d(&extri_inv, (TY_VECT_3F*)p3d.data, p3d.cols * p3d.rows, (TY_VECT_3F*)p3d.data);
            VideoStreamPtr->PointCloudInit(p3d, cam_color_intrinsic, timestamp);
        }
    } else
        ; 
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
                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device Offline, reconnect ok, now restart stream!");
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
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Device now work at invalid workmode.");
            break;
    }

    while (rclcpp::ok() && is_running_.load()) {
        TY_FRAME_DATA frame;
        if(workmode == CONTINUS || workmode == HARDTRIGGER) {
            status = TYFetchFrame(handle, &frame, 2000);
        } else if(workmode == SOFTTRIGGER) {
            status = TYFetchFrame(handle, &frame, 200);
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Invalid workmode error :" << workmode);
            status = TYFetchFrame(handle, &frame, 2000);
        }

        if(status == TY_STATUS_OK) {
            int fps = get_fps();
            if(fps > 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "fps = " << fps);
            }

            for (int i = 0; i < frame.validCount; i++){
                if (frame.image[i].status != TY_STATUS_OK) continue;

                if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
                    cv::Mat depth;
                    if(frame.image[i].pixelFormat == TYPixelFormatCoord3D_C16) {
                        if(b_depth_spk_filter_en) {
                            DepthSpkFilterPara param = {m_depth_spk_size, m_depth_spk_diff};
                            TYDepthSpeckleFilter(frame.image[i], param);
                        }

                        if(b_depth_time_domain_en) {
                            DepthDomainTimeFilterMgrPtr->add_frame(frame.image[i]);
                            if(!DepthDomainTimeFilterMgrPtr->do_time_domain_process(frame.image[i])) {
                                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Do Time-domain filter, drop frame!");
                                continue;
                            }
                        }
                        depth = cv::Mat(frame.image[i].height, frame.image[i].width, CV_16U, frame.image[i].buffer);
                    } else if((uint32_t)frame.image[i].pixelFormat == TYPixelFormatCoord3D_ABC16) {
                        depth = cv::Mat(frame.image[i].height, frame.image[i].width
                          , CV_16SC3, frame.image[i].buffer);
                    }
                    depthStreamReceive(depth, frame.image[i].timestamp);
                    p3dStreamReceive(depth, frame.image[i].timestamp);
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

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "frameDataReceive exit...");
}

//开启数据流
bool PercipioDevice::stream_start()
{
    TY_STATUS status;
    TY_ACCESS_MODE access;
    switch(gige_version) {
        case GigeE_2_1:
        {
            if(workmode == CONTINUS) {
                //CONTINUS
                status = TYEnumSetString(handle, "AcquisitionMode", "Continuous");
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set AcquisitionMode error :" << status);
                    return false;
                }
                status = TYParamGetAccess(handle, "AcquisitionFrameRateEnable", &access);
                if((status == TY_STATUS_OK) && (access & TY_ACCESS_WRITABLE)) {
                    status = TYBooleanSetValue(handle, "AcquisitionFrameRateEnable", false);
                    if(status != TY_STATUS_OK) {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set AcquisitionFrameRateEnable error :" << status);
                        return false;
                    }
                }
            } else if(workmode == SOFTTRIGGER) {
                //SOFTTRIGGER
                status = TYEnumSetString(handle, "AcquisitionMode", "SingleFrame");
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set AcquisitionMode error :" << status);
                    return false;
                }

                status = TYEnumSetString(handle, "TriggerSource", "Software");
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set TriggerSource to  Software error :" << status);
                    return false;
                }
            } else {
                //HARDTRIGGER
                status = TYEnumSetString(handle, "AcquisitionMode", "SingleFrame");
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set AcquisitionMode error :" << status);
                    return false;
                }

                status = TYEnumSetString(handle, "TriggerSource", "Line0");
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set TriggerSource to  Line0 error :" << status);
                    return false;
                }
            }
            break;
        }
        default:
        {
            TY_TRIGGER_PARAM_EX trigger;
            if(workmode != CONTINUS) {
                TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, true);

                trigger.mode = TY_TRIGGER_MODE_SLAVE;
                status = TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Set device to trigger mode error :" << status);
                    return false;
                } else {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Enable device trigger mode.");
                }
            } else {
                TYSetBool(handle, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, b_packet_resend_en);

                //Clear trigger mdoe status
                trigger.mode = TY_TRIGGER_MODE_OFF;
                TYSetStruct(handle, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
            }
            break;
        }
    }

    uint32_t frameSize;
    status = TYGetFrameBufferSize(handle, &frameSize);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Get frame buffer size error :" << status);
        return false;
    }
    frameBuffer[0].resize(frameSize);
    frameBuffer[1].resize(frameSize);
    
    TYEnqueueBuffer(handle, frameBuffer[0].data(), frameSize);
    TYEnqueueBuffer(handle, frameBuffer[1].data(), frameSize);

    status = TYStartCapture(handle);
    if(status != TY_STATUS_OK) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Start capture error :" << status);
      return false;
    }

    color_stream_aec_roi_init();

    is_running_.store(true);
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
    //if(workmode == SOFTTRIGGER) {
    //    softtrigger_detect_cond.notify_one();
    //}

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

void PercipioDevice::depth_stream_distortion_check()
{
    TY_STATUS ret;
    TY_ACCESS_MODE _access;
    switch(gige_version) {
        case GigeE_2_1:
        {
            ret = TYEnumSetString(handle, "SourceSelector", "Range");
            if(ret) {
                b_need_do_depth_undistortion = false;
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "TYEnumSetString set SourceSelector to range failed: " << ret);
                return;
            }

            ret = TYParamGetAccess(handle, "Distortion", &_access);
            if(ret) {
                b_need_do_depth_undistortion = false;
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "TYParamGetAccess get Distortion failed: " << ret);
                return;
            }

            if(_access & TY_ACCESS_READABLE) {
                b_need_do_depth_undistortion = true;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Range  distortion is readable!");
            } else {
                b_need_do_depth_undistortion = false;
                RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Range distortion is not readable!");
            }
            break;
        }
        default:
        {
            ret = TYHasFeature(handle, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_DISTORTION, &b_need_do_depth_undistortion);
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "Depth distortion calib data check ret:" << ret 
                    << "(" << b_need_do_depth_undistortion << ")");
        }
    }
    return;
}

void PercipioDevice::send_softtrigger()
{
    if(workmode != SOFTTRIGGER) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_DEVICE), "The camera is not working in soft trigger mode, ignore trigger signal");
        return;
    }

    //std::unique_lock<std::mutex> lck( softtrigger_mutex);
    //m_softtrigger_ready = true;
    //softtrigger_detect_cond.notify_one();
    while(TY_STATUS_BUSY == TYSendSoftTrigger(handle));
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

void PercipioDevice::depth_speckle_filter_init(bool enable, int spec_size, int spec_diff)
{
    b_depth_spk_filter_en = enable;
    m_depth_spk_size = spec_size;
    m_depth_spk_diff = spec_diff;
}

void PercipioDevice::dpeth_time_domain_filter_init(bool enable, int number)
{
    b_depth_time_domain_en = enable;
    m_depth_time_domain_frame_num = number;
    DepthDomainTimeFilterMgrPtr->reset(m_depth_time_domain_frame_num);
}

}
