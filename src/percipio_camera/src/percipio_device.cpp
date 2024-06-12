#include "percipio_device.h"
#include "common.hpp"
#include "TYCoordinateMapper.h"

namespace percipio_camera {

#define INVALID_COMPONENT_ID        (0xFFFFFFFF)

PercipioDevice::PercipioDevice(const char* faceId, const char* deviceId)
    : alive(false),
      hIface(nullptr),
      handle(nullptr)
{
    TY_STATUS status;
    status = TYOpenInterface(faceId, &hIface);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Open interface fail!");
        return;
    }
  
    status = TYOpenDevice(hIface, deviceId, &handle);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Open device fail!");
        TYCloseInterface(hIface);
        return;
    }

    status = TYGetDeviceInfo(handle, &base_info);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Invalid device handle!");
        TYCloseDevice(handle);
        TYCloseInterface(hIface);
        return;
    }

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
    }
    
    alive = true;
}

PercipioDevice::~PercipioDevice()
{
    is_running_.store(false);
    if (frame_recive_thread_ && frame_recive_thread_->joinable()) {
        frame_recive_thread_->join();
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

static void eventCallback(TY_EVENT_INFO *event_info, void *userdata) {
    PercipioDevice* handle = (PercipioDevice*)userdata;
    handle->_event_callback(handle, event_info);
}

void PercipioDevice::registerCameraEventCallback(PercipioDeviceEventCallbackFunction callback)
{
    if(alive) {
       _event_callback = callback;
        TYRegisterEventCallback(handle, eventCallback, this);
   }
}

std::string PercipioDevice::serialNumber()
{
    return std::string(base_info.id);
}

std::string PercipioDevice::modelName()
{
    return std::string(base_info.modelName);
}

std::string PercipioDevice::buildHash()
{
    return std::string(base_info.buildHash);
}

std::string PercipioDevice::configVersion()
{
    return std::string(base_info.configVersion);
}

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
    status = TYSetInt(handle, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, m_qua);
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
        }
    }

    if(valid_resolution) {
        if(image_mode_val_list.size()) {
            status = TYSetEnum(handle, m_comp, TY_ENUM_IMAGE_MODE, image_mode_val_list[0]);
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_device"), "Stream mode init error :" << status);
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

    StreamDistortionMapInit(TY_COMPONENT_DEPTH_CAM, depth_map);
    StreamDistortionMapInit(TY_COMPONENT_RGB_CAM, color_map);

    VideoStreamPtr = std::make_shared<VideoStream>();
    return true;
}

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


void PercipioDevice::colorStreamRecive(cv::Mat& color, uint64_t& timestamp)
{
    cv::Mat mapX, mapY;
    cv::Mat targetRGB;

    if(color.empty()) return;
    if(VideoStreamPtr) {
        if(color_map.IsValid()) {
            mapX = cv::Mat(cv::Size(color_map.m_map_width, color_map.m_map_height), CV_32F, color_map.f_map_x.data());
            mapY = cv::Mat(cv::Size(color_map.m_map_width, color_map.m_map_height), CV_32F, color_map.f_map_y.data());
            cv::remap(color, targetRGB, mapX, mapY, 0);
        } else {
            targetRGB = color;
        }
        VideoStreamPtr->ColorInit(targetRGB, &cam_color_intrinsic[0], timestamp);
    }
}

void PercipioDevice::leftIRStreamRecive(cv::Mat& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRLeftInit(ir, &cam_leftir_intrinsic[0], timestamp);
}

void PercipioDevice::rightIRStreamRecive(cv::Mat& ir, uint64_t& timestamp)
{
    if(ir.empty()) return;
    if(VideoStreamPtr) VideoStreamPtr->IRRightInit(ir, &cam_rightir_intrinsic[0], timestamp);
}

void PercipioDevice::depthStreamRecive(cv::Mat& depth, uint64_t& timestamp)
{
    cv::Mat mapX, mapY;
    cv::Mat targetDepth;

    if(depth.empty()) return;
    if(VideoStreamPtr) {
        if(depth_map.IsValid()) {
            mapX = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_x.data());
            mapY = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_y.data());
            cv::remap(depth, targetDepth, mapX, mapY, 0);
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
            VideoStreamPtr->DepthInit(targetDepth, &cam_color_intrinsic[0], timestamp);
        } else if(topics_depth_) {
            VideoStreamPtr->DepthInit(targetDepth, &cam_depth_intrinsic[0],timestamp);
        }
    }
    return;
}

void PercipioDevice::p3dStreamRecive(cv::Mat& depth, uint64_t& timestamp) {
    if(depth.empty()) return;
    if(!topics_p3d_  && !topics_color_p3d_) return;

    if(VideoStreamPtr) {
        cv::Mat p3d = cv::Mat(depth.size(), CV_32FC3);
        if(depth.type() == CV_16U) {
            cv::Mat targetDepth;
            if(depth_map.IsValid()) {
                cv::Mat mapX = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_x.data());
                cv::Mat mapY = cv::Mat(cv::Size(depth_map.m_map_width, depth_map.m_map_height), CV_32F, depth_map.f_map_y.data());
                cv::remap(depth, targetDepth, mapX, mapY, 0);
            } else {
                targetDepth = depth;
            }

            if(topics_p3d_) {
                TYMapDepthImageToPoint3d(&cam_depth_calib_data, targetDepth.cols, targetDepth.rows, (const uint16_t*)targetDepth.data, (TY_VECT_3F*)p3d.data, f_scale_unit);
                VideoStreamPtr->PointCloudInit(p3d, &cam_depth_intrinsic[0], timestamp);
            } else if(topics_color_p3d_) {
                TY_CAMERA_EXTRINSIC extri_inv;
                TYInvertExtrinsic(&cam_color_calib_data.extrinsic, &extri_inv);
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
            depth.at<unsigned short>(j, i) = static_cast<unsigned short>(p3d.at<cv::Vec3s>(i, j)[2]);
        }
    }
}

void PercipioDevice::frameDataRecive() {
    TY_STATUS status;

    while (rclcpp::ok() && is_running_) {
        TY_FRAME_DATA frame;
        status = TYFetchFrame(handle, &frame, 2000);
        if(status == TY_STATUS_OK) {
            for (int i = 0; i < frame.validCount; i++){
                if (frame.image[i].status != TY_STATUS_OK) continue;

                if (frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
                    cv::Mat depth;
                    if(frame.image[i].pixelFormat == TY_PIXEL_FORMAT_DEPTH16) {
                        depth = cv::Mat(frame.image[i].height, frame.image[i].width, CV_16U, frame.image[i].buffer);
                        p3dStreamRecive(depth, frame.image[i].timestamp);
                    } else if((uint32_t)frame.image[i].pixelFormat == TY_PIXEL_FORMAT_XYZ48) {
                        cv::Mat p3d = cv::Mat(frame.image[i].height, frame.image[i].width
                          , CV_16SC3, frame.image[i].buffer);
                        p3dStreamRecive(p3d, frame.image[i].timestamp);
                        PercipioXYZ48ToDepth(p3d, depth);
                    }
                    depthStreamRecive(depth, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_RGB_CAM) {
                    cv::Mat color;
                    parseColorFrame(&frame.image[i], &color);
                    colorStreamRecive(color, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT) {
                    cv::Mat IR;
                    parseIrFrame(&frame.image[i], &IR);
                    leftIRStreamRecive(IR, frame.image[i].timestamp);
                }

                if (frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT) {
                    cv::Mat IR;
                    parseIrFrame(&frame.image[i], &IR);
                    rightIRStreamRecive(IR, frame.image[i].timestamp);
                }
            }

            if(_callback)
               _callback(*VideoStreamPtr.get());

            TYEnqueueBuffer(handle, frame.userBuffer, frame.bufferSize);
        }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("percipio_device"), "frameDataRecive exit...");
}

bool PercipioDevice::stream_start()
{
    TY_STATUS status;
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
    frame_recive_thread_ = std::make_shared<std::thread>([this]() { frameDataRecive(); });

    return true;
}

bool PercipioDevice::stream_stop()
{
    is_running_.store(false);
    if (frame_recive_thread_ && frame_recive_thread_->joinable()) {
        frame_recive_thread_->join();
        TYStopCapture(handle);
        TYClearBufferQueue(handle);
        frameBuffer[0].clear();
        frameBuffer[1].clear();
    }
    return true;
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