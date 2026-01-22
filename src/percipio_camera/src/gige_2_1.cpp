#include "gige_2_1.h"

namespace percipio_camera {

#define LOG_HEAD_GIGE_2_1  "GIGE2_1"

const static int64_t m_Source_Range = 0;
const static int64_t m_Source_Color = 1;
const static int64_t m_Source_LeftIR = 2;
const static int64_t m_Source_RightIR = 3;
static int64_t CamComponentIDToSourceIdx(const TY_COMPONENT_ID comp)
{
    int64_t index = -1;
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


GigE_2_1::GigE_2_1(const TY_DEV_HANDLE dev) : GigEBase(dev)
{
    TYGetComponentIDs(hDevice, &allComps);
}

TY_STATUS GigE_2_1::init()
{
    TY_STATUS status = TYEnumSetString(hDevice, "DeviceTimeSyncMode", "SyncTypeHost");
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Set time sync type(host) error: " << status);
    }

    video_mode_init();
    return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes)
{
    int64_t source = CamComponentIDToSourceIdx(comp);
    if(source < 0) return TY_STATUS_INVALID_COMPONENT;

    TY_STATUS ret = source_init(source);
    if(ret) return ret;

    int64_t m_sensor_w, m_sensor_h;
    ret = TYIntegerGetValue(hDevice, "SensorWidth", &m_sensor_w);
    if(ret) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Read sensor width failed: " << ret);
        return ret;
    }

    ret = TYIntegerGetValue(hDevice, "SensorHeight", &m_sensor_h);
    if(ret) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Read sensor height failed: " << ret);
        return ret;
    }
    
    uint32_t PixFmtCnt = 0;
    std::vector<TYEnumEntry> PixelFormatList;
    TYEnumGetEntryCount(hDevice, "PixelFormat", &PixFmtCnt);
    if(PixFmtCnt > 0) {
        PixelFormatList.resize(PixFmtCnt);
        TYEnumGetEntryInfo(hDevice, "PixelFormat", &PixelFormatList[0], PixFmtCnt, &PixFmtCnt);
    }

    int64_t m_def_fmt = 0;
    ret = TYEnumGetValue(hDevice, "PixelFormat", &m_def_fmt);
    if(ret) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Read sensor default pixel format failed: " << ret);
        return ret;
    }

    int64_t m_def_binning = 0;
    ret = TYEnumGetValue(hDevice, "BinningHorizontal", &m_def_binning);
    if(ret) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Read sensor default binning failed: " << ret);
        return ret;
    }

    for(size_t i = 0; i < PixFmtCnt; i++) {
        uint32_t Fmt = PixelFormatList[i].value;
        std::string FmtDesc = std::string(PixelFormatList[i].name);
        ret = TYEnumSetValue(hDevice, "PixelFormat", Fmt);
        if(ret) continue;

        uint32_t BinningCnt = 0;
        std::vector<TYEnumEntry> BinningList;
        TYEnumGetEntryCount(hDevice, "BinningHorizontal", &BinningCnt);
        if(BinningCnt > 0) {
            BinningList.resize(BinningCnt);
            TYEnumGetEntryInfo( hDevice, "BinningHorizontal", &BinningList[0], BinningCnt, &BinningCnt);
        }

        for(size_t j = 0; j < BinningCnt; j++) {
            uint32_t binning = BinningList[j].value;
            uint32_t img_width = static_cast<uint32_t>(m_sensor_w / binning);
            uint32_t img_height = static_cast<uint32_t>(m_sensor_h / binning);
            modes.push_back({Fmt, img_width, img_height, static_cast<uint32_t>(BinningList[j].value), FmtDesc});
        }
    }

    ret = TYEnumSetValue(hDevice, "PixelFormat", m_def_fmt);
    if(ret) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Write sensor default pixel format failed: " << ret);
        return ret;
    }

    ret = TYEnumSetValue(hDevice, "BinningHorizontal", m_def_binning);
    if(ret) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Write sensor default binning failed: " << ret);
        return ret;
    }

    return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode)
{
    int64_t source = CamComponentIDToSourceIdx(comp);
    if(source < 0) return TY_STATUS_INVALID_COMPONENT;

    TY_STATUS ret = source_init(source);
    if(ret) return ret;

    ret = TYEnumSetValue(hDevice, "PixelFormat", mode.fmt);
    if(ret) return ret;

    return TYEnumSetValue(hDevice, "BinningHorizontal", mode.binning);
}

TY_STATUS GigE_2_1::work_mode_init(percipio_dev_workmode mode, const bool fix_rate, const float rate)
{
    TY_STATUS status = TY_STATUS_OK;
    soft_frame_rate_ctrl_enable = false;
    switch(mode) {
        case CONTINUS: {
            status = TYEnumSetString(hDevice, "AcquisitionMode", "Continuous");
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to set AcquisitionMode to Continuous mode, error: " << status);
                return status;
            }

            if(fix_rate) { //try fix frame rate
                float f_real_frame_rate = rate;
                status = fix_device_frame_rate(f_real_frame_rate);
                if(status) {
                    status = fix_device_frame_rate_in_soft_trigger_mode(f_real_frame_rate);
                    if(status) {
                        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "The device does not support the automatic fixed frame rate output mode.");
                        return status;
                    }
                }
            } else {
                status = TYBooleanSetValue(hDevice, "AcquisitionFrameRateEnable", false);
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to set AcquisitionFrameRateEnable to false, error: " << status);
                    return status;
                }
            }
            break;
        }
        case SOFTTRIGGER: {
            status = TYEnumSetString(hDevice, "AcquisitionMode", "SingleFrame");
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Set AcquisitionMode error: " << status);
                return status;
            }

            status = TYEnumSetString(hDevice, "TriggerSource", "Software");
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Set TriggerSource to  Software error: " << status);
                return status;
            }
            break;
        }
        case HARDTRIGGER: {
            status = TYEnumSetString(hDevice, "AcquisitionMode", "SingleFrame");
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Set AcquisitionMode error: " << status);
                return status;
            }

            status = TYEnumSetString(hDevice, "TriggerSource", "Line0");
            if(status != TY_STATUS_OK) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Set TriggerSource to  Line0 error: " << status);
                return status;
            }
            break;
        }
        default: {
            return TY_STATUS_INVALID_PARAMETER;
        }
    }
    return TY_STATUS_OK;
}

void GigE_2_1::device_load_parameters()
{
    TY_STATUS status;
    std::string  m_current_source;
    for(auto& iter : parameters) {
        static std::set<std::string> source_list = {
            "Depth", "Texture", "Left", "Right"
        };

        const std::string& source = iter.first;
        auto target_source = source_list.find(source);
        if(target_source != source_list.end()) {
            char szSourceString[200];
            status = TYEnumGetString(hDevice, "SourceSelector", szSourceString, sizeof(szSourceString));
            if(status) {
                m_current_source = "";
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to read SourceSelector: " << status);
            } else {
                m_current_source = std::string(szSourceString);
            }

            if(m_current_source != source) {
                status = TYEnumSetString(hDevice, "SourceSelector", source.c_str());
                if(status) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to write SourceSelector: " << status);
                    continue;
                }
                m_current_source = source;
            }
        }

        for(auto& feat : iter.second) {
            const std::string& feature = feat.node_desc;
            const std::string& feat_val = feat.node_info;

            ParamType type;
            status = TYParamGetType(hDevice, feature.c_str(), &type);
            if(status) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to check node type(" << feature << "), err: " << status);
                continue;
            }

            switch(type) {
                case Integer: {
                    int32_t val = atoi(feat_val.c_str());
                    status = TYIntegerSetValue(hDevice, feature.c_str(), val);
                    break;
                }
        
                case Float: {
                    double val = atoi(feat_val.c_str());
                    status = TYFloatSetValue(hDevice, feature.c_str(), val);
                    break;
                }

                case Boolean: {
                    bool val = static_cast<bool>(atof(feat_val.c_str()));
                    status = TYBooleanSetValue(hDevice, feature.c_str(), val);
                    break;
                }

                case Enumeration: {
                    int32_t val = atoi(feat_val.c_str());
                    status = TYEnumSetValue(hDevice, feature.c_str(), val);
                    break;
                }

                case String: {
                    status = TYStringSetValue(hDevice, feature.c_str(), feat_val.c_str());
                    break;
                }

                default:{
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Unsupported node type: " <<  source << "/" <<  feature);
                    break;
                }
            }

            if(status) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Xml parameter(" << source << ":" << feature << ") init failed(" << TYErrorString(status) << "(" << status << ")");
                continue;
            }
        }
    }
    return ;
}

TY_STATUS GigE_2_1::stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data)
{
    int64_t source = CamComponentIDToSourceIdx(comp);
    TY_STATUS status = source_init(source);
    if(TY_STATUS_OK == status) {
        int64_t intrinsicWidth = 0;
        int64_t intrinsicHeight = 0;
        TY_STATUS ret = TYIntegerGetValue(hDevice, "IntrinsicWidth", &intrinsicWidth);
        if(TY_STATUS_OK == ret) calib_data.intrinsicWidth = intrinsicWidth;

        ret = TYIntegerGetValue(hDevice, "IntrinsicHeight", &intrinsicHeight);
        if(TY_STATUS_OK == ret) calib_data.intrinsicHeight = intrinsicHeight;

        double intrinsic[9];
        ret = TYByteArrayGetValue(hDevice, "Intrinsic", reinterpret_cast<uint8_t*>(intrinsic), sizeof(intrinsic));
        if(TY_STATUS_OK == ret) {
            for(int32_t i = 0; i < 9; i++)
                calib_data.intrinsic.data[i] = static_cast<float>(intrinsic[i]);
        }

        double distortion[12];
        ret = TYByteArrayGetValue(hDevice, "Distortion", reinterpret_cast<uint8_t*>(distortion), sizeof(distortion));
        if(TY_STATUS_OK == ret) {
            for(int32_t i = 0; i < 12; i++) 
                calib_data.distortion.data[i] = static_cast<float>(distortion[i]);
        }

        double extrinsic[16];
        ret = TYByteArrayGetValue(hDevice, "Extrinsic", reinterpret_cast<uint8_t*>(extrinsic), sizeof(extrinsic));
        if(TY_STATUS_OK == ret) {
            for(int32_t i = 0; i < 16; i++)
                calib_data.extrinsic.data[i] = static_cast<float>(extrinsic[i]);
        }
    }

    return status;
}

TY_STATUS GigE_2_1::EnableHwIRUndistortion()
{
    bool exist = false;
    TY_STATUS ret = TYParamExist(hDevice, "IRUndistortion", &exist);
    if(ret) return ret;
    if(!exist) return TY_STATUS_NOT_PERMITTED;
    return TYBooleanSetValue(hDevice, "IRUndistortion", true);
}

TY_STATUS GigE_2_1::getIRLensType(TYLensOpticalType& type)
{
    type = TY_LENS_PINHOLE;
    return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::getIRRectificationMode(percipio_rectification_mode& mode)
{
    mode = DISTORTION_CORRECTION;

    TY_STATUS ret = source_init(m_Source_LeftIR);
    if(ret) return ret;

    bool rotation = false;
    bool rectified_intr = false;  
    TYParamExist(hDevice, "Rotation", &rotation);
    TYParamExist(hDevice, "Intrinsic2", &rectified_intr);
    if(rotation && rectified_intr) mode = EPIPOLAR_RECTIFICATION;
    return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::getLeftIRRotation(TY_CAMERA_ROTATION& Rotation)
{
    TY_STATUS ret = source_init(m_Source_LeftIR);
    if(ret) return ret;

    double rotation[9];
    ret = TYByteArrayGetValue(hDevice, "Rotation", reinterpret_cast<uint8_t*>(rotation), sizeof(rotation));
    if(TY_STATUS_OK == ret) {
        for(size_t i = 0; i < 9; i++) {
            Rotation.data[i] = static_cast<float>(rotation[i]);
        }
    }
    return ret;
}

TY_STATUS GigE_2_1::getLeftIRRectifiedIntr(TY_CAMERA_INTRINSIC& Rectified_intr)
{
    TY_STATUS ret = source_init(m_Source_LeftIR);
    if(ret) return ret;

    int64_t binning = 0;
    ret = TYEnumGetValue(hDevice, "BinningHorizontal", &binning);
    if(ret) binning = 1;

    double intrinsic[9];
    ret = TYByteArrayGetValue(hDevice, "Intrinsic2", reinterpret_cast<uint8_t*>(intrinsic), sizeof(intrinsic));
    if(TY_STATUS_OK == ret) {
        Rectified_intr.data[0] = static_cast<float>(intrinsic[0] / binning);
        Rectified_intr.data[1] = 0;
        Rectified_intr.data[2] = static_cast<float>(intrinsic[2] / binning);

        Rectified_intr.data[3] = 0;
        Rectified_intr.data[4] = static_cast<float>(intrinsic[4] / binning);
        Rectified_intr.data[5] = static_cast<float>(intrinsic[5] / binning);

        Rectified_intr.data[6] = 0;
        Rectified_intr.data[7] = 0;
        Rectified_intr.data[8] = 1;
    }
    return ret;
}

TY_STATUS GigE_2_1::getRightIRRotation(TY_CAMERA_ROTATION& Rotation)
{
    TY_STATUS ret = source_init(m_Source_RightIR);
    if(ret) return ret;

    double rotation[9];
    ret = TYByteArrayGetValue(hDevice, "Rotation", reinterpret_cast<uint8_t*>(rotation), sizeof(rotation));
    if(TY_STATUS_OK == ret) {
        for(size_t i = 0; i < 9; i++) {
            Rotation.data[i] = static_cast<float>(rotation[i]);
        }
    }
    return ret;
}

TY_STATUS GigE_2_1::getRightIRRectifiedIntr(TY_CAMERA_INTRINSIC& Rectified_intr)
{
    TY_STATUS ret = source_init(m_Source_RightIR);
    if(ret) return ret;

    int64_t binning = 0;
    ret = TYEnumGetValue(hDevice, "BinningHorizontal", &binning);
    if(ret) binning = 1;

    double intrinsic[9];
    ret = TYByteArrayGetValue(hDevice, "Intrinsic2", reinterpret_cast<uint8_t*>(intrinsic), sizeof(intrinsic));
    if(TY_STATUS_OK == ret) {
        Rectified_intr.data[0] = static_cast<float>(intrinsic[0] / binning);
        Rectified_intr.data[1] = 0;
        Rectified_intr.data[2] = static_cast<float>(intrinsic[2] / binning);

        Rectified_intr.data[3] = 0;
        Rectified_intr.data[4] = static_cast<float>(intrinsic[4] / binning);
        Rectified_intr.data[5] = static_cast<float>(intrinsic[5] / binning);

        Rectified_intr.data[6] = 0;
        Rectified_intr.data[7] = 0;
        Rectified_intr.data[8] = 1;
    }
    return ret;
}

void GigE_2_1::depth_stream_distortion_check(bool& has_undist_data)
{
    has_undist_data = false;
    TY_STATUS ret = source_init(CamComponentIDToSourceIdx(TY_COMPONENT_DEPTH_CAM));
    if(ret) {
        has_undist_data = false;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "TYEnumSetValue set SourceSelector to depth failed: " << ret);
        return;
    }

    TY_ACCESS_MODE access = 0;
    ret = TYParamGetAccess(hDevice, "Distortion", &access);
    if(ret) {
        has_undist_data = false;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "TYParamGetAccess get Distortion failed: " << ret);
        return;
    }

    if(access & TY_ACCESS_READABLE) {
        has_undist_data = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Range  distortion is readable!");
    } else {
        has_undist_data = false;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Range distortion is not readable!");
    }
}

TY_STATUS GigE_2_1::depth_scale_unit_init(float& dept_scale_unit)
{
    double scale = 1.f;
    TY_STATUS status = TYFloatGetValue(hDevice, "DepthScaleUnit", &scale);
    dept_scale_unit = (float)scale;
    return status;
}

TY_STATUS GigE_2_1::color_stream_aec_roi_init(const TY_AEC_ROI_PARAM& ROI)
{
    auto source = CamComponentIDToSourceIdx(TY_COMPONENT_RGB_CAM);
    if(source < 0) return TY_STATUS_INVALID_PARAMETER;

    TY_STATUS status = source_init(source);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "SourceSelector init failed: " << status);
        return status;
    }

    TY_ACCESS_MODE access;
    status = TYParamGetAccess(hDevice, "ExposureAuto", &access);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Get ExposureAuto access failed: " << status);
        return status;
    }
    
    if(!(access & TY_ACCESS_WRITABLE)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "RGB camera does not support AEC!");
        return TY_STATUS_NOT_PERMITTED;
    }

    status = TYBooleanSetValue(hDevice, "ExposureAuto", true);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Enable color aec failed: " << status);
        return status;
    }

    status = TYIntegerSetValue(hDevice, "AutoFunctionAOIOffsetX", ROI.x);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "AutoFunctionAOIOffsetX write failed: " << status);
        return status;
    }

    status = TYIntegerSetValue(hDevice, "AutoFunctionAOIOffsetY", ROI.y);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "AutoFunctionAOIOffsetY write failed: " << status);
        return status;
    }

    status = TYIntegerSetValue(hDevice, "AutoFunctionAOIWidth", ROI.w);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "AutoFunctionAOIWidth write failed: " << status);
        return status;
    }

    status = TYIntegerSetValue(hDevice, "AutoFunctionAOIHeight", ROI.h);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "AutoFunctionAOIHeight write failed: " << status);
        return status;
    }
    
    return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::send_soft_trigger_signal()
{
    return TYCommandExec(hDevice, "TriggerSoftware");
}

void GigE_2_1::reset()
{
    TYCommandExec(hDevice, "DeviceReset");
}

TY_STATUS GigE_2_1::source_init(int64_t source)
{
  int64_t m_current_source;
  TY_STATUS status = TYEnumGetValue(hDevice, "SourceSelector", &m_current_source);
  if(status) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "SourceSelector read failed: " << status);
      return status;
  }

  if(source == m_current_source) return TY_STATUS_OK;
  return TYEnumSetValue(hDevice, "SourceSelector", source);
}

TY_STATUS GigE_2_1::fix_device_frame_rate(float& rate)
{
    double min = 0, max = 0;
    double target_fps = static_cast<double>(rate);
    TY_STATUS status = TYBooleanSetValue(hDevice, "AcquisitionFrameRateEnable", true);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to set AcquisitionFrameRateEnable to true, error: " << status);
        return status;
    }

    status = TYFloatGetMin(hDevice, "AcquisitionFrameRate", &min);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to get minimum frame rate from device. error: " << status);
        min = 1.0;
    }
    
    status = TYFloatGetMax(hDevice, "AcquisitionFrameRate", &max);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to get maximum frame rate from device. error: " << status);
        max = 30.0;
    }
    
    if(target_fps < min) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Requested frame rate " << target_fps << " is below minimum " << min << ". Clamping to minimum.");
        target_fps = min;
    } else if(target_fps > max) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Requested frame rate " << target_fps << " exceeds maximum " << max << ". Clamping to maximum.");
        target_fps = max;
    }

    status = TYFloatSetValue(hDevice, "AcquisitionFrameRate", target_fps);
    if(status) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to set frame rate to " << target_fps << " FPS. error:" << status);
    } else {
        rate = static_cast<float>(rate);
    }

    return status;
}

TY_STATUS GigE_2_1::fix_device_frame_rate_in_soft_trigger_mode(const float rate)
{
    TY_STATUS status = TYEnumSetString(hDevice, "AcquisitionMode", "SingleFrame");
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to set AcquisitionMode to SingleFrame mode, error: " << status);
        return status;
    }

    status = TYEnumSetString(hDevice, "TriggerSource", "Software");
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Failed to set TriggerSource to  Software, error: " << status);
        return status;
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Try using the software trigger mode to achieve fixed frame rate triggered output.");
    soft_frame_rate_ctrl_enable = true;
    soft_frame_rate = rate;

    return TY_STATUS_OK;
}

}