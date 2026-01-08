#include "gige_2_1.h"

namespace percipio_camera {

#define LOG_HEAD_GIGE_2_1  "GIGE2_1"

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


GigE_2_1::GigE_2_1(const TY_DEV_HANDLE dev) : GigEBase(dev)
{
    TYGetComponentIDs(hDevice, &allComps);
}

TY_STATUS GigE_2_1::init()
{
    TY_STATUS status = TYEnumSetString(hDevice, "DeviceTimeSyncMode", "SyncTypeHost");
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_1), "Set time sync type(host) err : " << status);
    }

    video_mode_init();
    return TY_STATUS_OK;
}

TY_STATUS GigE_2_1::dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes)
{
    int source = CamComponentIDToSourceIdx(comp);
    if(source < 0) return TY_STATUS_INVALID_COMPONENT;

    TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", source);
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
    int source = CamComponentIDToSourceIdx(comp);
    if(source < 0) return TY_STATUS_INVALID_COMPONENT;

    TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", source);
    if(ret) return ret;

    ret = TYEnumSetValue(hDevice, "PixelFormat", mode.fmt);
    if(ret) return ret;

    return TYEnumSetValue(hDevice, "BinningHorizontal", mode.binning);
}

TY_STATUS GigE_2_1::stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data)
{
    int source = CamComponentIDToSourceIdx(comp);
    TY_STATUS status = TYEnumSetValue(hDevice, "SourceSelector", source);
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

void GigE_2_1::depth_stream_distortion_check(bool& has_undist_data)
{
    has_undist_data = false;
    TY_STATUS ret = TYEnumSetValue(hDevice, "SourceSelector", CamComponentIDToSourceIdx(TY_COMPONENT_DEPTH_CAM));
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

    TY_STATUS status = TYEnumSetValue(hDevice, "SourceSelector", source);
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

TY_STATUS GigE_2_1::set_tof_depth_quality(const std::string& qua)
{
    return TY_STATUS_NOT_PERMITTED;
}

TY_STATUS GigE_2_1::set_tof_modulation_threshold(int threshold)
{
    return TY_STATUS_NOT_PERMITTED;
}

TY_STATUS GigE_2_1::set_tof_jitter_threshold(int threshold)
{
    return TY_STATUS_NOT_PERMITTED;
}

TY_STATUS GigE_2_1::set_tof_filter_threshold(int threshold)
{
    return TY_STATUS_NOT_PERMITTED;
}

TY_STATUS GigE_2_1::set_tof_channel(int chan)
{
    return TY_STATUS_NOT_PERMITTED;
}

TY_STATUS GigE_2_1::set_tof_HDR_ratio(int ratio)
{
    return TY_STATUS_NOT_PERMITTED;
}

}