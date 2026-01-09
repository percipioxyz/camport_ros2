#include "gige_2_0.h"
#include "Utils.hpp"
#include "crc32.hpp"
#include "ParametersParse.hpp"
#include "huffman.h"

namespace percipio_camera {

#define LOG_HEAD_GIGE_2_0  "GIGE2_0"

#define MAX_STORAGE_SIZE    (10*1024*1024)

static std::map<std::string, TY_FEATURE_ID> m_gige_2_0_feature_map = {
    {"TriggerDelay",                              TY_INT_TRIGGER_DELAY_US},

    {"ExposureAuto",                              TY_BOOL_AUTO_EXPOSURE},
    {"ExposureTime",                              TY_INT_EXPOSURE_TIME},
    {"ExposureTargetBrightness",                  TY_INT_AE_TARGET_Y},
    {"AnalogAll",                                 TY_INT_ANALOG_GAIN},
    {"DigitalAll",                                TY_INT_GAIN},
    {"DigitalRed",                                TY_INT_R_GAIN},
    {"DigitalGreen",                              TY_INT_G_GAIN},
    {"DigitalBlue",                               TY_INT_B_GAIN},
    {"BalanceWhiteAuto",                          TY_BOOL_AUTO_AWB},

    {"AutoFunctionAOIOffsetX",                    TY_STRUCT_AEC_ROI},
    {"AutoFunctionAOIOffsetY",                    TY_STRUCT_AEC_ROI},
    {"AutoFunctionAOIWidth",                      TY_STRUCT_AEC_ROI},
    {"AutoFunctionAOIHeight",                     TY_STRUCT_AEC_ROI},

    {"DepthScaleUnit",                            TY_FLOAT_SCALE_UNIT},
    {"DepthSgbmImageNumber",                      TY_INT_SGBM_IMAGE_NUM},
    {"DepthSgbmDisparityNumber",                  TY_INT_SGBM_DISPARITY_NUM},
    {"DepthSgbmDisparityOffset",                  TY_INT_SGBM_DISPARITY_OFFSET},
    {"DepthSgbmMatchWinHeight",                   TY_INT_SGBM_MATCH_WIN_HEIGHT},
    {"DepthSgbmSemiParamP1",                      TY_INT_SGBM_SEMI_PARAM_P1},
    {"DepthSgbmSemiParamP2",                      TY_INT_SGBM_SEMI_PARAM_P2},
    {"DepthSgbmUniqueFactor",                     TY_INT_SGBM_UNIQUE_FACTOR},
    {"DepthSgbmUniqueAbsDiff",                    TY_INT_SGBM_UNIQUE_ABSDIFF},
    {"DepthSgbmUniqueMaxCost",                    TY_INT_SGBM_UNIQUE_MAX_COST},
    {"DepthSgbmHFilterHalfWin",                   TY_BOOL_SGBM_HFILTER_HALF_WIN},
    {"DepthSgbmMatchWinWidth",                    TY_INT_SGBM_MATCH_WIN_WIDTH},
    {"DepthSgbmMedFilter",                        TY_BOOL_SGBM_MEDFILTER},
    {"DepthSgbmLRC",                              TY_BOOL_SGBM_LRC},
    {"DepthSgbmLRCDiff",                          TY_INT_SGBM_LRC_DIFF},
    {"DepthSgbmMedFilterThresh",                  TY_INT_SGBM_MEDFILTER_THRESH},
    {"DepthSgbmSemiParamP1Scale",                 TY_INT_SGBM_SEMI_PARAM_P1_SCALE},
    {"DepthSgpmPhaseNumber",                      TY_INT_SGPM_PHASE_NUM},
    {"DepthSgpmPhaseScale",                       TY_INT_SGPM_NORMAL_PHASE_SCALE},
    {"DepthSgpmPhaseOffset",                      TY_INT_SGPM_NORMAL_PHASE_OFFSET},
    {"DepthSgpmReferencePhaseScale",              TY_INT_SGPM_REF_PHASE_SCALE},
    {"DepthSgpmReferencePhaseOffset",             TY_INT_SGPM_REF_PHASE_OFFSET},
    {"DepthSgpmEpipolarConstraintPatternScale",   TY_INT_SGPM_REF_PHASE_SCALE},
    {"DepthSgpmEpipolarConstraintPatternOffset",  TY_INT_SGPM_REF_PHASE_OFFSET},
    {"DepthSgpmEpipolarConstraintEnable",         TY_BOOL_SGPM_EPI_EN},
    {"DepthSgpmEpipolarConstraintChan0",          TY_INT_SGPM_EPI_CH0},
    {"DepthSgpmEpipolarConstraintChan1",          TY_INT_SGPM_EPI_CH1},
    {"DepthSgpmEpipolarConstraintThresh",         TY_INT_SGPM_EPI_THRESH},
    {"DepthSgpmPhaseOrderFilterEnable",           TY_BOOL_SGPM_ORDER_FILTER_EN},
    {"DepthSgpmPhaseOrderFilterChannel",          TY_INT_SGPM_ORDER_FILTER_CHN},
    {"DepthRangeMin",                             TY_INT_DEPTH_MIN_MM},
    {"DepthRangeMax",                             TY_INT_DEPTH_MAX_MM},
    {"DepthSgbmTextureFilterValueOffset",         TY_INT_SGBM_TEXTURE_OFFSET},
    {"DepthSgbmTextureFilterThreshold",           TY_INT_SGBM_TEXTURE_THRESH},

    {"DepthStreamTofFilterThreshold",             TY_INT_FILTER_THRESHOLD},
    {"DepthStreamTofChannel",                     TY_INT_TOF_CHANNEL},
    {"DepthStreamTofModulationThreshold",         TY_INT_TOF_MODULATION_THRESHOLD},
    {"DepthStreamTofDepthQuality",                TY_ENUM_DEPTH_QUALITY},
    {"DepthStreamTofHdrRatio",                    TY_INT_TOF_HDR_RATIO},
    {"DepthStreamTofJitterThreshold",             TY_INT_TOF_JITTER_THRESHOLD},

    {"DepthStreamTofAntiSunlightIndex",           TY_INT_TOF_ANTI_SUNLIGHT_INDEX},
    {"DepthStreamTofAntiInterference",            TY_BOOL_TOF_ANTI_INTERFERENCE},
    {"DepthStreamTofSpeckleSize",                 TY_INT_MAX_SPECKLE_SIZE},
    {"DepthStreamTofSpeckleDiff",                 TY_INT_MAX_SPECKLE_DIFF}
};

static bool isValidJsonString(const char* code)
{
    std::string err;
    const auto json = Json::parse(code, err);
    if(json.is_null()) return false;
    return true;
}

GigE_2_0::GigE_2_0(const TY_DEV_HANDLE dev) : GigEBase(dev)
{
    TYGetComponentIDs(hDevice, &allComps);
}


TY_STATUS GigE_2_0::init()
{
    TY_STATUS status = TYSetEnum(hDevice, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_HOST);
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Set time sync type(host) error: " << status);
    }
    load_default_parameter();
    
    video_mode_init();
    return TY_STATUS_OK;
}

TY_STATUS GigE_2_0::dump_image_mode_list(const TY_COMPONENT_ID comp, std::vector<percipio_video_mode>& modes)
{
    std::vector<TY_ENUM_ENTRY> entrys;
    TY_STATUS ret = get_feature_enum_list(hDevice, comp, TY_ENUM_IMAGE_MODE, entrys);
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

TY_STATUS GigE_2_0::image_mode_cfg(const TY_COMPONENT_ID comp, const percipio_video_mode& mode)
{
    TY_IMAGE_MODE image_enum_mode = TYImageMode2(mode.fmt, mode.width, mode.height);
    return TYSetEnum(hDevice, comp, TY_ENUM_IMAGE_MODE, image_enum_mode);
}

TY_STATUS GigE_2_0::work_mode_init(percipio_dev_workmode mode, const bool fix_rate, const float rate)
{
    TY_STATUS status = TY_STATUS_OK;
    TY_TRIGGER_PARAM_EX trigger;

    bool has_resend = false;
    status = TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, &has_resend);
    if(status) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Stream Resend ignore" << status);
    } else {
        if(has_resend) {
            status = TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_GVSP_RESEND, true);
            if(status) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Failed to enable stream resend, error: " << status);
            }
        }
    }

    soft_frame_rate_ctrl_enable = false;
    if(mode == CONTINUS) {
        if(fix_rate) {
            //try use m per mode
            trigger.mode = TY_TRIGGER_MODE_M_PER;
            trigger.fps = (int8_t)rate;
            status = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
            if(status != TY_STATUS_OK) {
                trigger.mode = TY_TRIGGER_MODE_SLAVE;
                status = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
                if(status != TY_STATUS_OK) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "The device does not support the automatic fixed frame rate output mode.");
                    return status;
                } else {
                    soft_frame_rate_ctrl_enable = true;
                    soft_frame_rate = rate;
                    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Try using the software trigger mode to achieve fixed frame rate triggered output.");
                }
            }
        } else {
            //Clear trigger mdoe status
            trigger.mode = TY_TRIGGER_MODE_OFF;
            status = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
            if(status != TY_STATUS_OK) { 
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Failed to set continue mode, error: " << status);
                return status;
            }
        }
    } else {
        trigger.mode = TY_TRIGGER_MODE_SLAVE;
        status = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &trigger, sizeof(trigger));
        if(status != TY_STATUS_OK) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Failed to set trigger mode, error: " << status);
            return status;
        }
    }

    return TY_STATUS_OK;
}

void GigE_2_0::device_load_parameters()
{
    for(auto& iter : parameters) {
        for(auto& feat : iter.second) {
            parameter_init(iter.first, feat.node_desc, feat.node_info);
        }
    }

    return;
}

TY_STATUS GigE_2_0::stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data)
{
    return TYGetStruct(hDevice, comp, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(calib_data));
}

void GigE_2_0::depth_stream_distortion_check(bool& has_undist_data)
{
    has_undist_data = false;
    TY_STATUS ret = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_DISTORTION, &has_undist_data);
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Depth distortion calib data check ret: " << ret << "(" << has_undist_data << ")");
}

TY_STATUS GigE_2_0::depth_scale_unit_init(float& dept_scale_unit)
{
    return TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &dept_scale_unit);
}

TY_STATUS GigE_2_0::color_stream_aec_roi_init(const TY_AEC_ROI_PARAM& ROI)
{
    bool has = false;
    TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, &has);
    if(!has) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "RGB camera does not support AEC!");
        return TY_STATUS_OK;
    }

    TY_STATUS status = TYSetBool(hDevice, TY_COMPONENT_RGB_CAM, TY_BOOL_AUTO_EXPOSURE, true);
    if(status != TY_STATUS_OK) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Enable color aec failed : " << status);
        return status;
    } else {
        has = false;
        TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_AEC_ROI, &has);
        if(!has) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "The RGB camera does not support AEC ROI metering!");
            return TY_STATUS_OK;
        }

        status = TYSetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_AEC_ROI, (void*)&ROI, sizeof(ROI));
        if(status != TY_STATUS_OK) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Set color aec roi failed : " << status);
        } else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Set color aec roi: " << ROI.x << ", " << ROI.y << ", " << ROI.w << ", " << ROI.h);
        }
    }
    
    return status;
}

TY_STATUS GigE_2_0::send_soft_trigger_signal()
{
    int err = TY_STATUS_OK;
    while(TY_STATUS_BUSY == (err = TYSendSoftTrigger(hDevice)));
    return err;
}

bool GigE_2_0::load_default_parameter()
{
    TY_STATUS status = TY_STATUS_OK;
    uint32_t block_size;
    uint8_t* blocks = new uint8_t[MAX_STORAGE_SIZE] ();
    status = TYGetByteArraySize(hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, &block_size);
    if(status != TY_STATUS_OK) {
        delete []blocks;
        return false;
    } 
    
    status = TYGetByteArray(hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_CUSTOM_BLOCK, blocks,  block_size);
    if(status != TY_STATUS_OK) {
        delete []blocks;
        return false;
    }
    
    uint32_t crc_data = *(uint32_t*)blocks;
    if(0 == crc_data || 0xffffffff == crc_data) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "The CRC check code is empty.");
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
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Storage data length error.");
                    delete []blocks;
                    return false;
                }
                
                crc = crc32_bitwise(huffman_ptr, huffman_size);
                if(crc_data != crc) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Storage area data check failed (check code error).");
                    delete []blocks;
                    return false;
                }

                std::string huffman_string(huffman_ptr, huffman_ptr + huffman_size);
                if(!TextHuffmanDecompression(huffman_string, js_string)) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Huffman decompression error.");
                    delete []blocks;
                    return false;
                }
                break;
            }
            default:
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Unsupported encoding format.");
                delete []blocks;
                return false;
            }
        }
    } else {
        js_string = std::string((const char*)js_code);
    }

    if(!isValidJsonString(js_string.c_str())) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Incorrect json data.");
        delete []blocks;
        return false;
    }

    bool ret =json_parse(hDevice, js_string.c_str());
    if(ret)  
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Loading default parameters successfully!");
    else
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Failed to load default parameters, some parameters cannot be loaded properly!");

    delete []blocks;
    return ret;
}

bool GigE_2_0::parameter_init(const std::string& source, const std::string& feat, const std::string& str_val)
{
    TY_STATUS status = TY_STATUS_OK;
    TY_COMPONENT_ID comp = 0;
    if(source == "Depth")
        comp = TY_COMPONENT_DEPTH_CAM;
    else if(source == "Texture")
        comp = TY_COMPONENT_RGB_CAM;
    else if(source == "Left")
        comp = TY_COMPONENT_IR_CAM_LEFT;
    else if(source == "Right")
        comp = TY_COMPONENT_IR_CAM_RIGHT;
    else if(source == "Device")
        comp = TY_COMPONENT_DEVICE;
    else if(source == "Laser")
        comp = TY_COMPONENT_LASER;
    else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Incorrect source name: " << source);
        return false;
    }

    if(!(allComps & comp)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Unsupported source name: " << source);
        return false;
    }

    auto feature = m_gige_2_0_feature_map.find(feat);
    if (feature == m_gige_2_0_feature_map.end()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Unsupported feature name: " << feat);
        return false;
    }

    TY_FEATURE_ID feat_id = feature->second;
    TY_FEATURE_TYPE type = TYFeatureType(feat_id);
    switch(type) {
        case TY_FEATURE_INT: {
            int32_t val = atoi(str_val.c_str());
            status = TYSetInt(hDevice, comp, feat_id, val);
            break;
        }

        case TY_FEATURE_ENUM: {
            uint32_t val = static_cast<uint32_t>(atoi(str_val.c_str()));
            status = TYSetEnum(hDevice, comp, feat_id, val);
            break;
        }

        case TY_FEATURE_FLOAT: {
            float val = static_cast<float>(atof(str_val.c_str()));
            status = TYSetFloat(hDevice, comp, feat_id, val);
            break;
        }

        case TY_FEATURE_BOOL: {
            bool val = static_cast<bool>(atof(str_val.c_str()));
            status = TYSetBool(hDevice, comp, feat_id, val);
            break;
        }
    
        case TY_FEATURE_STRING: {
            status = TYSetString(hDevice, comp, feat_id, str_val.c_str());
            break;
        }

        case TY_FEATURE_STRUCT: {
            if(feat_id == TY_STRUCT_AEC_ROI) {
                if(feat == "AutoFunctionAOIOffsetX") {
                    aec_roi.x = static_cast<uint32_t>(atoi(str_val.c_str()));
                } else if(feat == "AutoFunctionAOIOffsetY") {
                    aec_roi.y = static_cast<uint32_t>(atoi(str_val.c_str()));
                } else if(feat == "AutoFunctionAOIWidth") {
                    aec_roi.w = static_cast<uint32_t>(atoi(str_val.c_str()));
                } else if(feat == "AutoFunctionAOIHeight") {
                    aec_roi.h = static_cast<uint32_t>(atoi(str_val.c_str()));
                    status = TYSetStruct(hDevice, comp, feat_id, &aec_roi, sizeof(aec_roi));
                } else {
                    status = TY_STATUS_INVALID_FEATURE;
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Unsupported feature type: " << feat << "(" << feat_id << ")");
                }
            } else {
                status = TY_STATUS_INVALID_FEATURE;
                RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Unsupported feature type: " << feat << "(" << feat_id << ")");
            }
            break;
        }
    
        default: {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Unsupported feature type: " << feat << "(" << feat_id << ")");
            return false;
        }
    }

    if(status) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Xml parameter(" << source << ":" << feat << "(0x" << std::hex << feat_id << ")" << ") init failed(" << TYErrorString(status) << "(" << std::dec << status << ")");
        return false;
    }

    return true;
}
}