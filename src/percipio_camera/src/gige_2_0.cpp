#include "gige_2_0.h"
#include "Utils.hpp"
#include "crc32.hpp"
#include "ParametersParse.hpp"
#include "huffman.h"

namespace percipio_camera {

#define LOG_HEAD_GIGE_2_0  "GIGE2_0"

#define MAX_STORAGE_SIZE    (10*1024*1024)

static bool isValidJsonString(const char* code)
{
    std::string err;
    const auto json = Json::parse(code, err);
    if(json.is_null()) return false;
    return true;
}

static uint32_t depth_qua_desc_to_enum(const std::string& qua)
{
    if(qua == "basic") return TY_DEPTH_QUALITY_BASIC;
    else if(qua == "medium") return TY_DEPTH_QUALITY_MEDIUM;
    else if(qua == "high") return TY_DEPTH_QUALITY_HIGH;
    else return TY_DEPTH_QUALITY_MEDIUM;
}

GigE_2_0::GigE_2_0(const TY_DEV_HANDLE dev) : GigEBase(dev)
{
    TYGetComponentIDs(hDevice, &allComps);
}


TY_STATUS GigE_2_0::init()
{
    TY_STATUS status = TYSetEnum(hDevice, TY_COMPONENT_DEVICE, TY_ENUM_TIME_SYNC_TYPE, TY_TIME_SYNC_TYPE_HOST);
    if(status != TY_STATUS_OK) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Set time sync type(host) err : " << status);
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

TY_STATUS GigE_2_0::stream_calib_data_init(const TY_COMPONENT_ID comp, TY_CAMERA_CALIB_INFO& calib_data)
{
    return TYGetStruct(hDevice, comp, TY_STRUCT_CAM_CALIB_DATA, &calib_data, sizeof(calib_data));
}

void GigE_2_0::depth_stream_distortion_check(bool& has_undist_data)
{
    has_undist_data = false;
    TY_STATUS ret = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_DISTORTION, &has_undist_data);
    RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_GIGE_2_0), "Depth distortion calib data check ret:" << ret << "(" << has_undist_data << ")");
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

TY_STATUS GigE_2_0::set_tof_depth_quality(const std::string& qua)
{
    bool has = false;
    TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, &has);
    if(status != TY_STATUS_OK) return status;
    if(!has) return TY_STATUS_NOT_PERMITTED;

    uint32_t m_qua = depth_qua_desc_to_enum(qua);
    return TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_DEPTH_QUALITY, m_qua);
}

TY_STATUS GigE_2_0::set_tof_modulation_threshold(int threshold)
{
    bool has = false;
    TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_MODULATION_THRESHOLD, &has);
    if(status != TY_STATUS_OK) return status;
    if(!has) return TY_STATUS_NOT_PERMITTED;
    return TYSetInt(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_MODULATION_THRESHOLD, threshold);
}

TY_STATUS GigE_2_0::set_tof_jitter_threshold(int threshold)
{
    bool has = false;
    TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_JITTER_THRESHOLD, &has);
    if(status != TY_STATUS_OK) return status;
    if(!has) return TY_STATUS_NOT_PERMITTED;
    return TYSetInt(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_JITTER_THRESHOLD, threshold);
}

TY_STATUS GigE_2_0::set_tof_filter_threshold(int threshold)
{
    bool has = false;
    TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_FILTER_THRESHOLD, &has);
    if(status != TY_STATUS_OK) return status;
    if(!has) return TY_STATUS_NOT_PERMITTED;
    return TYSetInt(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_FILTER_THRESHOLD, threshold);
}

TY_STATUS GigE_2_0::set_tof_channel(int chan)
{
    bool has = false;
    TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_CHANNEL, &has);
    if(status != TY_STATUS_OK) return status;
    if(!has) return TY_STATUS_NOT_PERMITTED;
    return TYSetInt(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_CHANNEL, chan);
}

TY_STATUS GigE_2_0::set_tof_HDR_ratio(int ratio)
{
    bool has = false;
    TY_STATUS status = TYHasFeature(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_HDR_RATIO, &has);
    if(status != TY_STATUS_OK) return status;
    if(!has) return TY_STATUS_NOT_PERMITTED;
    return TYSetInt(hDevice, TY_COMPONENT_DEPTH_CAM, TY_INT_TOF_HDR_RATIO, ratio);
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

}