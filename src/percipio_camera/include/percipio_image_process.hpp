#pragma once
#include <vector>
#include "percipio_video_mode.h"
#include "TYParameter.h"

#define PERCIPIO_IMAGE_PROCESS_LOG    "percipio_image_process"

static int GrayIR_linearStretch(percipio_camera::TYImage& grayIR)
{
    TYPixFmt pixelFormat = grayIR.format();
    if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "LinearStretch support Mono8 or Mono16 gray, not support others type, please check grayIR pixel format");
        return -1;
    }
    int rows = grayIR.height();
    int cols = grayIR.width();
    
    double ratiocut = 0.1;
    int roi_x = static_cast<int>(cols * ratiocut);
    int roi_y = static_cast<int>(rows * ratiocut);
    int roi_width = static_cast<int>(cols - cols * ratiocut * 2);
    int roi_height = static_cast<int>(rows - rows * ratiocut * 2);
    if (roi_width <= 0 || roi_height <= 0 || roi_x < 0 || roi_y < 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "ROI is invalid");
        return -1;
    }
    void* data = grayIR.data();
    if (!data) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "grayIR data is null");
        return -1;
    }
    double minVal = 0.0, maxVal = 0.0;
    
    if (pixelFormat == TYPixelFormatMono8) {
        uint8_t* buffer = static_cast<uint8_t*>(data);
        minVal = 255.0;
        maxVal = 0.0;
        
        for (int y = roi_y; y < roi_y + roi_height; ++y) {
            for (int x = roi_x; x < roi_x + roi_width; ++x) {
                uint8_t val = buffer[y * cols + x];
                if (val < minVal) minVal = val;
                if (val > maxVal) maxVal = val;
            }
        }
    } else if (pixelFormat == TYPixelFormatMono16) {
        uint16_t* buffer = static_cast<uint16_t*>(data);
        minVal = 65535.0;
        maxVal = 0.0;
        
        for (int y = roi_y; y < roi_y + roi_height; ++y) {
            for (int x = roi_x; x < roi_x + roi_width; ++x) {
                uint16_t val = buffer[y * cols + x];
                if (val < minVal) minVal = val;
                if (val > maxVal) maxVal = val;
            }
        }
    }
    if (maxVal - minVal <= 0.0) {
        return 0;
    }
    static std::vector<uint8_t> result_buffer;
    if(result_buffer.size() != (size_t)(rows * cols)){
        result_buffer.resize(rows * cols);
    }
    double scale = 255.0 / (maxVal - minVal);
    double offset = -minVal * scale;
    
    if (pixelFormat == TYPixelFormatMono8) {
        uint8_t* buffer = static_cast<uint8_t*>(data);
        for (int i = 0; i < rows * cols; ++i) {
            double stretched = buffer[i] * scale + offset;
            if (stretched < 0.0) stretched = 0.0;
            if (stretched > 255.0) stretched = 255.0;
            result_buffer[i] = static_cast<uint8_t>(stretched);
        }
    } else if (pixelFormat == TYPixelFormatMono16) {
        uint16_t* buffer = static_cast<uint16_t*>(data);
        for (int i = 0; i < rows * cols; ++i) {
            double stretched = buffer[i] * scale + offset;
            if (stretched < 0.0) stretched = 0.0;
            if (stretched > 255.0) stretched = 255.0;
            result_buffer[i] = static_cast<uint8_t>(stretched);
        }
    }
    grayIR.resize(cols, rows);
    memcpy(grayIR.data(), result_buffer.data(), rows * cols * sizeof(uint8_t));
    grayIR.setPixelFormat(TYPixelFormatMono8);
    grayIR.setWidth(cols);
    grayIR.setHeight(rows);
    
    return 0;
}
static int GrayIR_linearStretch_multi(percipio_camera::TYImage& grayIR, double multi_expandratio = 8)
{
    if (multi_expandratio <= 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "LinearStretch_multi multi_expandratio must bigger than 0");
        return -1;
    }
    
    uint32_t pixelFormat = grayIR.format();
    if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "LinearStretch_multi support Mono8 or Mono16 gray, not support others type, please check grayIR pixel format");
        return -1;
    }
    
    int rows = grayIR.height();
    int cols = grayIR.width();
    size_t totalPixels = rows * cols;
    
    void* data = grayIR.data();
    if (!data) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "grayIR data is null");
        return -1;
    }
    
    static std::vector<uint8_t> result_buffer;
    if(result_buffer.size() != totalPixels) {
        result_buffer.resize(totalPixels);
    }
    
    double eight_bit_factor = multi_expandratio;
    double sixteen_bit_factor = multi_expandratio / 255.0;
    
    if (pixelFormat == TYPixelFormatMono8) {
        uint8_t* buffer = static_cast<uint8_t*>(data);
        
        if (multi_expandratio == 1.0) {
            memcpy(&result_buffer[0], buffer, totalPixels);
        } else {
            for (size_t i = 0; i < totalPixels; ++i) {
                double stretched = static_cast<double>(buffer[i]) * eight_bit_factor;
                
                int32_t rounded = static_cast<int32_t>(stretched + 0.5);
                if (rounded < 0) rounded = 0;
                if (rounded > 255) rounded = 255;
                
                result_buffer[i] = static_cast<uint8_t>(rounded);
            }
        }
    } 
    else if (pixelFormat == TYPixelFormatMono16) {
        uint16_t* buffer = static_cast<uint16_t*>(data);
        if (multi_expandratio == 1.0) {
            for (size_t i = 0; i < totalPixels; ++i) {
                result_buffer[i] = static_cast<uint8_t>(buffer[i] >> 8); //等价于 buffer[i] / 256
            }
        } else {
            for (size_t i = 0; i < totalPixels; ++i) {
                double stretched = static_cast<double>(buffer[i]) * sixteen_bit_factor;
                
                int32_t rounded = static_cast<int32_t>(stretched + 0.5);
                if (rounded < 0) rounded = 0;
                if (rounded > 255) rounded = 255;
                
                result_buffer[i] = static_cast<uint8_t>(rounded);
            }
        }
    }
    
    grayIR.resize(cols, rows);
    memcpy(grayIR.data(), result_buffer.data(), totalPixels * sizeof(uint8_t));
    grayIR.setPixelFormat(TYPixelFormatMono8);
    grayIR.setWidth(cols);
    grayIR.setHeight(rows);
    
    return 0;
}
static int GrayIR_linearStretch_std(percipio_camera::TYImage& grayIR, double std_expandratio = 6)
{
    if (std_expandratio <= 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "GrayIR_linearStretch_std multi_expandratio must be bigger than 0");
        return -1;
    }
    //Check pixel format
    uint32_t pixelFormat = grayIR.format();
    if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "GrayIR_linearStretch_std supports Mono8 or Mono16 gray, not other types");
        return -1;
    }
    int rows = grayIR.height();
    int cols = grayIR.width();
    size_t totalPixels = rows * cols;
    
    void* data = grayIR.data();
    if (!data) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "grayIR data is null");
        return -1;
    }
    //Calculate mean and standard deviation
    double sum = 0.0;
    double sumSquares = 0.0;
    
    if (pixelFormat == TYPixelFormatMono8) {
        uint8_t* buffer = static_cast<uint8_t*>(data);
        for (size_t i = 0; i < totalPixels; ++i) {
            double val = static_cast<double>(buffer[i]);
            sum += val;
            sumSquares += val * val;
        }
    } else { //TYPixelFormatMono16
        uint16_t* buffer = static_cast<uint16_t*>(data);
        for (size_t i = 0; i < totalPixels; ++i) {
            double val = static_cast<double>(buffer[i]);
            sum += val;
            sumSquares += val * val;
        }
    }
    
    double mean = sum / totalPixels;
    double variance = (sumSquares / totalPixels) - (mean * mean);
    double stddev = (variance > 0) ? sqrt(variance) : 0.0;
    
    //Calculate normalization factor
    double use_norm = stddev * std_expandratio + 1.0;
    if (use_norm <= 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "Calculated normalization factor is non-positive");
        return -1;
    }
    
    double scale = 255.0 / use_norm;
    
    //Find min and max for verification
    double minVal, maxVal;
    if (pixelFormat == TYPixelFormatMono8) {
        uint8_t* buffer = static_cast<uint8_t*>(data);
        minVal = 255.0;
        maxVal = 0.0;
        for (size_t i = 0; i < totalPixels; ++i) {
            double val = buffer[i];
            if (val < minVal) minVal = val;
            if (val > maxVal) maxVal = val;
        }
    } else { //TYPixelFormatMono16
        uint16_t* buffer = static_cast<uint16_t*>(data);
        minVal = 65535.0;
        maxVal = 0.0;
        for (size_t i = 0; i < totalPixels; ++i) {
            double val = buffer[i];
            if (val < minVal) minVal = val;
            if (val > maxVal) maxVal = val;
        }
    }
    
    //Apply linear stretch and convert to 8-bit
    static std::vector<uint8_t> result_buffer;
    if(result_buffer.size() != totalPixels) {
        result_buffer.resize(totalPixels);
    }
    if (pixelFormat == TYPixelFormatMono8) {
        uint8_t* buffer = static_cast<uint8_t*>(data);
        for (size_t i = 0; i < totalPixels; ++i) {
            double stretched = static_cast<double>(buffer[i]) * scale;
            if (stretched < 0.0) stretched = 0.0;
            if (stretched > 255.0) stretched = 255.0;
            result_buffer[i] = static_cast<uint8_t>(stretched);
        }
    } else { //TYPixelFormatMono16
        uint16_t* buffer = static_cast<uint16_t*>(data);
        for (size_t i = 0; i < totalPixels; ++i) {
            double stretched = static_cast<double>(buffer[i]) * scale;
            if (stretched < 0.0) stretched = 0.0;
            if (stretched > 255.0) stretched = 255.0;
            result_buffer[i] = static_cast<uint8_t>(stretched);
        }
    }
    
    //Update VideoFrameData
    grayIR.resize(cols, rows);
    memcpy(grayIR.data(), result_buffer.data(), totalPixels * sizeof(uint8_t));
    grayIR.setPixelFormat(TYPixelFormatMono8);
    grayIR.setWidth(cols);
    grayIR.setHeight(rows);
    
    return 0;
}
static int GrayIR_nonlinearStretch_log2(percipio_camera::TYImage& grayIR, double log_expandratio = 20)
{
    if (log_expandratio <= 0) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "GrayIR_nonlinearStretch_log multi_expandratio must be bigger than 0");
        return -1;
    }
    
    //Check pixel format
    uint32_t pixelFormat = grayIR.format();
    if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "GrayIR_linearStretch_std supports Mono8 or Mono16 gray, not other types");
        return -1;
    }
    
    int rows = grayIR.height();
    int cols = grayIR.width();
    size_t totalPixels = rows * cols;
    
    void* data = grayIR.data();
    if (!data) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "grayIR data is null");
        return -1;
    }
    
    //Create lookup table for 16-bit values (0-65535)
    std::vector<uint8_t> lut_16bit(65536);
    for (int i = 0; i < 65536; ++i) {
        double logValue = log_expandratio * log2(static_cast<double>(i) + 1.0);
        int intValue = static_cast<int>(logValue + 0.5); //Round to nearest integer
        if (intValue > 255) intValue = 255;
        lut_16bit[i] = static_cast<uint8_t>(intValue);
    }
    
    //Create lookup table for 8-bit values
    uint8_t lut_8bit[256];
    for (int i = 0; i < 256; ++i) {
        int scaledValue = i * 256;
        double logValue = log_expandratio * log2(static_cast<double>(scaledValue) + 1.0);
        int intValue = static_cast<int>(logValue + 0.5);
        if (intValue > 255) intValue = 255;
        lut_8bit[i] = static_cast<uint8_t>(intValue);
    }
    
    //Create result buffer
    static std::vector<uint8_t> result_buffer;
    if(result_buffer.size() != totalPixels) {
        result_buffer.resize(totalPixels);
    }
    
    //Apply stretching using lookup tables
    if (pixelFormat == TYPixelFormatMono16) {
        uint16_t* buffer = static_cast<uint16_t*>(data);
        for (size_t i = 0; i < totalPixels; ++i) {
            result_buffer[i] = lut_16bit[buffer[i]];
        }
    } 
    else { //TYPixelFormatMono8
        uint8_t* buffer = static_cast<uint8_t*>(data);
        for (size_t i = 0; i < totalPixels; ++i) {
            result_buffer[i] = lut_8bit[buffer[i]];
        }
    }
    
    //Update VideoFrameData
    grayIR.resize(cols, rows);
    memcpy(grayIR.data(), result_buffer.data(), totalPixels * sizeof(uint8_t));
    grayIR.setPixelFormat(TYPixelFormatMono8);
    grayIR.setWidth(cols);
    grayIR.setHeight(rows);
    
    return 0;
}
static int GrayIR_nonlinearStretch_hist(percipio_camera::TYImage& grayIR)
{
    //Check pixel format
    uint32_t pixelFormat = grayIR.format();
    if (!(pixelFormat == TYPixelFormatMono8 || pixelFormat == TYPixelFormatMono16)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "GrayIR_linearStretch_std supports Mono8 or Mono16 gray, not other types");
        return -1;
    }
    
    int rows = grayIR.height();
    int cols = grayIR.width();
    size_t totalPixels = rows * cols;
    
    void* data = grayIR.data();
    if (!data) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "grayIR data is null");
        return -1;
    }
    
    //For 16-bit images, perform linear stretching first
    if (pixelFormat == TYPixelFormatMono16) {
        //Use previously implemented linear stretch function
        int result = GrayIR_linearStretch(grayIR);
        if (result != 0) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "Linear stretch failed for 16-bit image");
            return result;
        }
        //Update data pointer after stretching
        data = grayIR.data();
    }
    
    //Now the image should be 8-bit
    if (grayIR.format() != TYPixelFormatMono8) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(PERCIPIO_IMAGE_PROCESS_LOG), "Expected 8-bit image after processing");
        return -1;
    }
    
    //Calculate histogram
    uint8_t* buffer = static_cast<uint8_t*>(data);
    int histogram[256] = {0};
    
    for (size_t i = 0; i < totalPixels; ++i) {
        histogram[buffer[i]]++;
    }
    
    //Calculate cumulative distribution function (CDF)
    int cdf[256] = {0};
    int minCdf = totalPixels; //Initialize with maximum possible value
    
    cdf[0] = histogram[0];
    for (int i = 1; i < 256; ++i) {
        cdf[i] = cdf[i-1] + histogram[i];
    }
    
    //Find minimum non-zero CDF value
    for (int i = 0; i < 256; ++i) {
        if (cdf[i] > 0 && cdf[i] < minCdf) {
            minCdf = cdf[i];
        }
    }
    
    //Create 8-bit result buffer
    static std::vector<uint8_t> result_buffer;
    if(result_buffer.size() != totalPixels) {
        result_buffer.resize(totalPixels);
    }
    
    //Apply histogram equalization
    for (size_t i = 0; i < totalPixels; ++i) {
        uint8_t pixelValue = buffer[i];
        //Apply histogram equalization formula:
        //h(v) = round((cdf(v) - minCdf) / (totalPixels - minCdf) * 255)
        int equalizedValue = static_cast<int>(
            (static_cast<double>(cdf[pixelValue] - minCdf) / 
            (totalPixels - minCdf)) * 255.0 + 0.5);
        
        //Clamp to 0-255 range
        if (equalizedValue < 0) equalizedValue = 0;
        if (equalizedValue > 255) equalizedValue = 255;
        
        result_buffer[i] = static_cast<uint8_t>(equalizedValue);
    }
    
    //Update VideoFrameData
    grayIR.resize(cols, rows);
    memcpy(grayIR.data(), result_buffer.data(), totalPixels * sizeof(uint8_t));
    grayIR.setPixelFormat(TYPixelFormatMono8);
    grayIR.setWidth(cols);
    grayIR.setHeight(rows);
    
    return 0;
}