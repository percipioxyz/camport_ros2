#include "percipio_video_mode.h"
#include "percipio_device.h"

namespace percipio_camera {

TYImage::TYImage() : m_Width(0), m_Height(0), m_fmt(TYPixelFormatInvalid), 
                m_ownsData(true), m_externalData(nullptr){}
    
TYImage::TYImage(const int _width, const int _height, const TYPixFmt _fmt)
        : m_Width(_width), m_Height(_height), m_fmt(_fmt), 
          m_ownsData(true), m_externalData(nullptr)
{
    if (_width > 0 && _height > 0 && _fmt != TYPixelFormatInvalid) {
        size_t size = calculateDataSize(_width, _height, _fmt);
        if (size > 0) {
            try {
                m_data.resize(size, 0);
            } catch (const std::bad_alloc& e) {
                clear();
                throw std::runtime_error("Failed to allocate memory for TYImage");
            }
        }
    }
}
    
TYImage::TYImage(const int _width, const int _height, const TYPixFmt _fmt, void* data)
        : m_Width(_width), m_Height(_height), m_fmt(_fmt), 
          m_ownsData(false), m_externalData(static_cast<unsigned char*>(data))
{
    if (_width <= 0 || _height <= 0 || _fmt == TYPixelFormatInvalid || data == nullptr) {
        m_Width = 0;
        m_Height = 0;
        m_fmt = TYPixelFormatInvalid;
        m_externalData = nullptr;
        m_ownsData = true;
    }
}
    
TYImage::TYImage(const TYImage& other)
        : m_Width(other.m_Width), m_Height(other.m_Height), 
          m_fmt(other.m_fmt), m_ownsData(true), m_externalData(nullptr)
{
    if (!other.empty()) {
        size_t size = calculateDataSize(m_Width, m_Height, m_fmt);
        if (size > 0) {
            try {
                m_data.resize(size);
                if (other.isExternalData()) {
                    memcpy(&m_data[0], other.data(), size);
                } else {
                    m_data = other.m_data;
                }
            } catch (const std::bad_alloc& e) {
                clear();
                throw std::runtime_error("Failed to allocate memory for TYImage copy");
            }
        }
    }
}
    
TYImage::TYImage(TYImage&& other) noexcept
        : m_Width(0), m_Height(0), m_fmt(TYPixelFormatInvalid), 
          m_ownsData(true), m_externalData(nullptr)
{
    *this = std::move(other);
}
    
TYImage::~TYImage() {
    cleanup();
}
    
TYImage& TYImage::operator=(const TYImage& other) {
    if (this != &other) {
        cleanup();
        
        m_Width = other.m_Width;
        m_Height = other.m_Height;
        m_fmt = other.m_fmt;
        m_ownsData = true;
        m_externalData = nullptr;
        
        if (!other.empty()) {
            size_t size = calculateDataSize(m_Width, m_Height, m_fmt);
            if (size > 0) {
                try {
                    m_data.resize(size);
                    if (other.isExternalData()) {
                        memcpy(m_data.data(), other.data(), size);
                    } else {
                        m_data = other.m_data;
                    }
                } catch (const std::bad_alloc& e) {
                    clear();
                    throw std::runtime_error("Failed to allocate memory for TYImage assignment");
                }
            }
        }
    }
    return *this;
}
    
TYImage& TYImage::operator=(TYImage&& other) noexcept {
    if (this != &other) {
        cleanup();
        
        m_Width = other.m_Width;
        m_Height = other.m_Height;
        m_fmt = other.m_fmt;
        m_ownsData = other.m_ownsData;
        
        if (other.m_ownsData) {
            m_data = std::move(other.m_data);
            m_externalData = nullptr;
        } else {
            m_externalData = other.m_externalData;
        }
        
        other.m_Width = 0;
        other.m_Height = 0;
        other.m_fmt = TYPixelFormatInvalid;
        other.m_data.clear();
        other.m_ownsData = true;
        other.m_externalData = nullptr;
    }
    return *this;
}
    
bool TYImage::empty() const {
    if (m_Width <= 0 || m_Height <= 0 || m_fmt == TYPixelFormatInvalid) {
        return true;
    }
    
    if (m_ownsData) {
        return m_data.empty();
    } else {
        return m_externalData == nullptr;
    }
}
    
TYImage TYImage::clone() const {
    return TYImage(*this);
}
    
TYImage TYImage::resize(int newWidth, int newHeight) const {
    if (newWidth <= 0 || newHeight <= 0 || m_fmt == TYPixelFormatInvalid) {
        return TYImage();
    }
    
    if (newWidth == m_Width && newHeight == m_Height) {
        return clone();
    }
    
    TYImage result(newWidth, newHeight, m_fmt);
    
    if (result.empty()) {
        return TYImage();
    }
    
    TY_STATUS status = TY_STATUS_OK;
    switch (m_fmt) {
        case TYPixelFormatMono8:
            status = resizeMono8To(newWidth, newHeight, result);
            break;
        case TYPixelFormatMono16:
        case TYPixelFormatCoord3D_C16:
            status = resizeMono16To(newWidth, newHeight, result);
            break;
        case TYPixelFormatRGB8:
        case TYPixelFormatBGR8:
            status = resizeRGB8To(newWidth, newHeight, result);
            break;
        case TYPixelFormatCoord3D_ABC32f:
            status = resizeABC32fTo(newWidth, newHeight, result);
            break;
        case TYPixelFormatCoord3D_ABC16:
            status = resizeABC16To(newWidth, newHeight, result);
            break;
        default:
            return TYImage();
    }
    
    if (status != TY_STATUS_OK) {
        return TYImage();
    }
    
    return result;
}
    
void TYImage::clear() {
    cleanup();
    
    m_Width = 0;
    m_Height = 0;
    m_fmt = TYPixelFormatInvalid;
    m_ownsData = true;
    m_externalData = nullptr;
    m_data.clear();
}
    
void TYImage::release() {
    if (!m_ownsData && m_externalData != nullptr) {
        size_t size = calculateDataSize(m_Width, m_Height, m_fmt);
        if (size > 0) {
            try {
                m_data.resize(size);
                memcpy(m_data.data(), m_externalData, size);
                m_ownsData = true;
                m_externalData = nullptr;
            } catch (const std::bad_alloc& e) {
                clear();
                throw std::runtime_error("Failed to allocate memory for TYImage release");
            }
        }
    }
}

void* TYImage::data() const {
    if (m_ownsData) {
        return (void*)m_data.data();
    } else {
        return (void*)m_externalData;
    }
}

size_t TYImage::size() const { 
    return calculateDataSize(m_Width, m_Height, m_fmt);
}
    
bool TYImage::isExternalData() const {
    return !m_ownsData;
}

size_t TYImage::calculateRowSize(int width, TYPixFmt fmt) const{
    size_t pixelSize = getPixelSize(fmt);
    if (pixelSize == 0) return 0;
    return width * pixelSize;
}

size_t TYImage::calculateDataSize(int width, int height, TYPixFmt fmt) const{
    size_t rowSize = calculateRowSize(width, fmt);
    if (rowSize == 0) return 0;
    return height * rowSize;
}

size_t TYImage::getPixelSize(TYPixFmt fmt) const{
    switch (fmt) {
        case TYPixelFormatMono8:
            return 1;
        case TYPixelFormatMono16:
        case TYPixelFormatCoord3D_C16:
            return 2;
        case TYPixelFormatRGB8:
        case TYPixelFormatBGR8:
            return 3;
        case TYPixelFormatCoord3D_ABC16:
            return 6;   //3 * 2 bytes (3 short)
        case TYPixelFormatCoord3D_ABC32f:
            return 12;  // 3 * 4 bytes (3 float)
        case TYPixelFormatInvalid:
        default:
            return 0;
    }
}

TY_STATUS TYImage::resizeMono8To(int newWidth, int newHeight, TYImage& dest) const {
    if (m_Width <= 0 || m_Height <= 0 || newWidth <= 0 || newHeight <= 0) {
        return TY_STATUS_ERROR;
    }
    
    if (empty()) {
        return TY_STATUS_ERROR;
    }
    
    const unsigned char* srcData = nullptr;
    if (m_ownsData) {
        srcData = m_data.data();
    } else {
        srcData = m_externalData;
    }
    
    if (!srcData) {
        return TY_STATUS_ERROR;
    }
    
    unsigned char* dstData = dest.m_data.data();
    if (!dstData) {
        return TY_STATUS_ERROR;
    }
    
    const float scaleX = static_cast<float>(m_Width) / static_cast<float>(newWidth);
    const float scaleY = static_cast<float>(m_Height) / static_cast<float>(newHeight);
    
    for (int32_t y = 0; y < newHeight; ++y) {
        const int32_t srcY = static_cast<int32_t>(std::floor(y * scaleY));
        const int32_t safeSrcY = (srcY < m_Height) ? srcY : (m_Height - 1);
        
        for (int32_t x = 0; x < newWidth; ++x) {
            const int32_t srcX = static_cast<int32_t>(std::floor(x * scaleX));
            const int32_t safeSrcX = (srcX < m_Width) ? srcX : (m_Width - 1);
            
            const unsigned char pixelValue = srcData[safeSrcY * m_Width + safeSrcX];
            dstData[y * newWidth + x] = pixelValue;
        }
    }
    
    return TY_STATUS_OK;
}

TY_STATUS TYImage::resizeMono16To(int newWidth, int newHeight, TYImage& dest) const {
    if (m_Width <= 0 || m_Height <= 0 || newWidth <= 0 || newHeight <= 0) {
        return TY_STATUS_ERROR;
    }
    
    if (empty()) {
        return TY_STATUS_ERROR;
    }
    
    const uint16_t* src = nullptr;
    if (m_ownsData) {
        src = reinterpret_cast<const uint16_t*>(m_data.data());
    } else {
        src = reinterpret_cast<const uint16_t*>(m_externalData);
    }
    
    if (!src) {
        return TY_STATUS_ERROR;
    }
    
    uint16_t* dst = reinterpret_cast<uint16_t*>(dest.m_data.data());
    if (!dst) {
        return TY_STATUS_ERROR;
    }
    
    const float scaleX = static_cast<float>(m_Width) / static_cast<float>(newWidth);
    const float scaleY = static_cast<float>(m_Height) / static_cast<float>(newHeight);
    
    for (int32_t y = 0; y < newHeight; ++y) {
        const int32_t srcY = static_cast<int32_t>(std::floor(y * scaleY));
        const int32_t safeSrcY = (srcY < m_Height) ? srcY : (m_Height - 1);
        
        for (int32_t x = 0; x < newWidth; ++x) {
            const int32_t srcX = static_cast<int32_t>(std::floor(x * scaleX));
            const int32_t safeSrcX = (srcX < m_Width) ? srcX : (m_Width - 1);
            
            const uint16_t pixelValue = src[safeSrcY * m_Width + safeSrcX];
            dst[y * newWidth + x] = pixelValue;
        }
    }
    
    return TY_STATUS_OK;
}

TY_STATUS TYImage::resizeRGB8To(int newWidth, int newHeight, TYImage& dest) const {
    if (m_Width <= 0 || m_Height <= 0 || newWidth <= 0 || newHeight <= 0) {
        return TY_STATUS_ERROR;
    }
    
    if (empty()) {
        return TY_STATUS_ERROR;
    }
    
    const unsigned char* srcData = nullptr;
    if (m_ownsData) {
        srcData = m_data.data();
    } else {
        srcData = m_externalData;
    }
    
    if (!srcData) {
        return TY_STATUS_ERROR;
    }
    
    unsigned char* dstData = dest.m_data.data();
    if (!dstData) {
        return TY_STATUS_ERROR;
    }
    
    const float scaleX = static_cast<float>(m_Width) / static_cast<float>(newWidth);
    const float scaleY = static_cast<float>(m_Height) / static_cast<float>(newHeight);
    
    for (int32_t y = 0; y < newHeight; ++y) {
        const int32_t srcY = static_cast<int32_t>(std::floor(y * scaleY));
        const int32_t safeSrcY = (srcY < m_Height) ? srcY : (m_Height - 1);
        
        for (int32_t x = 0; x < newWidth; ++x) {
            const int32_t srcX = static_cast<int32_t>(std::floor(x * scaleX));
            const int32_t safeSrcX = (srcX < m_Width) ? srcX : (m_Width - 1);
            
            const int srcIndex = (safeSrcY * m_Width + safeSrcX) * 3;
            const int dstIndex = (y * newWidth + x) * 3;
            
            dstData[dstIndex] = srcData[srcIndex];
            dstData[dstIndex + 1] = srcData[srcIndex + 1];
            dstData[dstIndex + 2] = srcData[srcIndex + 2];
        }
    }
    
    return TY_STATUS_OK;
}
TY_STATUS TYImage::resizeABC16To(int newWidth, int newHeight, TYImage& dest) const {
    if (m_Width <= 0 || m_Height <= 0 || newWidth <= 0 || newHeight <= 0) {
        return TY_STATUS_ERROR;
    }
    
    if (empty()) {
        return TY_STATUS_ERROR;
    }
    
    const short* src = nullptr;
    if (m_ownsData) {
        src = reinterpret_cast<const short*>(m_data.data());
    } else {
        src = reinterpret_cast<const short*>(m_externalData);
    }
    
    if (!src) {
        return TY_STATUS_ERROR;
    }
    
    short* dst = reinterpret_cast<short*>(dest.m_data.data());
    if (!dst) {
        return TY_STATUS_ERROR;
    }
    
    const float scaleX = static_cast<float>(m_Width) / static_cast<float>(newWidth);
    const float scaleY = static_cast<float>(m_Height) / static_cast<float>(newHeight);
    
    for (int32_t y = 0; y < newHeight; ++y) {
        const int32_t srcY = static_cast<int32_t>(std::floor(y * scaleY));
        const int32_t safeSrcY = (srcY < m_Height) ? srcY : (m_Height - 1);
        
        for (int32_t x = 0; x < newWidth; ++x) {
            const int32_t srcX = static_cast<int32_t>(std::floor(x * scaleX));
            const int32_t safeSrcX = (srcX < m_Width) ? srcX : (m_Width - 1);
            
            const int srcIndex = (safeSrcY * m_Width + safeSrcX) * 3;
            const int dstIndex = (y * newWidth + x) * 3;
            
            dst[dstIndex] = src[srcIndex];
            dst[dstIndex + 1] = src[srcIndex + 1];
            dst[dstIndex + 2] = src[srcIndex + 2];
        }
    }
    
    return TY_STATUS_OK;
}

TY_STATUS TYImage::resizeABC32fTo(int newWidth, int newHeight, TYImage& dest) const {
    if (m_Width <= 0 || m_Height <= 0 || newWidth <= 0 || newHeight <= 0) {
        return TY_STATUS_ERROR;
    }
    
    if (empty()) {
        return TY_STATUS_ERROR;
    }
    
    const float* src = nullptr;
    if (m_ownsData) {
        src = reinterpret_cast<const float*>(m_data.data());
    } else {
        src = reinterpret_cast<const float*>(m_externalData);
    }
    
    if (!src) {
        return TY_STATUS_ERROR;
    }
    
    float* dst = reinterpret_cast<float*>(dest.m_data.data());
    if (!dst) {
        return TY_STATUS_ERROR;
    }
    
    const float scaleX = static_cast<float>(m_Width) / static_cast<float>(newWidth);
    const float scaleY = static_cast<float>(m_Height) / static_cast<float>(newHeight);
    
    for (int32_t y = 0; y < newHeight; ++y) {
        const int32_t srcY = static_cast<int32_t>(std::floor(y * scaleY));
        const int32_t safeSrcY = (srcY < m_Height) ? srcY : (m_Height - 1);
        
        for (int32_t x = 0; x < newWidth; ++x) {
            const int32_t srcX = static_cast<int32_t>(std::floor(x * scaleX));
            const int32_t safeSrcX = (srcX < m_Width) ? srcX : (m_Width - 1);
            
            const int srcIndex = (safeSrcY * m_Width + safeSrcX) * 3;
            const int dstIndex = (y * newWidth + x) * 3;
            
            dst[dstIndex] = src[srcIndex];
            dst[dstIndex + 1] = src[srcIndex + 1];
            dst[dstIndex + 2] = src[srcIndex + 2];
        }
    }
    
    return TY_STATUS_OK;
}
////////////

void VideoStream::reset()
{
    _left_ir.release();
    _right_ir.release();
    _depth.release();
    _color.release();
    _p3d.release();
}

bool VideoStream::DepthInit(const TYImage& depth, image_intrinsic& intr, const uint64_t& timestamp)
{
    if(depth.empty()) {
        return false;
    }

    _depth = depth.clone();
    _depth_timestamp = timestamp;
    _depth_cam_info = convertToCameraInfo(intr.resize(depth.width(), depth.height()).data(), depth.width(), depth.height());

    
    return true;
}

bool VideoStream::ColorInit(const TYImage& color, image_intrinsic& intr, const uint64_t& timestamp)
{
    if(color.empty()) {
        return false;
    }

    _color = color.clone();
    _color_timestamp = timestamp;
    _color_cam_info = convertToCameraInfo(intr.resize(color.width(), color.height()).data(), color.width(), color.height());
    return true;
}

bool VideoStream::IRLeftInit(const TYImage& ir, image_intrinsic& intr, const uint64_t& timestamp)
{
    if(ir.empty()) {
        return false;
    }

    _left_ir = ir.clone();
    _lir_timestamp = timestamp;
    _lir_cam_info = convertToCameraInfo(intr.resize(ir.width(), ir.height()).data(), ir.width(), ir.height());
    return true;
}

bool VideoStream::IRRightInit(const TYImage& ir, image_intrinsic& intr, const uint64_t& timestamp)
{
    if(ir.empty()) {
        return false;
    }

    _right_ir = ir.clone();
    _rir_timestamp = timestamp;
    _rir_cam_info = convertToCameraInfo(intr.resize(ir.width(), ir.height()).data(), ir.width(), ir.height());
    return true;
}

bool VideoStream::PointCloudInit(const TYImage& p3d, image_intrinsic& intr, const uint64_t& timestamp)
{
    if(p3d.empty()) {
        return false;
    }

    if(p3d.format() != TYPixelFormatCoord3D_ABC32f) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("VideoStream"), "Invalid p3d data type!");
        return false;
    }
    
    _p3d = p3d.clone();
    _p3d_timestamp = timestamp;
    _p3d_cam_info = convertToCameraInfo(intr.resize(p3d.width(), p3d.height()).data(), p3d.width(), p3d.height());
    return true;
}

sensor_msgs::msg::CameraInfo VideoStream::convertToCameraInfo(const TY_CAMERA_INTRINSIC& intr, const int width, const int height)
{
    sensor_msgs::msg::CameraInfo ros_cam_info;
    ros_cam_info.width = width;
    ros_cam_info.height = height;

    //No distortion
    ros_cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    ros_cam_info.d.resize(12, 0.0);

    ros_cam_info.k.fill(0.0);
    ros_cam_info.k[0] = intr.data[0];   //fx
    ros_cam_info.k[2] = intr.data[2];   //cx
    ros_cam_info.k[4] = intr.data[4];   //fy;
    ros_cam_info.k[5] = intr.data[5];   //cy;
    ros_cam_info.k[8] = 1.0;

    ros_cam_info.r.fill(0.0);
    ros_cam_info.r[0] = 1;
    ros_cam_info.r[4] = 1;
    ros_cam_info.r[8] = 1;

    ros_cam_info.p.fill(0.0);
    ros_cam_info.p[0] = ros_cam_info.k[0];
    ros_cam_info.p[2] = ros_cam_info.k[2];
    ros_cam_info.p[5] = ros_cam_info.k[4];
    ros_cam_info.p[6] = ros_cam_info.k[5];
    ros_cam_info.p[10] = 1.0;
    
    return ros_cam_info;
}

}