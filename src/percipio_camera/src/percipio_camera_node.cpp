#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <filesystem>
#include <fstream>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "percipio_camera_node.h"

#if USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "percipio_video_mode.h"

namespace percipio_camera {

#define LOG_HEAD_PERCIPIO_CAMERA_NODE  "percipio_camera_node"

const static percipio_stream_index_pair DEPTH_STREAM{DEPTH, 0};
const static percipio_stream_index_pair COLOR_STREAM{COLOR, 0};
const static percipio_stream_index_pair LEFT_IR_STREAM{IR_LEFT, 0};
const static percipio_stream_index_pair RIGHT_IR_STREAM{IR_RIGHT, 0};
const static std::vector<percipio_stream_index_pair> PERCIPIO_IMAGE_STREAMS = {DEPTH_STREAM, COLOR_STREAM, LEFT_IR_STREAM, RIGHT_IR_STREAM};

static rclcpp::Time HWTimeUsToROSTime(uint64_t us)
{
    uint64_t sec = us / 1000000;
    uint64_t nano_sec = 1000* (us % 1000000);
    return rclcpp::Time(sec, nano_sec);
}

static rmw_qos_profile_t getRMWQosProfileFromString(const std::string &str_qos) {
  std::string upper_str_qos = str_qos;
  std::transform(upper_str_qos.begin(), upper_str_qos.end(), upper_str_qos.begin(), ::toupper);
  if (upper_str_qos == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  } else if (upper_str_qos == "DEFAULT") {
    return rmw_qos_profile_default;
  } else if (upper_str_qos == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  } else if (upper_str_qos == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  } else if (upper_str_qos == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  } else if (upper_str_qos == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE),
                        "Invalid QoS profile: " << upper_str_qos << ". Using default QoS profile.");
    return rmw_qos_profile_default;
  }
}


PercipioCameraNode::PercipioCameraNode(rclcpp::Node* node, std::shared_ptr<PercipioDevice>& device) 
        :node_(node),
    device_ptr_(device) {
    stream_name_[DEPTH_STREAM] = "depth";
    stream_name_[COLOR_STREAM] = "color";
    stream_name_[LEFT_IR_STREAM] = "left_ir";
    stream_name_[RIGHT_IR_STREAM] = "right_ir";
    device_ptr_->register_node(this);
    setupTopics();
}

PercipioCameraNode::~PercipioCameraNode()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "PercipioCameraNode shutting down");
  if(device_ptr_.get()) {
    device_ptr_->stream_stop();
  }
}

template <class T>
void PercipioCameraNode::setAndGetNodeParameter(
    T &param, const std::string &param_name, const T &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor) {
    rclcpp::ParameterValue result_value(default_value);
    if (!node_->has_parameter(param_name)) {
      result_value = node_->declare_parameter(param_name, rclcpp::ParameterValue(default_value), parameter_descriptor);
    } else {
      result_value = node_->get_parameter(param_name).get_parameter_value();
    }
    param = result_value.get<T>();
}

void PercipioCameraNode::getParameters() {
    setAndGetNodeParameter<std::string>(camera_name_, "camera_name", "camera");
    camera_link_frame_id_ = camera_name_ + "_link";

    std::string param_name_desc;
    for (auto index : PERCIPIO_IMAGE_STREAMS) {
        //stream enable
        param_name_desc = stream_name_[index] + "_enable";
        setAndGetNodeParameter(stream_enable_[index], param_name_desc, false);

        //stream resolution
        param_name_desc = stream_name_[index] + "_resolution";
        setAndGetNodeParameter<std::string>(stream_resolution_[index], param_name_desc, "");

        //stream image format
        param_name_desc = stream_name_[index] + "_format";
        setAndGetNodeParameter<std::string>(stream_image_mode_[index], param_name_desc, "");

        //qos setting
        param_name_desc = stream_name_[index] + "_qos";
        setAndGetNodeParameter<std::string>(stream_qos_[index], param_name_desc, "default");

        param_name_desc = stream_name_[index] + "_camera_info_qos";
        setAndGetNodeParameter<std::string>(camera_info_qos_[index], param_name_desc, "default");

        frame_id_[index] = camera_name_ + "_" + stream_name_[index] + "_frame";
        optical_frame_id_[index] = camera_name_ + "_" + stream_name_[index] + "_optical_frame";
    }

    //device offline auto reconnection
    setAndGetNodeParameter(offline_auto_reconnection_, "device_auto_reconnect", false);

    //device  frame rate control
    setAndGetNodeParameter(device_frame_rate_control_, "frame_rate_control", false);
    setAndGetNodeParameter<float>(device_frame_rate_, "frame_rate", 5.0);
    
    //laser power flag
    setAndGetNodeParameter(laser_power_, "laser_power", -1);

    //registration flag
    setAndGetNodeParameter(depth_registration_enable_, "depth_registration_enable", false);

    //depth spec filter
    setAndGetNodeParameter(depth_speckle_filter_enable_, "depth_speckle_filter", false);
    setAndGetNodeParameter(max_speckle_size_, "max_speckle_size", 150);
    setAndGetNodeParameter(max_speckle_diff_, "max_speckle_diff", 64);
    setAndGetNodeParameter<float>(max_physical_size_, "max_physical_size", 20.0);

    //depth time domain filter
    setAndGetNodeParameter(depth_time_domain_filter_enable_, "depth_time_domain_filter", false);
    setAndGetNodeParameter(depth_time_domain_num_, "depth_time_domain_num", 3);

    //point cloud qos setting
    setAndGetNodeParameter<std::string>(point_cloud_qos_, "point_cloud_qos", "default");

    setAndGetNodeParameter(point_cloud_enable_, "point_cloud_enable", true);

    setAndGetNodeParameter(color_point_cloud_enable_, "color_point_cloud_enable", false);

    setAndGetNodeParameter(ir_undistortion_, "ir_undistortion", true);

    static std::map<std::string, ir_enhance_model> ir_enhancement_list = {
        {"off",           IREnhanceOFF},
        {"linear",        IREnhanceLinearStretch},
        {"multi_linear",  IREnhanceLinearStretch_Multi},
        {"std_linear",    IREnhanceLinearStretch_STD},
        {"log",           IREnhanceLinearStretch_LOG2},
        {"hist",          IREnhanceLinearStretch_Hist}
    };
    
    std::string enhance_mode_desc;
    setAndGetNodeParameter<std::string>(enhance_mode_desc, "ir_enhancement", "off");
    auto iter = ir_enhancement_list.find(enhance_mode_desc);
    ir_enhance_mode_ = (iter != ir_enhancement_list.end()) ? iter->second : IREnhanceOFF;
    setAndGetNodeParameter(ir_enhancement_coefficient_, "ir_enhancement_coefficient", 6);

    if(color_point_cloud_enable_)
        depth_registration_enable_ = true;

    //disable registration if color stream is closed
    if (!stream_enable_[COLOR_STREAM]) {
        color_point_cloud_enable_ = false;
        depth_registration_enable_ = false;
    }
}

bool PercipioCameraNode::setupDevices() {
    device_ptr_->depth_speckle_filter_init(depth_speckle_filter_enable_, max_speckle_size_, max_speckle_diff_, max_physical_size_);
    device_ptr_->depth_time_domain_filter_init(depth_time_domain_filter_enable_, depth_time_domain_num_);

    device_ptr_->ir_enhance_mode_init(ir_enhance_mode_, ir_enhancement_coefficient_);
    device_ptr_->ir_undistortion_enable(ir_undistortion_);

    device_ptr_->registerCameraEventCallback(boost::bind(&PercipioCameraNode::onCameraEventCallback, this, _1, _2));

    if(!device_ptr_->hasColor()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Color stream not supported by device");
        stream_enable_[COLOR_STREAM] = false;
    }

    if(!device_ptr_->hasDepth()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Depth stream not supported by device");
        stream_enable_[DEPTH_STREAM] = false;
    }

    if(!device_ptr_->hasLeftIR()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Left-IR stream not supported by device");
        stream_enable_[LEFT_IR_STREAM] = false;
    }

    if(!device_ptr_->hasRightIR()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Right-IR stream not supported by device");
        stream_enable_[RIGHT_IR_STREAM] = false;
    }

    device_ptr_->enable_offline_reconnect(offline_auto_reconnection_);

    device_ptr_->frame_rate_init(device_frame_rate_control_, device_frame_rate_);

    if(laser_power_ >= 0)
        device_ptr_->set_laser_power(laser_power_);

    for (auto index : PERCIPIO_IMAGE_STREAMS) {
        if(stream_enable_[index]) {
            if(!device_ptr_->stream_open(index, stream_resolution_[index], stream_image_mode_[index])) {
                return false;
            }
        } else {
            if(index == DEPTH_STREAM) {
                if(point_cloud_enable_ || color_point_cloud_enable_) {
                    if(!device_ptr_->stream_open(index, stream_resolution_[index], stream_image_mode_[index])) {
                        return false;
                    }
                } else {
                    device_ptr_->stream_close(index);
                }
            } else {
                device_ptr_->stream_close(index);
            }
        }
    }

    if (!stream_enable_[COLOR_STREAM]) {
        color_point_cloud_enable_ = false;
        depth_registration_enable_ = false;
    }

    if(color_point_cloud_enable_) {
        point_cloud_enable_ = false;
    }

    device_ptr_->topics_depth_stream_enable(stream_enable_[DEPTH_STREAM]);
    device_ptr_->topics_point_cloud_enable(point_cloud_enable_);
    device_ptr_->topics_color_point_cloud_enable(color_point_cloud_enable_);
    device_ptr_->topics_depth_registration_enable(depth_registration_enable_);

    if(stream_enable_[DEPTH_STREAM])
        depth_scale_ = device_ptr_->getDepthValueScale();

    return startStreams();
}

bool PercipioCameraNode::startStreams() {
    device_ptr_->setFrameCallback(boost::bind(&PercipioCameraNode::onNewFrame, this, _1));
    return device_ptr_->stream_start();
}

void PercipioCameraNode::topic_softtrigger_callback(const std_msgs::msg::String::SharedPtr) const
{
    RCLCPP_INFO(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Received soft trigger signal");
    device_ptr_->send_softtrigger();
}

void PercipioCameraNode::topic_dynamic_config_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Received dynamic configuration");
    device_ptr_->setDeviceConfig(msg->data);
}

void PercipioCameraNode::topic_device_reset_callback(const std_msgs::msg::Empty::SharedPtr) const
{
    RCLCPP_INFO(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Received device reset request");
    device_ptr_->reset();
}

void PercipioCameraNode::setupSubscribers() {
    trigger_event_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "soft_trigger", rclcpp::SensorDataQoS(),
            std::bind(&PercipioCameraNode::topic_softtrigger_callback, this, std::placeholders::_1));

    
    config_event_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "dynamic_config", rclcpp::SensorDataQoS(),
            std::bind(&PercipioCameraNode::topic_dynamic_config_callback, this, std::placeholders::_1));

    device_reset_event_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
            "reset", rclcpp::SensorDataQoS(),
            std::bind(&PercipioCameraNode::topic_device_reset_callback, this, std::placeholders::_1));
}

void PercipioCameraNode::setupPublishers() {  
    auto point_cloud_qos_profile = getRMWQosProfileFromString(point_cloud_qos_);

    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    if (color_point_cloud_enable_) {
        color_point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "depth_registered/points",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
            point_cloud_qos_profile),
            pub_options);
    }

    if (point_cloud_enable_) {
        point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "depth/points", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
            point_cloud_qos_profile),
            pub_options);
    }

    for (const auto &stream_index : PERCIPIO_IMAGE_STREAMS) {
        if (!stream_enable_[stream_index]) {
            continue;
        }

        std::string name = stream_name_[stream_index];
        std::string topic = name + "/image_raw";
        auto image_qos = stream_qos_[stream_index];
        auto image_qos_profile = getRMWQosProfileFromString(image_qos);
        image_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::Image>(
            topic, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_qos_profile),
            image_qos_profile),
            pub_options);

        topic = name + "/camera_info";
        auto camera_info_qos = camera_info_qos_[stream_index];
        auto camera_info_qos_profile = getRMWQosProfileFromString(camera_info_qos);
        camera_info_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic, 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
            camera_info_qos_profile),
            pub_options);
    }

    device_event_publisher_ = node_->create_publisher<std_msgs::msg::String>("device_event", rclcpp::QoS(1).transient_local());
}

void PercipioCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  setupPublishers();
  setupSubscribers();
}

void PercipioCameraNode::SendOfflineMsg(const char* sn) {
    auto msg = std_msgs::msg::String();
    msg.data = " DeviceOffline<" + std::string(sn) + ">";
    RCLCPP_INFO(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Device event: %s", msg.data.c_str());
    device_event_publisher_->publish(std::move(msg));
}

void PercipioCameraNode::SendConnectMsg(const char* sn) {
    auto msg = std_msgs::msg::String();
    msg.data = " DeviceConnect<" + std::string(sn) + ">";
    RCLCPP_INFO(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Device event: %s", msg.data.c_str());
    device_event_publisher_->publish(std::move(msg));
}

void PercipioCameraNode::SendTimeoutMsg(const char* sn) {
    auto msg = std_msgs::msg::String();
    msg.data = " DeviceTimeout<" + std::string(sn) + ">";
    RCLCPP_INFO(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Device event: %s", msg.data.c_str());
    device_event_publisher_->publish(std::move(msg));
}

#define SUBSCRIBER_CHECK(has)  do {   \
    if(!has) return; \
} while(0)

void PercipioCameraNode::publishImageMsg(const percipio_stream_index_pair& idx,
        const void* data, int width, int height,
        const char* encoding, int bytes_per_pixel,
        uint64_t timestamp_us,
        const sensor_msgs::msg::CameraInfo& info)
{
    camera_info_publishers_[idx]->publish(info);

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = HWTimeUsToROSTime(timestamp_us);
    msg->header.frame_id = optical_frame_id_[idx];
    msg->height = height;
    msg->width = width;
    msg->encoding = encoding;
    msg->is_bigendian = false;
    msg->step = bytes_per_pixel * width;

    size_t data_size = msg->step * msg->height;
    msg->data.resize(data_size);
    memcpy(msg->data.data(), data, data_size);

    image_publishers_[idx]->publish(std::move(msg));
}

void PercipioCameraNode::publishColorFrame(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = image_publishers_[COLOR_STREAM]->get_subscription_count() > 0;
    SUBSCRIBER_CHECK(has_subscriber);

    const TYImage& color = stream.getColorImage();
    if (color.empty()) {
        return;
    }

    auto image_info = stream.getColorInfo();
    image_info.header.stamp = HWTimeUsToROSTime(stream.getColorStreamTimestamp());
    image_info.header.frame_id = optical_frame_id_[COLOR_STREAM];
    image_info.width = color.width();
    image_info.height = color.height();

    publishImageMsg(COLOR_STREAM, color.data(), color.width(), color.height(),
                    sensor_msgs::image_encodings::BGR8, 3,
                    stream.getColorStreamTimestamp(), image_info);
}

void PercipioCameraNode::publishIRFrame(percipio_camera::VideoStream& stream, const percipio_stream_index_pair& ir_stream)
{
    bool has_subscriber = image_publishers_[ir_stream]->get_subscription_count() > 0;
    SUBSCRIBER_CHECK(has_subscriber);

    const TYImage& ir = (ir_stream == LEFT_IR_STREAM) ? stream.getLeftIRImage() : stream.getRightIRImage();
    if (ir.empty()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "IR image buffer is empty");
        return;
    }

    TYPixFmt fmt = ir.format();
    const char* encoding_type = nullptr;
    int bytes_per_pixel = 0;
    if (fmt == TYPixelFormatMono8) {
        encoding_type = sensor_msgs::image_encodings::MONO8;
        bytes_per_pixel = 1;
    } else if (fmt == TYPixelFormatMono16) {
        encoding_type = sensor_msgs::image_encodings::MONO16;
        bytes_per_pixel = 2;
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Invalid ir image format.");
        return;
    }

    auto image_info = (ir_stream == LEFT_IR_STREAM) ? stream.getLeftIRInfo() : stream.getRightIRInfo();
    uint64_t timestamp = (ir_stream == LEFT_IR_STREAM) ? stream.getLeftIRStreamTimestamp() : stream.getRightIRStreamTimestamp();
    image_info.header.stamp = HWTimeUsToROSTime(timestamp);
    image_info.header.frame_id = optical_frame_id_[ir_stream];
    image_info.width = ir.width();
    image_info.height = ir.height();

    publishImageMsg(ir_stream, ir.data(), ir.width(), ir.height(),
                    encoding_type, bytes_per_pixel, timestamp, image_info);
}

void PercipioCameraNode::publishDepthFrame(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = image_publishers_[DEPTH_STREAM]->get_subscription_count() > 0;
    SUBSCRIBER_CHECK(has_subscriber);

    const TYImage& image = stream.getDepthImage();
    if (image.empty()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE), "Depth image buffer is empty");
        return;
    }

    auto image_info = stream.getDepthInfo();
    image_info.header.stamp = HWTimeUsToROSTime(stream.getDepthStreamTimestamp());
    image_info.header.frame_id = optical_frame_id_[DEPTH_STREAM];
    image_info.width = image.width();
    image_info.height = image.height();

    const char* encoding = nullptr;
    int bytes_per_pixel = 0;
    if (image.format() == TYPixelFormatCoord3D_C16) {
        encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        bytes_per_pixel = 2;
    } else if (image.format() == TYPixelFormatCoord3D_ABC16) {
        encoding = sensor_msgs::image_encodings::TYPE_16SC3;
        bytes_per_pixel = 6;
    }

    if (encoding) {
        publishImageMsg(DEPTH_STREAM, image.data(), image.width(), image.height(),
                        encoding, bytes_per_pixel,
                        stream.getDepthStreamTimestamp(), image_info);
    }
}

//#define PUBLISH_INVALID_POINT_CLOUD_DATA
void PercipioCameraNode::publishColorPointCloud(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = color_point_cloud_pub_->get_subscription_count() > 0;
    SUBSCRIBER_CHECK(has_subscriber);
    
    const TYImage& p3d = stream.getPointCloud();
    const TYImage& color = stream.getColorImage();
    if(p3d.empty() || color.empty()) {
        return;
    }

    TYImage rsz_color = color.resize(p3d.width(), p3d.height());
    const auto *p3d_data = (float *)p3d.data();
    const auto *color_data = (uint8_t *)rsz_color.data();
    auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    point_cloud_msg->width = p3d.width();
    point_cloud_msg->height = p3d.height();
    std::string format_str = "rgb";
    point_cloud_msg->point_step =
                    addPointField(*point_cloud_msg, format_str, 1, sensor_msgs::msg::PointField::FLOAT32,
                                  static_cast<int>(point_cloud_msg->point_step));
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
    
    const int width = p3d.width();
    const int height = p3d.height();
    size_t valid_count = 0;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud_msg, "b");
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = 3 * (y * width + x);
            float depth = p3d_data[idx + 2];
            if (!std::isnan(depth)) {
                *iter_x = p3d_data[idx + 0] / 1000.0;
                *iter_y = p3d_data[idx + 1] / 1000.0;
                *iter_z = depth / 1000.0;
                *iter_b = color_data[idx + 0];
                *iter_g = color_data[idx + 1];
                *iter_r = color_data[idx + 2];
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
                ++iter_g;
                ++iter_b;
                ++valid_count;
            }
        }
    }
    point_cloud_msg->is_dense = true;
#ifdef PUBLISH_INVALID_POINT_CLOUD_DATA
    point_cloud_msg->width = width;
    point_cloud_msg->height = height;
#else
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
#endif

    point_cloud_msg->header.stamp = HWTimeUsToROSTime(stream.getPointCloudStreamTimestamp());
    point_cloud_msg->header.frame_id = optical_frame_id_[DEPTH_STREAM];
    color_point_cloud_pub_->publish(std::move(point_cloud_msg));
}

void PercipioCameraNode::publishPointCloud(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = point_cloud_pub_->get_subscription_count() > 0;
    SUBSCRIBER_CHECK(has_subscriber);

    const TYImage& p3d = stream.getPointCloud();
    if(p3d.empty()) {
        return;
    }

    const auto *p3d_data = (float *)p3d.data();
    auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    const int width = p3d.width();
    const int height = p3d.height();
    modifier.resize(width * height);
    point_cloud_msg->width = width;
    point_cloud_msg->height = height;
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);

    size_t valid_count = 0;    
    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = 3 * (y * width + x);
            float depth = p3d_data[idx + 2];
            if (!std::isnan(depth)) {
                *iter_x = p3d_data[idx + 0] / 1000.0;
                *iter_y = p3d_data[idx + 1] / 1000.0;
                *iter_z = depth / 1000.0;

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++valid_count;
            } else {
#ifdef PUBLISH_INVALID_POINT_CLOUD_DATA
                *iter_x = 0;
                *iter_y = 0;
                *iter_z = 0;

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++valid_count;
#endif
            }
        }
    }

    point_cloud_msg->is_dense = true;
#ifdef PUBLISH_INVALID_POINT_CLOUD_DATA
    point_cloud_msg->width = width;
    point_cloud_msg->height = height;
#else
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);
#endif

    point_cloud_msg->header.stamp = HWTimeUsToROSTime(stream.getPointCloudStreamTimestamp());
    point_cloud_msg->header.frame_id = optical_frame_id_[DEPTH_STREAM];
    point_cloud_pub_->publish(std::move(point_cloud_msg));
}


void PercipioCameraNode::onCameraEventCallback(PercipioDevice* Handle, TY_EVENT_INFO *event_info)
{
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE) ,  "Device Event Callback: Device Offline, SN = " << Handle->serialNumber());
        SendOfflineMsg(Handle->serialNumber().c_str());
    } else if(event_info->eventId == TY_EVENT_DEVICE_CONNECT) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE) ,  "Device Event Callback: Device Connect, SN = " << Handle->serialNumber());
        SendConnectMsg(Handle->serialNumber().c_str());
    } else if(event_info->eventId == TY_EVENT_DEVICE_TIMEOUT) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(LOG_HEAD_PERCIPIO_CAMERA_NODE),  "Device Event Callback: Device Timeout, SN = " << Handle->serialNumber());
        SendTimeoutMsg(Handle->serialNumber().c_str());
    }
}

void PercipioCameraNode::publishStaticTF(const rclcpp::Time &t, const tf2::Vector3 &trans,
                                   const tf2::Quaternion &q, const std::string &from,
                                   const std::string &to) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans[2] / 1000.0;
  msg.transform.translation.y = -trans[0] / 1000.0;
  msg.transform.translation.z = -trans[1] / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();

  static_tf_msgs_.push_back(msg);
}

void PercipioCameraNode::publishStaticTransforms() {
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    tf2::Vector3 zero_trans(0.0, 0.0, 0.0);
    for (const auto &stream_index : PERCIPIO_IMAGE_STREAMS) {
        if(stream_enable_[stream_index]) {
            tf2::Vector3 trans(0.0, 0.0, 0.0);
            tf2::Quaternion Q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

            auto timestamp = node_->now();
            const std::string& frame_id = frame_id_[stream_index];
            const std::string& optical_frame_id = optical_frame_id_[stream_index];

            publishStaticTF(timestamp, trans,      Q,                  camera_link_frame_id_,   frame_id);
            publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id,                optical_frame_id);
        }
    }

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
}

void PercipioCameraNode::onNewFrame(percipio_camera::VideoStream& stream) 
{
    if (!tf_published_) {
      publishStaticTransforms();
      tf_published_ = true;
    }

    if(stream_enable_[DEPTH_STREAM]) {
        publishDepthFrame(stream);
    }

    if(stream_enable_[COLOR_STREAM]) {
        publishColorFrame(stream);
    }

    if(stream_enable_[LEFT_IR_STREAM]) {
        publishIRFrame(stream, LEFT_IR_STREAM);
    }

    if(stream_enable_[RIGHT_IR_STREAM]) {
        publishIRFrame(stream, RIGHT_IR_STREAM);
    }

    if(color_point_cloud_enable_) {
        publishColorPointCloud(stream);
    }
    
    if(point_cloud_enable_) {
        publishPointCloud(stream);
    } 
}

}