#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <filesystem>
#include <fstream>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "percipio_camera_node.h"

namespace percipio_camera {

const static percipio_stream_index_pair DEPTH_STREAM{DEPTH, 0};
const static percipio_stream_index_pair COLOR_STREAM{COLOR, 0};
const static percipio_stream_index_pair LEFT_IR_STREAM{IR_LETF, 0};
const static percipio_stream_index_pair RIGHT_IR_STREAM{IR_RIGHT, 0};
const static std::vector<percipio_stream_index_pair> PERCIPIO_IMAGE_STREAMS = {DEPTH_STREAM, COLOR_STREAM, LEFT_IR_STREAM, RIGHT_IR_STREAM};

static rclcpp::Time HWTimeUsToROSTime(uint64_t us)
{
    uint64_t sec = us / 1000000;
    uint64_t nano_sec = us % 1000000;
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
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"),
                        "Invalid QoS profile: " << upper_str_qos << ". Using default QoS profile.");
    return rmw_qos_profile_default;
  }
}

PercipioCameraNode::PercipioCameraNode(rclcpp::Node* node, std::shared_ptr<PercipioDevice>& device) 
        :node_(node),
    device_ptr(device) {
    stream_name[DEPTH_STREAM] = "depth";
    stream_name[COLOR_STREAM] = "color";
    stream_name[LEFT_IR_STREAM] = "left_ir";
    stream_name[RIGHT_IR_STREAM] = "right_ir";
    setupTopics();
}

PercipioCameraNode::~PercipioCameraNode()
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "PercipioCameraNode release...");
  if(device_ptr.get()) {
    device_ptr->stream_stop();
  }
}

template <class T>
void PercipioCameraNode::setAndGetNodeParameter(
    T &param, const std::string &param_name, const T &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor) {
    rclcpp::ParameterValue result_value(default_value);
    const rcl_interfaces::msg::ParameterDescriptor& descriptor = rcl_interfaces::msg::ParameterDescriptor();
    if (!node_->has_parameter(param_name)) {
      result_value = node_->declare_parameter(param_name, rclcpp::ParameterValue(default_value), descriptor);
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
        param_name_desc = stream_name[index] + "_enable";
        setAndGetNodeParameter(stream_enable[index], param_name_desc, false);

        //stream resolution
        param_name_desc = stream_name[index] + "_resolution";
        setAndGetNodeParameter<std::string>(stream_resolution[index], param_name_desc, "");

        //stream image format
        param_name_desc = stream_name[index] + "_format";
        setAndGetNodeParameter<std::string>(stream_image_mode[index], param_name_desc, "");

        //qos setting
        param_name_desc = stream_name[index] + "_qos";
        setAndGetNodeParameter<std::string>(stream_qos[index], param_name_desc, "default");

        param_name_desc = stream_name[index] + "_camera_info_qos";
        setAndGetNodeParameter<std::string>(camera_info_qos_[index], param_name_desc, "default");
    }

    //registration flag
    setAndGetNodeParameter(m_laser_power, "laser_power", -1);

    //registration flag
    setAndGetNodeParameter(depth_registration_enable, "depth_registration_enable", false);

    //point cloud qos setting
    setAndGetNodeParameter<std::string>(point_cloud_qos, "point_cloud_qos", "default");

    setAndGetNodeParameter(point_cloud_enable, "point_cloud_enable", true);

    setAndGetNodeParameter(color_point_cloud_enable, "color_point_cloud_enable", false);

    setAndGetNodeParameter(tof_depth_quality, "tof_depth_quality", std::string(""));
    setAndGetNodeParameter(m_tof_modulation_threshold, "tof_modulation_threshold", -1);
    setAndGetNodeParameter(m_tof_jitter_threshold, "tof_jitter_threshold", -1);
    setAndGetNodeParameter(m_tof_filter_threshold, "tof_filter_threshold", -1);
    setAndGetNodeParameter(m_tof_channel, "tof_channel", -1);
    setAndGetNodeParameter(m_tof_HDR_ratio, "tof_HDR_ratio", -1);

    if(color_point_cloud_enable)
        depth_registration_enable = true;

    //disable registration if color stream is closed
    if (!stream_enable[COLOR_STREAM]) {
        color_point_cloud_enable = false;
        depth_registration_enable = false;
    }
}

void PercipioCameraNode::setupDevices() {
    if(!device_ptr->hasColor()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Color image data stream is not supported!");
        stream_enable[COLOR_STREAM] = false;
    }

    if(!device_ptr->hasDepth()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Depth image data stream is not supported!");
        stream_enable[DEPTH_STREAM] = false;
    }

    if(!device_ptr->hasLeftIR()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Left-IR image data stream is not supported!");
        stream_enable[LEFT_IR_STREAM] = false;
    }

    if(!device_ptr->hasRightIR()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("percipio_device"), "Right-IR image data stream is not supported!");
        stream_enable[RIGHT_IR_STREAM] = false;
    }

    if(m_laser_power >= 0)
        device_ptr->set_laser_power(m_laser_power);

    for (auto index : PERCIPIO_IMAGE_STREAMS) {
        if(stream_enable[index]) {
            device_ptr->stream_open(index, stream_resolution[index], stream_image_mode[index]);   
        } else {
            if(index == DEPTH_STREAM) {
                if(point_cloud_enable || color_point_cloud_enable) {
                    device_ptr->stream_open(index, stream_resolution[index], stream_image_mode[index]);
                } else {
                    device_ptr->stream_close(index);
                }
            } else {
                device_ptr->stream_close(index);
            }
        }
    }

    if(!tof_depth_quality.empty())
        device_ptr->set_tof_depth_quality(tof_depth_quality);

    if(m_tof_modulation_threshold)
        device_ptr->set_tof_modulation_threshold(m_tof_modulation_threshold);

    if(m_tof_jitter_threshold)
        device_ptr->set_tof_jitter_threshold(m_tof_jitter_threshold);

    if(m_tof_filter_threshold)
        device_ptr->set_tof_filter_threshold(m_tof_filter_threshold);

    if(m_tof_channel)
        device_ptr->set_tof_channel(m_tof_channel);

    if(m_tof_HDR_ratio)
        device_ptr->set_tof_HDR_ratio(m_tof_HDR_ratio);

    if (!stream_enable[COLOR_STREAM]) {
        color_point_cloud_enable = false;
        depth_registration_enable = false;
    }

    device_ptr->topics_depth_stream_enable(stream_enable[DEPTH_STREAM]);
    device_ptr->topics_point_cloud_enable(point_cloud_enable);
    device_ptr->topics_color_point_cloud_enable(color_point_cloud_enable);
    device_ptr->topics_depth_registration_enable(depth_registration_enable);

    startStreams();
}

void PercipioCameraNode::startStreams() {
    device_ptr->setFrameCallback(boost::bind(&PercipioCameraNode::onNewFrame, this, _1));
    device_ptr->stream_start();
}

void PercipioCameraNode::setupPublishers() {
    auto point_cloud_qos_profile = getRMWQosProfileFromString(point_cloud_qos);
    if (color_point_cloud_enable) {
        color_point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "depth_registered/points",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
            point_cloud_qos_profile));
    }

    if (point_cloud_enable) {
        point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "depth/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
            point_cloud_qos_profile));
    }

    for (const auto &stream_index : PERCIPIO_IMAGE_STREAMS) {
        if (!stream_enable[stream_index]) {
            continue;
        }

        std::string name = stream_name[stream_index];
        std::string topic = name + "/image_raw";
        auto image_qos = stream_qos[stream_index];
        auto image_qos_profile = getRMWQosProfileFromString(image_qos);
        image_publishers_[stream_index] = image_transport::create_publisher(node_, topic, image_qos_profile);

        topic = name + "/camera_info";
        auto camera_info_qos = camera_info_qos_[stream_index];
        auto camera_info_qos_profile = getRMWQosProfileFromString(camera_info_qos);
        camera_info_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
            camera_info_qos_profile));
    }

    _tf = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PercipioCameraNode::broadcast_timer_callback, this));
}

void PercipioCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  setupPublishers();
}

void PercipioCameraNode::broadcast_timer_callback() 
{
    rclcpp::Time now;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = camera_link_frame_id_;
    t.child_frame_id = camera_name_ + "_" + stream_name[DEPTH_STREAM] + "_optical_frame";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
 
    _tf->sendTransform(t);
}

#define SUBSCRIVER_CHECK(has)  do {   \
    if(!has) return; \
} while(0)

void PercipioCameraNode::publishColorFrame(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = image_publishers_[COLOR_STREAM].getNumSubscribers() > 0;
    SUBSCRIVER_CHECK(has_subscriber);

    const cv::Mat& color = stream.getColorImage();
    if(color.empty()) {
        return;
    }

    auto image_info = stream.getColorInfo();
    image_info.header.stamp = HWTimeUsToROSTime(stream.getColorStramTimestamp());
    image_info.header.frame_id = camera_name_ + "_" + stream_name[COLOR_STREAM] + "_optical_frame";
    image_info.width = color.cols;
    image_info.height = color.rows;
    camera_info_publishers_[COLOR_STREAM]->publish(image_info);

    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC3, color).toImageMsg();
    image_msg->header.stamp = HWTimeUsToROSTime(stream.getDepthStramTimestamp());
    image_msg->is_bigendian = false;
    image_msg->step = 3 * color.cols;
    image_msg->header.frame_id = camera_name_ + "_" + stream_name[COLOR_STREAM] + "_optical_frame";
    image_publishers_[COLOR_STREAM].publish(std::move(image_msg));
}

void PercipioCameraNode::publishLeftIRFrame(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = image_publishers_[LEFT_IR_STREAM].getNumSubscribers() > 0;
    SUBSCRIVER_CHECK(has_subscriber);

    const cv::Mat& IR = stream.getLeftIRImage();
    if(IR.empty()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "ir image is empty.");
        return;
    }

    int type = IR.type();
    const char*   sz_encoding_type = nullptr;
    if(type == CV_8U)
        sz_encoding_type = sensor_msgs::image_encodings::MONO8;
    else if(type == CV_16U)
        sz_encoding_type = sensor_msgs::image_encodings::MONO16;
    else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Invalid ir image format.");
        return;
    }

    auto image_info = stream.getLeftIRInfo();
    image_info.header.stamp = HWTimeUsToROSTime(stream.getLeftIRStramTimestamp());
    image_info.header.frame_id = camera_name_ + "_" + stream_name[LEFT_IR_STREAM] + "_optical_frame";
    image_info.width = IR.cols;
    image_info.height = IR.rows;
    camera_info_publishers_[LEFT_IR_STREAM]->publish(image_info);

    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sz_encoding_type, IR).toImageMsg();
    image_msg->header.stamp = HWTimeUsToROSTime(stream.getDepthStramTimestamp());
    image_msg->is_bigendian = false;
    if(type == CV_8U)
        image_msg->step = IR.cols;
    else
        image_msg->step = 2 * IR.cols;
    image_msg->header.frame_id = camera_name_ + "_" + stream_name[LEFT_IR_STREAM] + "_optical_frame";
    image_publishers_[LEFT_IR_STREAM].publish(std::move(image_msg));
}

void PercipioCameraNode::publishRightIRFrame(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = image_publishers_[RIGHT_IR_STREAM].getNumSubscribers() > 0;
    SUBSCRIVER_CHECK(has_subscriber);

    const cv::Mat& IR = stream.getRightIRImage();
    if(IR.empty()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "ir image is empty.");
        return;
    }

    int type = IR.type();
    const char*   sz_encoding_type = nullptr;
    if(type == CV_8U)
        sz_encoding_type = sensor_msgs::image_encodings::MONO8;
    else if(type == CV_16U)
        sz_encoding_type = sensor_msgs::image_encodings::MONO16;
    else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("percipio_camera"), "Invalid ir image format.");
        return;
    }

    auto image_info = stream.getRightIRInfo();
    image_info.header.stamp = HWTimeUsToROSTime(stream.getLeftIRStramTimestamp());
    image_info.header.frame_id = camera_name_ + "_" + stream_name[RIGHT_IR_STREAM] + "_optical_frame";
    image_info.width = IR.cols;
    image_info.height = IR.rows;
    camera_info_publishers_[RIGHT_IR_STREAM]->publish(image_info);

    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sz_encoding_type, IR).toImageMsg();
    image_msg->header.stamp = HWTimeUsToROSTime(stream.getDepthStramTimestamp());
    image_msg->is_bigendian = false;
    if(type == CV_8U)
        image_msg->step = IR.cols;
    else
        image_msg->step = 2 * IR.cols;
    image_msg->header.frame_id = camera_name_ + "_" + stream_name[RIGHT_IR_STREAM] + "_optical_frame";
    image_publishers_[RIGHT_IR_STREAM].publish(std::move(image_msg));
}

void PercipioCameraNode::publishDepthFrame(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = image_publishers_[DEPTH_STREAM].getNumSubscribers() > 0;
    SUBSCRIVER_CHECK(has_subscriber);
    
    const cv::Mat& image = stream.getDepthImage();
    if(image.empty()) {
        return;
    }
            
    auto image_info = stream.getDepthInfo();
    image_info.header.stamp = HWTimeUsToROSTime(stream.getDepthStramTimestamp());
    image_info.header.frame_id = camera_name_ + "_" + stream_name[DEPTH_STREAM] + "_optical_frame";
    image_info.width = image.cols;
    image_info.height = image.rows;
    camera_info_publishers_[DEPTH_STREAM]->publish(image_info);

    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
    image_msg->header.stamp = HWTimeUsToROSTime(stream.getDepthStramTimestamp());
    image_msg->is_bigendian = false;
    image_msg->step = 2 * image.cols;
    image_msg->header.frame_id = camera_name_ + "_" + stream_name[DEPTH_STREAM] + "_optical_frame";
    image_publishers_[DEPTH_STREAM].publish(std::move(image_msg));
}

void PercipioCameraNode::publishColorPointCloud(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = color_point_cloud_pub_->get_subscription_count() > 0;
    SUBSCRIVER_CHECK(has_subscriber);
    
    const cv::Mat& p3d = stream.getPointCloud();
    const cv::Mat& color = stream.getColorImage();
    if(p3d.empty() || color.empty()) {
        return;
    }

    cv::Mat rszColor;
    cv::resize(color, rszColor, p3d.size());
    const auto *p3d_data = (float *)p3d.data;
    const auto *color_data = (uint8_t *)rszColor.data;
    auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    point_cloud_msg->width = p3d.cols;
    point_cloud_msg->height = p3d.rows;
    std::string format_str = "rgb";
    point_cloud_msg->point_step =
                    addPointField(*point_cloud_msg, format_str, 1, sensor_msgs::msg::PointField::FLOAT32,
                                  static_cast<int>(point_cloud_msg->point_step));
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
    
    size_t valid_count = 0;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud_msg, "b");
    for (int y = 0; y < p3d.rows; y++) {
        for (int x = 0; x < p3d.cols; x++) {
            bool valid_point = true;
            float depth = p3d_data[3 * y * p3d.cols + 3 * x + 2];
            if (std::isnan(depth)) {
                valid_point = false; 
            }
            
            if (valid_point) {
                *iter_x = p3d_data[3 * y * p3d.cols + 3 * x + 0] / 1000.0;
                *iter_y = p3d_data[3 * y * p3d.cols + 3 * x + 1] / 1000.0;
                *iter_z = depth / 1000.0;
                *iter_r = color_data[(3 * y * p3d.cols + 3 * x)  + 2];
                *iter_g = color_data[(3 * y * p3d.cols + 3 * x)  + 1];
                *iter_b = color_data[(3 * y * p3d.cols + 3 * x)  + 0];
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
    point_cloud_msg->width = valid_count;
    point_cloud_msg->height = 1;
    modifier.resize(valid_count);

    point_cloud_msg->header.stamp = HWTimeUsToROSTime(stream.getPointCloudStramTimestamp());
    point_cloud_msg->header.frame_id = camera_name_ + "_" + stream_name[DEPTH_STREAM] + "_optical_frame";;
    color_point_cloud_pub_->publish(std::move(point_cloud_msg));
}

void PercipioCameraNode::publishPointCloud(percipio_camera::VideoStream& stream)
{
    bool has_subscriber = point_cloud_pub_->get_subscription_count() > 0;
    SUBSCRIVER_CHECK(has_subscriber);

    const cv::Mat& p3d = stream.getPointCloud();
    if(p3d.empty()) {
        return;
    }

    const auto *p3d_data = (float *)p3d.data;
    auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    auto m_width = p3d.cols;
    auto m_height = p3d.rows;
    modifier.resize(m_width * m_height);
    point_cloud_msg->width = m_width;
    point_cloud_msg->height = m_height;
    point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
    point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);

    size_t valid_count = 0;    
    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            bool valid_point = true;
            float depth = p3d_data[3 * y * m_width + 3 * x + 2];
            if (std::isnan(depth)) {
                valid_point = false; 
            }

            if (valid_point) {
                *iter_x = p3d_data[3 * y * point_cloud_msg->width + 3 * x + 0] / 1000.0;
                *iter_y = p3d_data[3 * y * point_cloud_msg->width + 3 * x + 1] / 1000.0;
                *iter_z = depth / 1000.0;

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++valid_count;
            }
        }
    }

    point_cloud_msg->header.stamp = HWTimeUsToROSTime(stream.getPointCloudStramTimestamp());
    point_cloud_msg->header.frame_id = camera_name_ + "_" + stream_name[DEPTH_STREAM] + "_optical_frame";;
    point_cloud_pub_->publish(std::move(point_cloud_msg));
}

void PercipioCameraNode::onNewFrame(percipio_camera::VideoStream& stream) 
{
    if(stream_enable[DEPTH_STREAM]) {
        publishDepthFrame(stream);
    }

    if(stream_enable[COLOR_STREAM]) {
        publishColorFrame(stream);
    }

    if(stream_enable[LEFT_IR_STREAM]) {
        publishLeftIRFrame(stream);
    }

    if(stream_enable[RIGHT_IR_STREAM]) {
        publishRightIRFrame(stream);
    }

    if(color_point_cloud_enable) {
        publishColorPointCloud(stream);
    }
    
    if(point_cloud_enable) {
        publishPointCloud(stream);
    } 
}

}