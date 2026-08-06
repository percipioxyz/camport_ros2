#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "TYApi.h"
#include "percipio_device.h"

namespace percipio_camera {

class PercipioCameraNode {
    public:
        PercipioCameraNode(rclcpp::Node* node, std::shared_ptr<PercipioDevice>& device);

        ~PercipioCameraNode();

        template <class T>
            void setAndGetNodeParameter(
                        T& param, const std::string& param_name, const T& default_value,
                        const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
                        rcl_interfaces::msg::ParameterDescriptor()
            );

        void getParameters();
        bool setupDevices();
        void setupPublishers();
        void setupSubscribers();
        void setupTopics();

        void SendOfflineMsg(const char* sn);
        
        void SendConnectMsg(const char* sn);
        
        void SendTimeoutMsg(const char* sn);

        rclcpp::Node* Node() { return node_; }
        std::shared_ptr<PercipioDevice>  Device() const { return device_ptr_; }
        
    private:
        rclcpp::Node* node_ = nullptr;
        std::string camera_name_ = "percipio_camera";
        std::string camera_link_frame_id_;

        bool tf_published_ = false;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;

        const std::shared_ptr<PercipioDevice>& device_ptr_;

        std::map<percipio_stream_index_pair, bool>          stream_enable_;
        std::map<percipio_stream_index_pair, std::string>   stream_name_;
        std::map<percipio_stream_index_pair, std::string>   stream_resolution_;
        std::map<percipio_stream_index_pair, std::string>   stream_image_mode_;

        std::map<percipio_stream_index_pair, std::string>   frame_id_;
        std::map<percipio_stream_index_pair, std::string>   optical_frame_id_;
        std::map<percipio_stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
        std::map<percipio_stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_point_cloud_pub_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_event_publisher_;

        void topic_softtrigger_callback(const std_msgs::msg::String::SharedPtr msg) const;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_event_subscriber_;

        void topic_dynamic_config_callback(const std_msgs::msg::String::SharedPtr msg) const;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_event_subscriber_;

        void topic_device_reset_callback(const std_msgs::msg::Empty::SharedPtr msg) const;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr device_reset_event_subscriber_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;
        void broadcast_timer_callback();

        std::map<percipio_stream_index_pair, std::string>   stream_qos_;
        std::map<percipio_stream_index_pair, std::string>   camera_info_qos_;
        std::string point_cloud_qos_;

        bool offline_auto_reconnection_ = false;

        bool device_frame_rate_control_ = false;
        float device_frame_rate_ = 5.0;

        bool point_cloud_enable_ = true;
        bool color_point_cloud_enable_ = false;

        bool depth_registration_enable_ = false;

        bool depth_speckle_filter_enable_ = false;
        int  max_speckle_size_ = 150;
        int  max_speckle_diff_ = 64;
        float max_physical_size_ = 20.0;

        bool depth_time_domain_filter_enable_ = false;
        int  depth_time_domain_num_ = 3;

        ir_enhance_model ir_enhance_mode_ = IREnhanceOFF;
        int ir_enhancement_coefficient_ = 6;

        bool ir_undistortion_ = true;

        int laser_power_ = -1;

        int roi_[4];
        bool enable_roi_aec_ = false;

        float depth_scale_ = 1.f;

        std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;

        void onCameraEventCallback(PercipioDevice* Handle, TY_EVENT_INFO *event_info);

        void publishStaticTF(const rclcpp::Time &t, const tf2::Vector3 &trans,
                                   const tf2::Quaternion &q, const std::string &from,
                                   const std::string &to);
        void publishStaticTransforms();

        bool startStreams();
        void onNewFrame(percipio_camera::VideoStream& stream);
        void publishColorFrame(percipio_camera::VideoStream& stream);
        void publishIRFrame(percipio_camera::VideoStream& stream, const percipio_stream_index_pair& ir_stream);
        void publishDepthFrame(percipio_camera::VideoStream& stream);
        void publishColorPointCloud(percipio_camera::VideoStream& stream);
        void publishPointCloud(percipio_camera::VideoStream& stream);

        void publishImageMsg(const percipio_stream_index_pair& idx,
                             const void* data, int width, int height,
                             const char* encoding, int bytes_per_pixel,
                             uint64_t timestamp_us,
                             const sensor_msgs::msg::CameraInfo& info);
};


}
