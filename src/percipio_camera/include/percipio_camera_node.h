#pragma once


#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <image_publisher/image_publisher.hpp>
#include <image_transport/publisher.hpp>

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
            );  // set and get parameter

        void getParameters();
        void setupDevices();
        void setupPublishers();
        void setupTopics();
        
    
    private:
        rclcpp::Node* node_ = nullptr;
        std::string camera_name_ = "percipio_camera";
        std::string camera_link_frame_id_;

        bool tf_published_ = false;
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tf = nullptr;

        const std::shared_ptr<PercipioDevice>& device_ptr;

        std::map<percipio_stream_index_pair, bool>          stream_enable;
        std::map<percipio_stream_index_pair, std::string>   stream_name;
        std::map<percipio_stream_index_pair, std::string>   stream_resolution;
        std::map<percipio_stream_index_pair, std::string>   stream_image_mode;

        std::map<percipio_stream_index_pair, std::string>   frame_id;
        std::map<percipio_stream_index_pair, std::string>   optical_frame_id;
        std::map<percipio_stream_index_pair, image_transport::Publisher> image_publishers_;
        std::map<percipio_stream_index_pair, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_point_cloud_pub_;
        
        rclcpp::TimerBase::SharedPtr timer_ = nullptr;
        void broadcast_timer_callback();

        std::map<percipio_stream_index_pair, std::string>   stream_qos;
        std::map<percipio_stream_index_pair, std::string>   camera_info_qos_;
        std::string point_cloud_qos;

        bool m_offline_auto_reconnection = false;

        bool point_cloud_enable = true;
        bool color_point_cloud_enable = false;

        bool depth_registration_enable = false;

        int m_laser_power = -1;

        //Tof camera parameters
        std::string tof_depth_quality = "";
        int m_tof_modulation_threshold = -1;
        int m_tof_jitter_threshold = -1;
        int m_tof_filter_threshold = -1;
        int m_tof_channel = -1;
        int m_tof_HDR_ratio = -1;

        float f_depth_scale = 1.f;

        std::vector<geometry_msgs::msg::TransformStamped> static_tf_msgs_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;

        void publishStaticTF(const rclcpp::Time &t, const tf2::Vector3 &trans,
                                   const tf2::Quaternion &q, const std::string &from,
                                   const std::string &to);
        void publishStaticTransforms();

        void startStreams();
        void onNewFrame(percipio_camera::VideoStream& stream);
        void publishColorFrame(percipio_camera::VideoStream& stream);
        void publishLeftIRFrame(percipio_camera::VideoStream& stream);
        void publishRightIRFrame(percipio_camera::VideoStream& stream);
        void publishDepthFrame(percipio_camera::VideoStream& stream);
        void publishColorPointCloud(percipio_camera::VideoStream& stream);
        void publishPointCloud(percipio_camera::VideoStream& stream);
};
}
