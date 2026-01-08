#pragma once
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>

#include "TYApi.h"
#include "TYImageProc.h"
#include "percipio_camera_node.h"

namespace percipio_camera {

class PercipioCameraNodeDriver : public rclcpp::Node {
 public:
    PercipioCameraNodeDriver(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    PercipioCameraNodeDriver(const std::string& node_name, const std::string& ns,
                     const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
    ~PercipioCameraNodeDriver() override;

 private:
    void init();
    std::unique_ptr<PercipioCameraNode> percipio_camera_node_ = nullptr;
    std::shared_ptr<PercipioDevice> percipio_device = nullptr;

 private:
    rclcpp::Logger logger_;
    std::string device_serial_number_;
    std::string device_ip_;
    std::string device_workmode_;

    std::string device_config_xml_;

    bool device_log_enable_;
    std::string device_log_level_;
    int32_t device_log_server_port_;

    std::string device_model_name_;
    std::string device_buildhash_;
    std::string device_cfg_version_;

    void startDevice();
    bool initializeDevice(const TY_DEVICE_BASE_INFO& device);
    int tycam_log_server_init(bool enable, const std::string& level, int32_t port);
    void onCameraEventCallback(PercipioDevice* Handle, TY_EVENT_INFO *event_info);
};
}