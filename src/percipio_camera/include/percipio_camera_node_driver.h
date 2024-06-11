#pragma once
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>

#include "TYApi.h"
#include "percipio_camera_node.h"

namespace percipio_camera {

class PercipioCameraNodeDriver : public rclcpp::Node {
 public:
  explicit PercipioCameraNodeDriver(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
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

    std::string device_model_name_;
    std::string device_buildhash_;
    std::string device_cfg_version_;

    void startDevice();
    bool initializeDevice(const TY_DEVICE_BASE_INFO& device);
    void onCameraEventCallback(PercipioDevice* Handle, TY_EVENT_INFO *event_info);
};
}