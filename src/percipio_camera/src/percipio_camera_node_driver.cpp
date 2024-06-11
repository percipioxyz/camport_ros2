#include "percipio_camera_node_driver.h"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <csignal>
#include <sys/mman.h>

#include <vector>

#include "common.hpp"


namespace percipio_camera {
PercipioCameraNodeDriver::PercipioCameraNodeDriver(const rclcpp::NodeOptions &node_options)
    : Node("orbbec_camera_node", "/", node_options),
      logger_(this->get_logger()) {

  RCLCPP_INFO_STREAM(logger_, "Init lib");
  TYInitLib();

  TY_VERSION_INFO ver;
  TYLibVersion(&ver);
  RCLCPP_INFO_STREAM(logger_, "     - lib version: " << ver.major << "." << ver.minor << "." << ver.patch);

  init();
}

PercipioCameraNodeDriver::PercipioCameraNodeDriver(const std::string &node_name, const std::string &ns,
                                       const rclcpp::NodeOptions &node_options)
    : Node(node_name, ns, node_options),
      //ctx_(std::make_unique<ob::Context>()),
      logger_(this->get_logger()) {
  RCLCPP_INFO_STREAM(logger_, "Init lib");
  TYInitLib();

  TY_VERSION_INFO ver;
  TYLibVersion(&ver);
  RCLCPP_INFO_STREAM(logger_, "     - lib version: " << ver.major << "." << ver.minor << "." << ver.patch);
  init();
}

PercipioCameraNodeDriver::~PercipioCameraNodeDriver() {
  TYDeinitLib();
}

void PercipioCameraNodeDriver::init() {
    device_serial_number_   = declare_parameter<std::string>("serial_number", "");
    device_ip_              = declare_parameter<std::string>("device_ip", "");
    RCLCPP_INFO_STREAM(logger_, "PercipioCameraNodeDriver::init, deivce sn :" << device_serial_number_);
    RCLCPP_INFO_STREAM(logger_, "PercipioCameraNodeDriver::init, deivce ip :" << device_ip_);
    startDevice();
}

void PercipioCameraNodeDriver::startDevice() {
    TY_STATUS status;
    std::vector<TY_DEVICE_BASE_INFO> selected;
    while(1) {
        selected.clear();
        status = selectDevice(TY_INTERFACE_ALL, device_serial_number_, device_ip_, 1, selected);
        //RCLCPP_INFO_STREAM(logger_, "selectDevice  ret = " << status);
        //RCLCPP_INFO_STREAM(logger_, "selectDevice  devices cnt: " << selected.size());
        if(status != TY_STATUS_OK || selected.size() == 0) {
            RCLCPP_ERROR_STREAM(logger_, "Not found any device!");
            continue;
        }
        TY_DEVICE_BASE_INFO& selectedDev = selected[0];
        if(initializeDevice(selectedDev)) {
            break;
        }
    }
}

void PercipioCameraNodeDriver::onCameraEventCallback(PercipioDevice* Handle, TY_EVENT_INFO *event_info)
{
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        RCLCPP_ERROR_STREAM(logger_,  "Device Event Callback: Device Offline, SN = " << Handle->serialNumber());
    }
}

bool PercipioCameraNodeDriver::initializeDevice(const TY_DEVICE_BASE_INFO& device) {
    percipio_device = std::make_shared<PercipioDevice>(device.iface.id, device.id);
    if(!percipio_device->isAlive()) 
        return false;

    device_serial_number_ = percipio_device->serialNumber();
    device_model_name_ = percipio_device->modelName();
    device_buildhash_ = percipio_device->buildHash();
    device_cfg_version_ = percipio_device->configVersion();
    RCLCPP_INFO_STREAM(logger_, "Serial number:  " << device_serial_number_);
    RCLCPP_INFO_STREAM(logger_, "Model name:     " << device_model_name_);
    //RCLCPP_INFO_STREAM(logger_, "Build hash:     " << device_buildhash_);
    RCLCPP_INFO_STREAM(logger_, "Config version: " << device_cfg_version_);

    percipio_device->registerCameraEventCallback(boost::bind(&PercipioCameraNodeDriver::onCameraEventCallback, this, _1, _2));
    percipio_camera_node_ = std::make_unique<PercipioCameraNode>(this, percipio_device);
    return true;
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(percipio_camera::PercipioCameraNodeDriver)