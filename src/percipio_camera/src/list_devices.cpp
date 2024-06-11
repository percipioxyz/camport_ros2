#include "common.hpp"

#include <rclcpp/rclcpp.hpp>

int main() {
    // Init lib
    ASSERT_OK(TYInitLib());
    TY_VERSION_INFO ver;
    ASSERT_OK(TYLibVersion(&ver));
    ASSERT_OK(TYUpdateInterfaceList());

    uint32_t n = 0;
    ASSERT_OK(TYGetInterfaceNumber(&n));
    if (n == 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("list_device"), "interface number incorrect");
        return TY_STATUS_ERROR;
    }

    std::vector<TY_INTERFACE_INFO> ifaces(n);
    ASSERT_OK(TYGetInterfaceList(&ifaces[0], n, &n));
    ASSERT(n == ifaces.size());
    std::vector<TY_INTERFACE_HANDLE> hIfaces;
    for (uint32_t i = 0; i < n; i++) {
        TY_INTERFACE_HANDLE hIface;
        ASSERT_OK(TYOpenInterface(ifaces[i].id, &hIface));
        hIfaces.push_back(hIface);
    }
    updateDevicesParallel(hIfaces);
    for (uint32_t i = 0; i < n; i++) {
        TY_INTERFACE_HANDLE hIface = hIfaces[i];

        uint32_t n = 0;
        TYGetDeviceNumber(hIface, &n);
        if (n == 0) continue;

        std::vector<TY_DEVICE_BASE_INFO> devs(n);
        TYGetDeviceList(hIface, &devs[0], n, &n);
        ASSERT(n == devs.size());
        for (uint32_t j = 0; j < n; j++) {
            if (TYIsNetworkInterface(devs[j].iface.type)) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "serial_number :" << devs[j].id);
                if (strlen(devs[j].userDefinedName) != 0) {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        vendor     :" << devs[j].userDefinedName);
                } else {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        vendor     :" << devs[j].vendorName);
                }
                RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        model      :" << devs[j].modelName);

                RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        device MAC :" << devs[j].netInfo.mac);
                RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        device IP  :" << devs[j].netInfo.ip);

            } else {
                TY_DEV_HANDLE handle;
                int32_t ret = TYOpenDevice(hIface, devs[j].id, &handle);
                if (ret == 0) {
                    TYGetDeviceInfo(handle, &devs[j]);
                    TYCloseDevice(handle);
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "serial_number :" << devs[j].id);

                    if (strlen(devs[j].userDefinedName) != 0) {
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        vendor     : " << devs[j].userDefinedName);
                    } else {
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        vendor     : " << devs[j].vendorName);
                    }
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("list_device"), "        model      : " << devs[j].modelName);
                }
            }
        }
        TYCloseInterface(hIface);
    }
    ASSERT_OK(TYDeinitLib());
}