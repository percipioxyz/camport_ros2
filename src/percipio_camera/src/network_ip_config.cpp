#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

#include "Utils.hpp"


int main(int argc, char** argv) {
    std::cout << "\n";
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::setw(45) << std::right << "PERCIPIO NETWORK CAMERA IP CONFIGURATION TOOL" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    std::cout << "\n[DEVICE SCAN]" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    std::vector<TY_DEVICE_BASE_INFO> network_cameras;
    


    return 0;
}