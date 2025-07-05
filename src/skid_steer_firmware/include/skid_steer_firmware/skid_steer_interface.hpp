#ifndef SKID_STEER_INTERFACE_HPP
#define SKID_STEER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <libserial/SerialPort.h>
#include <string>

namespace skid_steer_firmware{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class SkidsteerInterface : public hardware_interface::SystemInterface{
public:
    SkidsteerInterface();
    virtual ~SkidsteerInterface();

    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    LibSerial::SerialPort esp32S3;
    std::string port;
    std::vector<double> velocity_commands;
    std::vector<double> position_states;
    std::vector<double> velocity_states;
    rclcpp::Time last_run;

};
}
#endif

// #ifndef SKID_STEER_INTERFACE_HPP
// #define SKID_STEER_INTERFACE_HPP

// #include <rclcpp/rclcpp.hpp>
// #include <hardware_interface/system_interface.hpp>
// #include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
// #include <rclcpp_lifecycle/state.hpp>
// #include <vector>
// #include <libserial/SerialPort.h>
// #include <string>

// namespace skid_steer_firmware{
// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
// class SkidsteerInterface : public hardware_interface::SystemInterface{
// public:
//     SkidsteerInterface(); 
//     virtual ~SkidsteerInterface();

//     virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    
//     virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

//     virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

//     virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

//     virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//     virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
//     virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//     LibSerial::SerialPort esp32S3;
//     std::string port;
//     std::vector<double> velocity_commands;
//     std::vector<double> position_states;
//     std::vector<double> velocity_states;
//     rclcpp::Clock clock_;
//     rclcpp::Time last_run_;

// };
// }
// #endif