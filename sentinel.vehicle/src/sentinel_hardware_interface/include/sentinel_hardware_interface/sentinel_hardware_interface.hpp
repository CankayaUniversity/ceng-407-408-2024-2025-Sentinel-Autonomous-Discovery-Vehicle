#ifndef SENTINEL_HARDWARE_INTERFACE__SENTINEL_HARDWARE_INTERFACE
#define SENTINEL_HARDWARE_INTERFACE__SENTINEL_HARDWARE_INTERFACE

#include "sentinel_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "sentinel_hardware_interface/wheel.hpp"
#include "sentinel_hardware_interface/raspberrypi_commands.hpp"

#include <vector>

namespace sentinel_hardware_interface
{
    class SentinelHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SentinelHardwareInterface)
        SentinelHardwareInterface();
        ~SentinelHardwareInterface();

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        SENTINEL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::vector<Wheel> left_wheels;
        std::vector<Wheel> right_wheels;
        RaspberryPiCommands raspberry_pi_commands;
        int loop_rate;
    };

}

#endif // SENTINEL_HARDWARE_INTERFACE__SENTINEL_HARDWARE_INTERFACE