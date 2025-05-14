#include "sentinel_hardware_interface/sentinel_hardware_interface.hpp"
#include <cstdint>

namespace sentinel_hardware_interface
{
  SentinelHardwareInterface::SentinelHardwareInterface() {}
  SentinelHardwareInterface::~SentinelHardwareInterface() {}

  hardware_interface::CallbackReturn SentinelHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::string left_front_wheel = info_.hardware_parameters["left_front_wheel"];
    std::string left_rear_wheel = info_.hardware_parameters["left_rear_wheel"];
    std::string right_front_wheel =
        info_.hardware_parameters["right_front_wheel"];
    std::string right_rear_wheel = info_.hardware_parameters["right_rear_wheel"];

    std::string device = info_.hardware_parameters["device"];
    int timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    int loop_rate = std::stoi(info_.hardware_parameters["loop_rate"]);

    int count_per_rev = std::stoi(info_.hardware_parameters["count_per_rev"]);
    int kp = std::stoi(info_.hardware_parameters["kp"]);
    int kd = std::stoi(info_.hardware_parameters["kd"]);
    int ki = std::stoi(info_.hardware_parameters["ki"]);
    int ko = std::stoi(info_.hardware_parameters["ko"]);

    this->left_wheels.emplace_back(left_front_wheel, count_per_rev);
    this->left_wheels.emplace_back(left_rear_wheel, count_per_rev);

    this->right_wheels.emplace_back(right_front_wheel, count_per_rev);
    this->right_wheels.emplace_back(right_rear_wheel, count_per_rev);

    this->raspberry_pi_commands.connect(device, timeout_ms);
    this->raspberry_pi_commands.set_pid_values(kp, kd, ki, ko);
    this->loop_rate = loop_rate;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  SentinelHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &left_wheel : this->left_wheels)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          left_wheel.name, hardware_interface::HW_IF_POSITION,
          &left_wheel.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          left_wheel.name, hardware_interface::HW_IF_VELOCITY,
          &left_wheel.velocity));
    }
    for (auto &right_wheel : this->right_wheels)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          right_wheel.name, hardware_interface::HW_IF_POSITION,
          &right_wheel.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          right_wheel.name, hardware_interface::HW_IF_VELOCITY,
          &right_wheel.velocity));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  SentinelHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto &left_wheel : this->left_wheels)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          left_wheel.name, hardware_interface::HW_IF_VELOCITY, &left_wheel.cmd));
    }
    for (auto &right_wheel : this->right_wheels)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          right_wheel.name, hardware_interface::HW_IF_VELOCITY,
          &right_wheel.cmd));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  SentinelHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"), "Configuring");
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"),
                "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SentinelHardwareInterface::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"), "Cleaning up");
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"),
                "Successfully cleaned up");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SentinelHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"), "Activating");
    // Check if necessary
    if (!this->raspberry_pi_commands.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"),
                "Successfully activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn SentinelHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"), "Deactivating");
    if (this->raspberry_pi_commands.connected())
    {
      this->raspberry_pi_commands.disconnect();
    }
    this->left_wheels.clear();
    this->right_wheels.clear();
    RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"),
                "Successfully deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type
  SentinelHardwareInterface::read(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration &period)
  {
    if (!this->raspberry_pi_commands.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // this->raspberry_pi_commands.read_encoder_values(
    //     this->right_wheels[0].encoder, this->left_wheels[0].encoder);
    this->raspberry_pi_commands.read_encoder_values(
        this->left_wheels[0].encoder, this->right_wheels[0].encoder);
    double delta_seconds = period.seconds();

    for (auto &wheel : this->left_wheels)
    {
      double pos_prev = wheel.position;
      wheel.encoder = this->left_wheels[0].encoder;
      wheel.position = wheel.calc_enc_angle();
      wheel.velocity = (wheel.position - pos_prev) / delta_seconds;
    }

    for (auto &wheel : this->right_wheels)
    {
      double pos_prev = wheel.position;
      wheel.encoder = this->right_wheels[0].encoder;
      wheel.position = wheel.calc_enc_angle();
      wheel.velocity = (wheel.position - pos_prev) / delta_seconds;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type
  SentinelHardwareInterface::write(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/)
  {

    float left_cmd = 0.0f;
    float right_cmd = 0.0f;

    for (const auto &left_wheel : this->left_wheels)
    {
      left_cmd += left_wheel.cmd;
    }
    left_cmd /= this->left_wheels.size();

    for (const auto &right_wheel : this->right_wheels)
    {
      right_cmd += right_wheel.cmd;
    }
    right_cmd /= this->right_wheels.size();

    int left_cmd_value = left_cmd / this->left_wheels[0].rads_per_count / this->loop_rate;
    int right_cmd_value = right_cmd / this->right_wheels[0].rads_per_count / this->loop_rate;

    this->raspberry_pi_commands.set_motor_values(
        static_cast<int>(left_cmd_value), static_cast<int>(right_cmd_value));

    return hardware_interface::return_type::OK;
  }

} // namespace sentinel_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sentinel_hardware_interface::SentinelHardwareInterface,
                       hardware_interface::SystemInterface)
