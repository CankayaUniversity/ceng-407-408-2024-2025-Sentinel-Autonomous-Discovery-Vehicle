#ifndef SKID_STEER_CONTROLLER__SKID_STEER_CONTROLLER_HPP
#define SKID_STEER_CONTROLLER__SKID_STEER_CONTROLLER_HPP

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "skid_steer_controller/odometry.hpp"
#include "skid_steer_controller/skid_steer_controller_parameters.hpp"
#include "skid_steer_controller/speed_limiter.hpp"

namespace skid_steer_controller
{
class SkidSteerController : public controller_interface::ChainableControllerInterface
{
    using TwistStamped = geometry_msgs::msg::TwistStamped;

  public:
    SkidSteerController();
    ~SkidSteerController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &time,
                                                                        const rclcpp::Duration &period) override;
    controller_interface::return_type update_and_write_commands(const rclcpp::Time &time,
                                                                const rclcpp::Duration &period) override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

  protected:
    const char *feedback_type() const;
    bool reset();
    void halt();
    bool on_set_chained_mode(bool chained_mode) override;

    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    struct WheelHandle
    {
        WheelHandle(std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback,
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity)
            : feedback{feedback}, velocity{velocity}
        {
        }
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
    };

    controller_interface::CallbackReturn configure_side(const std::string &wheel_name,
                                                        std::shared_ptr<WheelHandle> &handle);

    std::shared_ptr<WheelHandle> left_front;
    std::shared_ptr<WheelHandle> left_rear;
    std::shared_ptr<WheelHandle> right_front;
    std::shared_ptr<WheelHandle> right_rear;

    std::shared_ptr<ParamListener> param_listener;
    Params params;
    realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> received_velocity_msg_ptr{nullptr};

    rclcpp::Duration cmd_vel_timeout = rclcpp::Duration::from_seconds(0.5);
    rclcpp::Time previous_update_timestamp{0};
    Odometry odometry;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher{nullptr};
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher{nullptr};
    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher{nullptr};
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher{
        nullptr};

    bool subscriber_is_active = false;
    rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_subscriber = nullptr;

    std::queue<std::array<double, 2>> previous_two_commands;

    std::unique_ptr<SpeedLimiter> limiter_linear;
    std::unique_ptr<SpeedLimiter> limiter_angular;

    bool publish_limited_velocity = false;
    std::shared_ptr<rclcpp::Publisher<TwistStamped>> limited_velocity_publisher{nullptr};
    std::shared_ptr<realtime_tools::RealtimePublisher<TwistStamped>> realtime_limited_velocity_publisher{nullptr};

    // publish rate limiter
    double publish_rate = 50.0;
    rclcpp::Duration publish_period{rclcpp::Duration::from_nanoseconds(0)};
    rclcpp::Time previous_publish_timestamp{0, 0, RCL_CLOCK_UNINITIALIZED};

  private:
    void reset_buffers();
};

} // namespace skid_steer_controller

#endif // SKID_STEER_CONTROLLER__SKID_STEER_CONTROLLER_HPP
