#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "skid_steer_controller/skid_steer_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace skid_steer_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

SkidSteerController::SkidSteerController()
{
}

SkidSteerController::~SkidSteerController()
{
}

controller_interface::CallbackReturn SkidSteerController::on_init()
{
    try
    {
        this->param_listener = std::make_shared<ParamListener>(get_node());
        this->params = this->param_listener->get_params();
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration SkidSteerController::command_interface_configuration() const
{
    std::vector<std::string> conf_names;
    for (const auto &joint_name : this->params.left_wheel_names)
    {
        conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }
    for (const auto &joint_name : this->params.right_wheel_names)
    {
        conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration skid_steer_controller::SkidSteerController::state_interface_configuration() const
{
    std::vector<std::string> conf_names;
    for (const auto &joint_name : this->params.left_wheel_names)
    {
        conf_names.push_back(joint_name + "/" + feedback_type());
    }
    for (const auto &joint_name : this->params.right_wheel_names)
    {
        conf_names.push_back(joint_name + "/" + feedback_type());
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type skid_steer_controller::SkidSteerController::update_reference_from_subscribers(
    const rclcpp::Time &time, const rclcpp::Duration &)
{
    auto logger = get_node()->get_logger();
    const std::shared_ptr<TwistStamped> command_msg_ptr = *(this->received_velocity_msg_ptr.readFromRT());

    if (command_msg_ptr == nullptr)
    {
        RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - command_msg_ptr->header.stamp;

    if (age_of_last_command > this->cmd_vel_timeout)
    {
        this->reference_interfaces_[0] = 0.0;
        this->reference_interfaces_[1] = 0.0;
    }
    else if (std::isfinite(command_msg_ptr->twist.linear.x) && std::isfinite(command_msg_ptr->twist.angular.z))
    {
        this->reference_interfaces_[0] = command_msg_ptr->twist.linear.x;
        this->reference_interfaces_[1] = command_msg_ptr->twist.angular.z;
    }
    else
    {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(logger, *get_node()->get_clock(), this->cmd_vel_timeout.seconds() * 1000,
                                       "Command message contains NaNs. Not updating reference interfaces.");
    }
    this->previous_update_timestamp = time;
    return controller_interface::return_type::OK;
}

controller_interface::return_type skid_steer_controller::SkidSteerController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    auto logger = get_node()->get_logger();

    double linear_command = this->reference_interfaces_[0];
    double angular_command = this->reference_interfaces_[1];

    if (!std::isfinite(linear_command) || !std::isfinite(angular_command))
    {
        return controller_interface::return_type::OK;
    }

    const double wheel_separation = this->params.wheel_separation;
    const double left_wheel_radius = this->params.wheel_radius;
    const double right_wheel_radius = this->params.wheel_radius;

    if (this->params.open_loop) // open_loop
    {
        odometry.updateOpenLoop(linear_command, angular_command, time);
    }
    else
    {
        const auto left_front_op = this->left_front->feedback.get().get_optional();
        const auto left_rear_op = this->left_rear->feedback.get().get_optional();
        const auto right_front_op = this->right_front->feedback.get().get_optional();
        const auto right_rear_op = this->right_rear->feedback.get().get_optional();

        if (!left_front_op.has_value() || !left_rear_op.has_value() || !right_front_op.has_value() ||
            !right_rear_op.has_value())
        {
            RCLCPP_DEBUG(logger, "Unable to retrieve the data from the left or right wheels feedback!");
            return controller_interface::return_type::OK;
        }

        const double left_front_val = left_front_op.value();
        const double left_rear_val = left_rear_op.value();
        const double right_front_val = right_front_op.value();
        const double right_rear_val = right_rear_op.value();

        if (std::isnan(left_front_val) || std::isnan(left_rear_val) || std::isnan(right_front_val) ||
            std::isnan(right_rear_val))
        {
            RCLCPP_ERROR(logger, "Either the left or right wheel %s is invalid", feedback_type());
            return controller_interface::return_type::ERROR;
        }

        this->odometry.update(Wheels{left_front_val, left_rear_val, right_front_val, right_rear_val}, time);
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry.getHeading());

    bool should_publish = false;
    try
    {
        if (this->previous_publish_timestamp + this->publish_period < time)
        {
            this->previous_publish_timestamp += this->publish_period;
            should_publish = true;
        }
    }
    catch (const std::runtime_error &)
    {
        // Handle exceptions when the time source changes and initialize publish timestamp
        this->previous_publish_timestamp = time;
        should_publish = true;
    }

    if (should_publish)
    {
        if (this->realtime_odometry_publisher->trylock())
        {
            auto &odometry_message = this->realtime_odometry_publisher->msg_;
            odometry_message.header.stamp = time;
            odometry_message.pose.pose.position.x = this->odometry.getX();
            odometry_message.pose.pose.position.y = this->odometry.getY();
            odometry_message.pose.pose.orientation.x = orientation.x();
            odometry_message.pose.pose.orientation.y = orientation.y();
            odometry_message.pose.pose.orientation.z = orientation.z();
            odometry_message.pose.pose.orientation.w = orientation.w();
            odometry_message.twist.twist.linear.x = this->odometry.getLinear();
            odometry_message.twist.twist.angular.z = this->odometry.getAngular();
            this->realtime_odometry_publisher->unlockAndPublish();
        }

        if (this->params.enable_odom_tf && this->realtime_odometry_transform_publisher->trylock())
        {
            auto &transform = realtime_odometry_transform_publisher->msg_.transforms.front();
            transform.header.stamp = time;
            transform.transform.translation.x = this->odometry.getX();
            transform.transform.translation.y = this->odometry.getY();
            transform.transform.rotation.x = orientation.x();
            transform.transform.rotation.y = orientation.y();
            transform.transform.rotation.z = orientation.z();
            transform.transform.rotation.w = orientation.w();
            this->realtime_odometry_transform_publisher->unlockAndPublish();
        }
    }

    double &last_linear = this->previous_two_commands.back()[0];
    double &second_to_last_linear = this->previous_two_commands.front()[0];
    double &last_angular = this->previous_two_commands.back()[1];
    double &second_to_last_angular = this->previous_two_commands.front()[1];

    this->limiter_linear->limit(linear_command, last_linear, second_to_last_linear, period.seconds());
    this->limiter_angular->limit(angular_command, last_angular, second_to_last_angular, period.seconds());
    this->previous_two_commands.pop();
    this->previous_two_commands.push({{linear_command, angular_command}});

    if (this->publish_limited_velocity && this->realtime_limited_velocity_publisher->trylock())
    {
        auto &limited_velocity_command = this->realtime_limited_velocity_publisher->msg_;
        limited_velocity_command.header.stamp = time;
        limited_velocity_command.twist.linear.x = linear_command;
        limited_velocity_command.twist.linear.y = 0.0;
        limited_velocity_command.twist.linear.z = 0.0;
        limited_velocity_command.twist.angular.x = 0.0;
        limited_velocity_command.twist.angular.y = 0.0;
        limited_velocity_command.twist.angular.z = angular_command;
        this->realtime_limited_velocity_publisher->unlockAndPublish();
    }

    const double velocity_left = (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
    const double velocity_right = (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

    bool set_command_result = true;

    set_command_result &= this->left_front->velocity.get().set_value(velocity_left);
    set_command_result &= this->left_rear->velocity.get().set_value(velocity_left);

    set_command_result &= this->right_front->velocity.get().set_value(velocity_right);
    set_command_result &= this->right_rear->velocity.get().set_value(velocity_right);

    RCLCPP_DEBUG_EXPRESSION(logger, !set_command_result, "Unable to set the command to one of the command handles!");

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn skid_steer_controller::SkidSteerController::on_configure(
    const rclcpp_lifecycle::State &)
{
    auto logger = this->get_node()->get_logger();
    if (this->param_listener->is_old(this->params))
    {
        this->params = this->param_listener->get_params();
        RCLCPP_INFO(logger, "Parameters were updated");
    }

    if (this->params.left_wheel_names.size() != this->params.right_wheel_names.size())
    {
        RCLCPP_ERROR(logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
                     this->params.left_wheel_names.size(), this->params.right_wheel_names.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    const double wheel_separation = this->params.wheel_separation;
    const double left_wheel_radius = this->params.wheel_radius;
    const double right_wheel_radius = this->params.wheel_radius;

    this->odometry.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
    this->odometry.setVelocityRollingWindowSize(static_cast<size_t>(this->params.velocity_rolling_window_size));

    this->cmd_vel_timeout = rclcpp::Duration::from_seconds(this->params.cmd_vel_timeout);
    this->publish_limited_velocity = this->params.publish_limited_velocity;

    // Allocate reference interfaces if needed
    const int nr_ref_itfs = 2;
    this->reference_interfaces_.resize(nr_ref_itfs, std::numeric_limits<double>::quiet_NaN());

    if (!this->params.linear.x.has_velocity_limits)
    {
        RCLCPP_WARN(logger,
                    "[deprecated] has_velocity_limits parameter is deprecated, instead set the respective limits "
                    "to NAN");
        this->params.linear.x.min_velocity = this->params.linear.x.max_velocity =
            std::numeric_limits<double>::quiet_NaN();
    }
    if (!this->params.linear.x.has_acceleration_limits)
    {
        RCLCPP_WARN(logger, "[deprecated] has_acceleration_limits parameter is deprecated, instead set the respective "
                            "limits to "
                            "NAN");
        this->params.linear.x.max_deceleration = this->params.linear.x.max_acceleration =
            this->params.linear.x.max_deceleration_reverse = this->params.linear.x.max_acceleration_reverse =
                std::numeric_limits<double>::quiet_NaN();
    }
    if (!this->params.linear.x.has_jerk_limits)
    {
        RCLCPP_WARN(logger,
                    "[deprecated] has_jerk_limits parameter is deprecated, instead set the respective limits to "
                    "NAN");
        this->params.linear.x.min_jerk = this->params.linear.x.max_jerk = std::numeric_limits<double>::quiet_NaN();
    }
    if (!this->params.angular.z.has_velocity_limits)
    {
        RCLCPP_WARN(logger,
                    "[deprecated] has_velocity_limits parameter is deprecated, instead set the respective limits "
                    "to NAN");
        this->params.angular.z.min_velocity = this->params.angular.z.max_velocity =
            std::numeric_limits<double>::quiet_NaN();
    }
    if (!this->params.angular.z.has_acceleration_limits)
    {
        RCLCPP_WARN(logger, "[deprecated] has_acceleration_limits parameter is deprecated, instead set the respective "
                            "limits to "
                            "NAN");
        this->params.angular.z.max_deceleration = this->params.angular.z.max_acceleration =
            this->params.angular.z.max_deceleration_reverse = this->params.angular.z.max_acceleration_reverse =
                std::numeric_limits<double>::quiet_NaN();
    }
    if (!this->params.angular.z.has_jerk_limits)
    {
        RCLCPP_WARN(logger,
                    "[deprecated] has_jerk_limits parameter is deprecated, instead set the respective limits to "
                    "NAN");
        this->params.angular.z.min_jerk = this->params.angular.z.max_jerk = std::numeric_limits<double>::quiet_NaN();
    }
    // END DEPRECATED
    this->limiter_linear = std::make_unique<SpeedLimiter>(
        this->params.linear.x.min_velocity, this->params.linear.x.max_velocity,
        this->params.linear.x.max_acceleration_reverse, this->params.linear.x.max_acceleration,
        this->params.linear.x.max_deceleration, this->params.linear.x.max_deceleration_reverse,
        this->params.linear.x.min_jerk, this->params.linear.x.max_jerk);

    this->limiter_angular = std::make_unique<SpeedLimiter>(
        this->params.angular.z.min_velocity, this->params.angular.z.max_velocity,
        this->params.angular.z.max_acceleration_reverse, this->params.angular.z.max_acceleration,
        this->params.angular.z.max_deceleration, this->params.angular.z.max_deceleration_reverse,
        this->params.angular.z.min_jerk, this->params.angular.z.max_jerk);

    if (!this->reset())
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (this->publish_limited_velocity)
    {
        this->limited_velocity_publisher =
            get_node()->create_publisher<TwistStamped>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
        this->realtime_limited_velocity_publisher =
            std::make_shared<realtime_tools::RealtimePublisher<TwistStamped>>(limited_velocity_publisher);
    }

    velocity_command_subscriber = this->get_node()->create_subscription<TwistStamped>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<TwistStamped> msg) -> void {
            if (!this->subscriber_is_active)
            {
                RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                return;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
            {
                RCLCPP_WARN_ONCE(get_node()->get_logger(),
                                 "Received TwistStamped with zero timestamp, setting it to current "
                                 "time, this message will only be shown once");
                msg->header.stamp = get_node()->now();
            }

            const auto current_time_diff = this->get_node()->now() - msg->header.stamp;

            if (this->cmd_vel_timeout == rclcpp::Duration::from_seconds(0.0) ||
                current_time_diff < this->cmd_vel_timeout)
            {
                this->received_velocity_msg_ptr.writeFromNonRT(msg);
            }
            else
            {
                RCLCPP_WARN(get_node()->get_logger(),
                            "Ignoring the received message (timestamp %.10f) because it is older than "
                            "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
                            rclcpp::Time(msg->header.stamp).seconds(), current_time_diff.seconds(),
                            this->cmd_vel_timeout.seconds());
            }
        });

    this->odometry_publisher =
        get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
    this->realtime_odometry_publisher =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(this->odometry_publisher);

    std::string tf_prefix = "";
    if (this->params.tf_frame_prefix_enable)
    {
        if (this->params.tf_frame_prefix != "")
        {
            tf_prefix = this->params.tf_frame_prefix;
        }
        else
        {
            tf_prefix = std::string(get_node()->get_namespace());
        }

        // Make sure prefix does not start with '/' and always ends with '/'
        if (tf_prefix.back() != '/')
        {
            tf_prefix = tf_prefix + "/";
        }
        if (tf_prefix.front() == '/')
        {
            tf_prefix.erase(0, 1);
        }
    }

    const auto odom_frame_id = tf_prefix + this->params.odom_frame_id;
    const auto base_frame_id = tf_prefix + this->params.base_frame_id;

    auto &odometry_message = this->realtime_odometry_publisher->msg_;
    odometry_message.header.frame_id = odom_frame_id;
    odometry_message.child_frame_id = base_frame_id;

    // limit the publication on the topics /odom and /tf
    this->publish_rate = params.publish_rate;
    this->publish_period = rclcpp::Duration::from_seconds(1.0 / this->publish_rate);

    odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;
        odometry_message.pose.covariance[diagonal_index] = this->params.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = this->params.twist_covariance_diagonal[index];
    }

    // initialize transform publisher and message
    this->odometry_transform_publisher =
        get_node()->create_publisher<tf2_msgs::msg::TFMessage>(DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    this->realtime_odometry_transform_publisher =
        std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            this->odometry_transform_publisher);

    // keeping track of odom and base_link transforms only
    auto &odometry_transform_message = this->realtime_odometry_transform_publisher->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

    this->previous_update_timestamp = this->get_node()->get_clock()->now();
    RCLCPP_INFO(get_node()->get_logger(), "Configuration completed successfully");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn skid_steer_controller::SkidSteerController::on_activate(
    const rclcpp_lifecycle::State &)
{
    const auto left_front_r = this->configure_side("left_front", this->left_front);
    const auto left_rear_r = this->configure_side("left_rear", this->left_rear);
    const auto right_front_r = this->configure_side("right_front", this->right_front);
    const auto right_rear_r = this->configure_side("right_rear", this->right_rear);

    if (left_front_r == controller_interface::CallbackReturn::ERROR ||
        left_rear_r == controller_interface::CallbackReturn::ERROR ||
        right_front_r == controller_interface::CallbackReturn::ERROR ||
        right_rear_r == controller_interface::CallbackReturn::ERROR)
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    this->subscriber_is_active = true;
    RCLCPP_INFO(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn skid_steer_controller::SkidSteerController::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    this->subscriber_is_active = false;
    this->halt();
    this->reset_buffers();
    this->left_front.reset();
    this->left_rear.reset();
    this->right_front.reset();
    this->right_rear.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn skid_steer_controller::SkidSteerController::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    if (!this->reset())
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn skid_steer_controller::SkidSteerController::on_error(
    const rclcpp_lifecycle::State &)
{
    if (!reset())
    {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

const char *SkidSteerController::feedback_type() const
{
    return params.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

bool SkidSteerController::reset()
{
    this->odometry.resetOdometry();

    this->reset_buffers();

    this->left_front.reset();
    this->left_rear.reset();
    this->right_front.reset();
    this->right_rear.reset();

    this->subscriber_is_active = false;
    this->velocity_command_subscriber.reset();

    return true;
}

void SkidSteerController::halt()
{
    bool r;
    r = left_front->velocity.get().set_value(0.0);
    r = left_rear->velocity.get().set_value(0.0);
    r = right_front->velocity.get().set_value(0.0);
    r = right_rear->velocity.get().set_value(0.0);
    (void)r;
}

bool SkidSteerController::on_set_chained_mode(bool chained_mode)
{
    return true || chained_mode;
}

std::vector<hardware_interface::CommandInterface> SkidSteerController::on_export_reference_interfaces()
{
    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_interfaces_.size());

    reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name() + std::string("/linear"),
                                                                        hardware_interface::HW_IF_VELOCITY,
                                                                        &reference_interfaces_[0]));

    reference_interfaces.push_back(
        hardware_interface::CommandInterface(get_node()->get_name() + std::string("/angular"),
                                             hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[1]));
    return reference_interfaces;
}

controller_interface::CallbackReturn SkidSteerController::configure_side(const std::string &wheel_n,
                                                                         std::shared_ptr<WheelHandle> &handle)
{
    std::string wheel_name = wheel_n + "_wheel_joint";

    auto logger = this->get_node()->get_logger();
    const auto interface_name = this->feedback_type();

    const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
                                           [&logger, &wheel_name, &interface_name](const auto &interface) {
                                               return interface.get_prefix_name() == wheel_name &&
                                                      interface.get_interface_name() == interface_name;
                                           });

    if (state_handle == this->state_interfaces_.cend())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_name](const auto &interface) {
            return interface.get_prefix_name() == wheel_name && interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (command_handle == command_interfaces_.end())
    {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    handle = std::make_shared<WheelHandle>(std::ref(*state_handle), std::ref(*command_handle));
    return controller_interface::CallbackReturn::SUCCESS;
}
void SkidSteerController::reset_buffers()
{
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), std::numeric_limits<double>::quiet_NaN());
    // Empty out the old queue. Fill with zeros (not NaN) to catch early accelerations.
    std::queue<std::array<double, 2>> empty;
    std::swap(this->previous_two_commands, empty);
    this->previous_two_commands.push({{0.0, 0.0}});
    this->previous_two_commands.push({{0.0, 0.0}});

    // Fill RealtimeBuffer with NaNs so it will contain a known value
    // but still indicate that no command has yet been sent.
    this->received_velocity_msg_ptr.reset();
    std::shared_ptr<TwistStamped> empty_msg_ptr = std::make_shared<TwistStamped>();
    empty_msg_ptr->header.stamp = get_node()->now();
    empty_msg_ptr->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
    empty_msg_ptr->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
    empty_msg_ptr->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    empty_msg_ptr->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    empty_msg_ptr->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    empty_msg_ptr->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    this->received_velocity_msg_ptr.writeFromNonRT(empty_msg_ptr);
}

} // namespace skid_steer_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(skid_steer_controller::SkidSteerController,
                            controller_interface::ChainableControllerInterface)