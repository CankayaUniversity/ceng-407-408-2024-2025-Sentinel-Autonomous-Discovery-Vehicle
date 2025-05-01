#include <cmath>

#include "skid_steer_controller/odometry.hpp"

namespace skid_steer_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
    : timestamp(0.0), x(0.0), y(0.0), yaw(0.0), linear(0.0), angular(0.0), wheel_separation(0.0),
      left_wheel_radius(0.0), right_wheel_radius(0.0), left_wheel_old_pos(0.0), right_wheel_old_pos(0.0),
      velocity_rolling_window_size(velocity_rolling_window_size), linear_accumulator(velocity_rolling_window_size),
      angular_accumulator(velocity_rolling_window_size)

{
}

Odometry::~Odometry()
{
}

void Odometry::init(const rclcpp::Time &time)
{
    this->resetAccumulators();
    this->timestamp = time;
}

bool Odometry::update(Wheels wheels_pos, const rclcpp::Time &time)
{
    const double left_average = (wheels_pos.left_front + wheels_pos.left_rear) / 2;
    const double right_average = (wheels_pos.right_front + wheels_pos.right_rear) / 2;

    const double left_wheel_est_vel = (left_average - this->left_wheel_old_pos) * this->left_wheel_radius;
    const double right_wheel_est_vel = (right_average - this->right_wheel_old_pos) * this->right_wheel_radius;

    this->left_wheel_old_pos = left_average;
    this->right_wheel_old_pos = right_average;

    this->updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time);
    return true;
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time &time)
{
    const double dt = time.seconds() - this->timestamp.seconds();

    const double linear = (left_vel + right_vel) * 0.5;
    const double angular = (right_vel - left_vel) / this->wheel_separation;

    this->integrateExact(linear, angular);
    this->timestamp = time;


    this->linear = linear / dt;
    this->angular = angular / dt;

    return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time &time)
{
    this->linear = linear;
    this->angular = angular;

    const double dt = time.seconds() - this->timestamp.seconds();
    this->timestamp = time;
    this->integrateExact(linear * dt, angular * dt);
}

void Odometry::resetOdometry()
{
    this->x = 0.0;
    this->y = 0.0;
    this->yaw = 0.0;
}

void Odometry::setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius)
{
    this->wheel_separation = wheel_separation;
    this->left_wheel_radius = left_wheel_radius;
    this->right_wheel_radius = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
    this->velocity_rolling_window_size = velocity_rolling_window_size;
    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
    (void)linear;
    (void)angular;
    // const double direction = this->heading + angular * 0.5;

    // this->x += linear * std::cos(direction);
    // this->y += linear * std::sin(direction);
    // this->heading += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
    this->yaw += angular;
    this->yaw = std::atan2(std::sin(this->yaw), std::cos(this->yaw));

    this->x += linear * std::cos(this->yaw);
    this->y += linear * std::sin(this->yaw);
}

void Odometry::resetAccumulators()
{
    this->linear_accumulator = RollingMeanAccumulator(this->velocity_rolling_window_size);
    this->angular_accumulator = RollingMeanAccumulator(this->velocity_rolling_window_size);
}

Wheels::Wheels(double left_front, double left_rear, double right_front, double right_rear)
{
    this->left_front = left_front;
    this->left_rear = left_rear;
    this->right_front = right_front;
    this->right_rear = right_rear;
}

Wheels::Wheels()
{
    this->left_front = 0.0;
    this->left_rear = 0.0;
    this->right_front = 0.0;
    this->right_rear = 0.0;
}
} // namespace skid_steer_controller
