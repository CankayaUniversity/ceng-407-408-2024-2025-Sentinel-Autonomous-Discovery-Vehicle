#ifndef SKID_STEER_CONTROLLER__ODOMETRY_HPP
#define SKID_STEER_CONTROLLER__ODOMETRY_HPP

#include <rclcpp/time.hpp>

#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include <rcppmath/rolling_mean_accumulator.hpp>
#endif

namespace skid_steer_controller
{

struct Wheels
{
  public:
    Wheels(double left_front, double left_rear, double right_front, double right_rear);
    Wheels();

  public:
    double left_front;
    double left_rear;
    double right_front;
    double right_rear;
};

class Odometry
{
  public:
    Odometry(size_t velocity_rolling_window_size = 10);
    ~Odometry();

    void init(const rclcpp::Time &time);
    bool update(Wheels wheels_pos, const rclcpp::Time &time);
    bool updateFromVelocity(double left_vel, double right_vel, const rclcpp::Time &time);
    void updateOpenLoop(double linear, double angular, const rclcpp::Time &time);
    void resetOdometry();
    void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

    double getX() const
    {
        return this->x;
    }
    double getY() const
    {
        return this->y;
    }
    double getHeading() const
    {
        return this->yaw;
    }
    double getLinear() const
    {
        return this->linear;
    }
    double getAngular() const
    {
        return this->angular;
    }

  private:
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
    using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
    using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);
    void resetAccumulators();

    rclcpp::Time timestamp;

    double x;   //   [m]
    double y;   //   [m]
    double yaw; // [rad]

    // Current velocity:
    double linear;  //   [m/s]
    double angular; // [rad/s]

    // Wheel kinematic parameters [m]:
    double wheel_separation;
    double left_wheel_radius;
    double right_wheel_radius;

    // Previous wheel position/state [rad]:
    double left_wheel_old_pos;
    double right_wheel_old_pos;

    size_t velocity_rolling_window_size;
    RollingMeanAccumulator linear_accumulator;
    RollingMeanAccumulator angular_accumulator;
};

} // namespace skid_steer_controller

#endif // SKID_STEER_CONTROLLER__ODOMETRY_HPP