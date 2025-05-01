#ifndef SENTINEL_HARDWARE_INTERFACE__WHEEL
#define SENTINEL_HARDWARE_INTERFACE__WHEEL

#include <string>

class Wheel
{
public:
    Wheel(const std::string &name, int count_per_rev);
    ~Wheel() = default;
    double calc_enc_angle();

public:
    std::string name;
    int encoder;
    double position;
    double real_position;
    double cmd;
    double velocity;
    double rads_per_count;
    double max_velocity;
};

#endif // SENTINEL_HARDWARE_INTERFACE__WHEEL