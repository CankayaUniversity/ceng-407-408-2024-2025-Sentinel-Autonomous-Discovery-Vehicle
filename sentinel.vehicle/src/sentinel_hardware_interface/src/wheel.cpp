#include <cmath>
#include "sentinel_hardware_interface/wheel.hpp"

Wheel::Wheel(const std::string &name, int count_per_rev) : name{name}, encoder{0}, position{0}, real_position{0}, cmd{0}, velocity{0}, max_velocity{1.0}
{
    this->rads_per_count = (2 * M_PI) / count_per_rev;
}

double Wheel::calc_enc_angle()
{
    return this->encoder * rads_per_count;
}
