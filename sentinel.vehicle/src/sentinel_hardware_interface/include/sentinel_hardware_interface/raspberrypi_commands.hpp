#ifndef SENTINEL_HARDWARE_INTERFACE__RASPBERRY_PI_COMMANDS_HPP
#define SENTINEL_HARDWARE_INTERFACE__RASPBERRY_PI_COMMANDS_HPP

#include <sstream>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <libserial/SerialPort.h>

namespace sentinel_hardware_interface
{
    class RaspberryPiCommands
    {

    public:
        RaspberryPiCommands() = default;
        void connect(const std::string &serial_device, int32_t timeout_ms);
        void disconnect();
        bool connected() const;
        std::string send_command(const std::string &command, bool print_output = true);
        void send_empty_command();
        void read_encoder_values(int &val1, int &val2);
        void set_motor_values(int speed_left, int speed_right);
        void set_pid_values(int k_p, int k_d, int k_i, int k_o);

    private:
        int timeout_ms_;
        LibSerial::SerialPort serial_conn_;
    };

} // namespace sentinel_hardware_interface

#endif