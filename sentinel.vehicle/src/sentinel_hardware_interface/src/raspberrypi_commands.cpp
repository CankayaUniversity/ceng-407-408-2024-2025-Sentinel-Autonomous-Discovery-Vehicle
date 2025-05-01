#include "sentinel_hardware_interface/raspberrypi_commands.hpp"
#include <rclcpp/rclcpp.hpp>

namespace sentinel_hardware_interface
{

    void RaspberryPiCommands::connect(const std::string &serial_device, int32_t timeout_ms)
    {
        this->timeout_ms_ = timeout_ms;
        this->serial_conn_.Open(serial_device);
        this->serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
    }

    void RaspberryPiCommands::disconnect()
    {
        this->serial_conn_.Close();
    }

    bool RaspberryPiCommands::connected() const
    {
        return this->serial_conn_.IsOpen();
    }

    std::string RaspberryPiCommands::send_command(const std::string &command, bool print_output)
    {
        this->serial_conn_.FlushIOBuffers(); // Just in case
        this->serial_conn_.Write(command);
        std::string response = "";

        try
        {
            this->serial_conn_.ReadLine(response, '\n', this->timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "The ReadByte() call has timed out." << std::endl;
        }

        if (print_output)
        {
            RCLCPP_INFO(rclcpp::get_logger("SentinelHardwareInterface"), "Sent:%s Recv:%s .", command.c_str(), response.c_str());
        }
        return response;
    }

    void RaspberryPiCommands::send_empty_command()
    {
        std::string response = this->send_command("\r");
    }

    void RaspberryPiCommands::read_encoder_values(int &val1, int &val2)
    {
        std::string response = this->send_command("e\r");

        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());

        val1 = std::atoi(token_1.c_str());
        val2 = std::atoi(token_2.c_str());
    }

    void RaspberryPiCommands::set_motor_values(int left_speed, int right_speed)
    {
        std::stringstream ss;
        ss << "m " << left_speed << " " << right_speed << "\r";
        this->send_command(ss.str());
    }

    void RaspberryPiCommands::set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        this->send_command(ss.str());
    }
} // namespace sentinel_hardware_interface
