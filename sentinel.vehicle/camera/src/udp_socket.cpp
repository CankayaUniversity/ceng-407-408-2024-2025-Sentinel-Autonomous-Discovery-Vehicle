#include <csignal>
#include <iostream>

#include "udp_socket.hpp"

int UdpSocket::server_socket = -1;
std::atomic<bool> UdpSocket::open = false;

UdpSocket::UdpSocket(const std::string &vehicle_host, int vehicle_port, const std::string &computer_host, int computer_port)
    : vehicle_host{vehicle_host}, vehicle_port{vehicle_port}, computer_host{computer_host}, computer_port{computer_port}
{
    std::signal(SIGINT, &UdpSocket::quit_handler);
    this->create();
    this->attach();
    this->configure_vehicle();
    this->configure_computer();
    this->bind();
}

UdpSocket::~UdpSocket()
{
    if (UdpSocket::server_socket > -1)
    {
        close(UdpSocket::server_socket);
    }
}

void UdpSocket::create()
{
    UdpSocket::server_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (UdpSocket::server_socket <= 0)
    {
        throw std::runtime_error(std::string("Socket:") + std::strerror(errno));
    }
}

void UdpSocket::attach()
{
    int opt = 1;
    int err = setsockopt(UdpSocket::server_socket, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
    if (err)
    {
        throw std::runtime_error(std::string("Setsockopt:") + std::strerror(errno));
    }
}

void UdpSocket::configure_vehicle()
{
    std::memset(&this->vehicle_addr, 0, sizeof(this->vehicle_addr));
    this->vehicle_addr.sin_family = AF_INET;
    this->vehicle_addr.sin_port = htons(this->vehicle_port);
    this->vehicle_addr.sin_addr.s_addr = inet_addr(this->vehicle_host.c_str());
}

void UdpSocket::configure_computer()
{
    std::memset(&this->computer_addr, 0, sizeof(this->computer_addr));
    this->computer_addr.sin_family = AF_INET;
    this->computer_addr.sin_port = htons(this->computer_port);
    this->computer_addr.sin_addr.s_addr = inet_addr(this->computer_host.c_str());
}

void UdpSocket::bind()
{
    int addr_len = sizeof(this->vehicle_addr);
    int status = ::bind(UdpSocket::server_socket, (struct sockaddr *)&this->vehicle_addr, addr_len);
    if (status < 0)
    {
        throw std::runtime_error(std::string("Bind: ") + std::strerror(errno));
    }
    UdpSocket::open = true;
}

void UdpSocket::quit_handler(int signal)
{
    UdpSocket::open = false;
    shutdown(UdpSocket::server_socket, SHUT_RDWR);
}

void UdpSocket::send(void *data, int size)
{
    if (!UdpSocket::open)
    {
        throw std::runtime_error("Socket is closed");
    }

    size_t offset = 0;
    size_t chunk_size = 65500;

    while (offset < size)
    {
        size_t remaining_size = size - offset;
        size_t send_size = std::min(remaining_size, chunk_size);

        ssize_t bytes_sent = ::sendto(
            UdpSocket::server_socket,
            static_cast<char *>(data) + offset,
            send_size,
            0,
            (struct sockaddr *)&this->computer_addr,
            sizeof(this->computer_addr));

        if (bytes_sent < 0)
        {
            throw std::runtime_error(std::string("Send failed: ") + std::strerror(errno));
        }

        if (static_cast<size_t>(bytes_sent) < send_size)
        {
            throw std::runtime_error("Not all data was sent.");
        }
        offset += send_size;
    }
}