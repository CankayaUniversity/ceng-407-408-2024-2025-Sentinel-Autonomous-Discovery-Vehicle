#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <iostream>

#include "base64/base64.hpp"
#include "camera/udp_socket.hpp"

int UdpSocket::server_socket = -1;
std::atomic<bool> UdpSocket::open = false;

UdpSocket::UdpSocket(const std::string &vehicle_host, int vehicle_port, const std::string &computer_host, int computer_port, int chunk_size)
    : vehicle_host{vehicle_host}, vehicle_port{vehicle_port}, computer_host{computer_host}, computer_port{computer_port}, chunk_size{chunk_size}
{

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
  int err = setsockopt(UdpSocket::server_socket, SOL_SOCKET,
                       SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
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
  this->computer_addr.sin_port = htons(this->computer_port);
  if (inet_pton(AF_INET, this->computer_host.c_str(),
                &this->computer_addr.sin_addr) <= 0)
  {
    throw std::runtime_error("Invalid IP address format.");
  }
}

void UdpSocket::bind()
{
  int addr_len = sizeof(this->vehicle_addr);
  int status = ::bind(UdpSocket::server_socket,
                      (struct sockaddr *)&this->vehicle_addr, addr_len);
  if (status < 0)
  {
    throw std::runtime_error(std::string("Bind: ") + std::strerror(errno));
  }
  UdpSocket::open = true;
}

void UdpSocket::quit_handler(int)
{
  UdpSocket::open = false;
  shutdown(UdpSocket::server_socket, SHUT_RDWR);
}

void UdpSocket::send(void *data, int size, int id)
{
  if (!UdpSocket::open)
  {
    throw std::runtime_error("Socket is closed");
  }

  std::string base64_frame = base64::to_base64(
      std::string_view(static_cast<const char *>(data), size));

  size_t total_size = base64_frame.size();

  for (size_t offset = 0; offset < total_size; offset += this->chunk_size)
  {
    size_t send_size =
        std::min(static_cast<size_t>(this->chunk_size), total_size - offset);
    std::string_view chunk(base64_frame.data() + offset, send_size);

    std::string payload = "{"
                          "\"frame_id\":" +
                          std::to_string(id) +
                          ",\"total_size\":" + std::to_string(total_size) +
                          ",\"chunk_size\":" + std::to_string(send_size) +
                          ",\"data\":\"" + std::string(chunk) + "\"}";

    ssize_t bytes_sent =
        ::sendto(UdpSocket::server_socket, payload.data(), payload.size(), 0,
                 reinterpret_cast<struct sockaddr *>(&this->computer_addr),
                 sizeof(this->computer_addr));

    if (bytes_sent < 0)
    {
      throw std::runtime_error("Send failed: " +
                               std::string(std::strerror(errno)));
    }

    if (static_cast<size_t>(bytes_sent) < payload.size())
    {
      throw std::runtime_error(
          "Partial data sent. Expected: " + std::to_string(payload.size()) +
          ", Sent: " + std::to_string(bytes_sent));
    }
  }
}
