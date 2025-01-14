#pragma once

#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <atomic>

class UdpSocket
{
public:
  UdpSocket(const std::string &vehicle_host, int vehicle_port, const std::string &computer_host, int computer_port);
  ~UdpSocket();

public:
  inline bool is_open() { return UdpSocket::open; }

  void send(void *data, int size);

private:
  void create();
  void attach();
  void configure_vehicle();
  void configure_computer();
  void bind();

  static void quit_handler(int signal);

private:
  std::string vehicle_host;
  int vehicle_port;

  std::string computer_host;
  int computer_port;

private:
  struct sockaddr_in vehicle_addr;
  struct sockaddr_in computer_addr;

  static int server_socket;
  static std::atomic<bool> open;
};
