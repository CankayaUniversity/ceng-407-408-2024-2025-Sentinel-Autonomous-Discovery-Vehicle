#pragma once

#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <atomic>

class UdpSocket
{
public:
  UdpSocket(const std::string &vehicle_host, int vehicle_port, const std::string &computer_host, int computer_port, int chunk_size);
  ~UdpSocket();

public:
  inline bool is_open() { return UdpSocket::open; }
  static void quit_handler(int signal);

  void send(void *data, int size, int id);

private:
  void create();
  void attach();
  void configure_vehicle();
  void configure_computer();
  void bind();

private:
  std::string vehicle_host;
  int vehicle_port;

  std::string computer_host;
  int computer_port;

  int chunk_size;

private:
  struct sockaddr_in vehicle_addr;
  struct sockaddr_in computer_addr;

  static int server_socket;
  static std::atomic<bool> open;
};
