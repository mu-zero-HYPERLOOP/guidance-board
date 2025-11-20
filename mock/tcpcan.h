#pragma once

#include <arpa/inet.h>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

struct CanFrame {
  uint64_t data;
  uint32_t can_id;
  uint8_t dlc;
  explicit CanFrame(uint32_t can_id, uint64_t data, uint8_t dlc)
      : data(data), can_id(can_id), dlc(dlc) {}
};

template <uint8_t BUS_COUNT> class TcpCan {
private:
  struct NetworkFrame;

public:
  static std::unique_ptr<TcpCan> connect() {

    int tcp_socket;
    struct sockaddr_in broadcast_addr;

    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket == -1) {
      perror("Can't create UDP socket");
      exit(-1);
    }
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(0);
    addr.sin_addr.s_addr = INADDR_ANY;

    int enableBroadcast = 1;
    if (setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &enableBroadcast,
                   sizeof(enableBroadcast))) {
      perror("Failed to set SO_BROADCAST option");
    }

    if (bind(udp_socket, (struct sockaddr *)&addr, sizeof(addr))) {
      perror("Failed to bind socket");
      exit(-1);
    }

    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(9003);
    broadcast_addr.sin_addr.s_addr = INADDR_BROADCAST;

    uint8_t udp_datagram[216];
    udp_datagram[0] = 0x0;
    char service_name[] = "CANzero";
    udp_datagram[24] = sizeof(service_name) - 1;
    memcpy(udp_datagram + 25, service_name, sizeof(service_name));

    std::cout << "Searching for Server" << std::endl;
    ssize_t send_bytes =
        sendto(udp_socket, udp_datagram, sizeof(udp_datagram), 0,
               (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
    if (send_bytes < 0) {
      perror("Failed to send on UDP socket");
      exit(-1);
    }

    struct sockaddr_in server_udp_socket_addr;
    socklen_t addr_len = sizeof(server_udp_socket_addr);
    memset(&server_udp_socket_addr, 0, sizeof(server_udp_socket_addr));
    ssize_t recv_bytes =
        recvfrom(udp_socket, udp_datagram, sizeof(udp_datagram), MSG_WAITALL,
                 (struct sockaddr *)&server_udp_socket_addr, &addr_len);
    if (recv_bytes < 0) {
      perror("Failed to recv on UDP socket");
      exit(-1);
    }

    if (close(udp_socket)) {
      perror("Failed to close UDP socket");
      exit(-1);
    }

    assert(udp_datagram[0] == 0x1);
    struct NetworkDescription {
      uint16_t service_port;
      uint64_t config_hash;
      uint64_t time_since_sor_us;
      std::string service_name;
      std::string build_time;
      std::string server_name;
    };

    uint8_t service_name_len = udp_datagram[24];
    uint8_t build_time_len = udp_datagram[88];
    uint8_t server_name_len = udp_datagram[152];
    NetworkDescription nd = {
        .service_port = reinterpret_cast<uint16_t *>(udp_datagram)[1],
        .config_hash = reinterpret_cast<uint64_t *>(udp_datagram)[1],
        .time_since_sor_us = reinterpret_cast<uint64_t *>(udp_datagram)[2],
        .service_name = std::string(reinterpret_cast<char *>(udp_datagram) + 25,
                                    service_name_len),
        .build_time = std::string(reinterpret_cast<char *>(udp_datagram) + 89,
                                  build_time_len),
        .server_name = std::string(reinterpret_cast<char *>(udp_datagram) + 153,
                                   server_name_len),
    };

    std::chrono::time_point<std::chrono::high_resolution_clock> sor =
        std::chrono::high_resolution_clock::now() -
        std::chrono::duration<unsigned long long, std::micro>(
            nd.time_since_sor_us);

    char server_ip_addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &server_udp_socket_addr.sin_addr, server_ip_addr_str,
              INET_ADDRSTRLEN);

    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket == -1) {
      perror("Failed to create TCP socket");
      exit(-1);
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = server_udp_socket_addr.sin_addr.s_addr;
    server_addr.sin_port = htons(nd.service_port);

    if (::connect(tcp_socket, reinterpret_cast<sockaddr *>(&server_addr),
                  sizeof(server_addr))) {
      perror("Failed to connect to TCP server");
      exit(-1);
    }

    uint8_t connection_handshake[2];
    memset(&connection_handshake, 0, sizeof(connection_handshake));

    ssize_t tx_bytes = ::send(tcp_socket, &connection_handshake,
                              sizeof(connection_handshake), MSG_WAITALL);
    assert(tx_bytes == 2);

    /* std::cout << (uint32_t)connection_handshake[0] << "|" <<
     * (uint32_t)connection_handshake[1] << std::endl; */

    // NOTE watchdogs are igored, which is fine for now
    // on the server side they only get argmed when the first
    // keep alive message arrives if we never send one we can't
    // miss the wdg. Crude, but great for development
    // We should however expect to receive them!

    std::cout << "\033[0;32m" << "Successfully connected to " << nd.server_name << " at " << server_ip_addr_str << ":" << nd.service_port << "\033[0m"<< std::endl;


    return std::make_unique<TcpCan>(tcp_socket, sor, broadcast_addr);
  }

  ~TcpCan() {
    if (close(m_socket)) {
      perror("Failed to close UDP socket");
      exit(-1);
    }
  }

  template <uint8_t BUS_ID> void send(const CanFrame &frame) {
    sock_send(NetworkFrame(frame, BUS_ID));
  }

  template <uint8_t BUS_ID> std::optional<CanFrame> recv() {
    return m_sync_buffers[BUS_ID].dequeue();
  }

private:
  void sock_send(const NetworkFrame &frame) {
    uint8_t tx_buf[24];
    tx_buf[0] = 0x1;
    tx_buf[1] = frame.bus_id;
    tx_buf[2] = frame.can_frame.dlc;
    reinterpret_cast<uint32_t *>(tx_buf)[1] = frame.can_frame.can_id;
    uint64_t timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - m_sor)
            .count();
    reinterpret_cast<uint64_t *>(tx_buf)[1] = timestamp_us;
    reinterpret_cast<uint64_t *>(tx_buf)[2] = frame.can_frame.data;

    size_t send = 0;
    while (send != sizeof(tx_buf)) {
      ssize_t send_bytes =
          ::send(m_socket, tx_buf + send, sizeof(tx_buf) - send, MSG_WAITALL);
      if (send_bytes < 0) {
        perror("Failed to send on TCP socket");
        exit(-1);
      }
      send += send_bytes;
    }
  }

  NetworkFrame sock_recv() {
    uint8_t rx_buf[24];
    while (true) {
      size_t received = 0;
      while (received != sizeof(rx_buf)) {
        ssize_t bytes_received =
            ::recv(m_socket, (void *)rx_buf, sizeof(rx_buf), MSG_WAITALL);

        if (bytes_received < 0) {
          perror("Failed to receive TCP frame");
          exit(-1);
        }
        if (bytes_received == 0) {
          std::cout << "\033[1;31mTCP connection closed\033[0m" << std::endl;
          exit(-1);
        }
        received += bytes_received;
      }

      uint8_t tag = rx_buf[0];
      if (tag == 0x1) {
        break;
      }
      // ignores keep alive frames!
    }


    uint64_t timestamp_us = reinterpret_cast<uint64_t *>(rx_buf)[1];

    return NetworkFrame(CanFrame(reinterpret_cast<uint32_t *>(rx_buf)[1], 
          reinterpret_cast<uint64_t*>(rx_buf)[2], rx_buf[2]),
                        rx_buf[1]);
  }

  void sock_recv_task() {
    while (true) {
      auto frame = sock_recv();
      // intentionally ignore dropped frames
      bool suc = m_sync_buffers[frame.bus_id].enqueue(frame.can_frame);
      if (!suc) {
        std::cout << "Dropped CanFrame" << std::endl;
      }
    }
  }

  struct NetworkFrame {
    CanFrame can_frame;
    uint8_t bus_id;

    NetworkFrame(const CanFrame &can_frame, uint8_t bus_id)
        : can_frame(can_frame), bus_id(bus_id) {}
  };

  struct CyclicBuffer {

    CyclicBuffer() : m_buffer(), m_read(0), m_write(0) {}

    bool enqueue(const CanFrame &frame) {
      const size_t w = m_write.load(std::memory_order_relaxed);
      size_t w_next = w + 1;
      if (w_next == BUF_SIZE) {
        w_next = 0U;
      }
      const size_t r = m_read.load(std::memory_order_acquire);
      if (w_next == r) {
        return false;
      }
      m_buffer[w] = frame;
      m_write.store(w_next, std::memory_order_release);
      return true;
    }

    std::optional<CanFrame> dequeue() {
      size_t r = m_read.load(std::memory_order_relaxed);
      const size_t w = m_write.load(std::memory_order_acquire);
      if (r == w) {
        return std::nullopt;
      }
      assert(m_buffer[r].has_value());
      CanFrame ret = m_buffer[r].value();
      r++;
      if (r == BUF_SIZE) {
        r = 0U;
      }
      m_read.store(r, std::memory_order_release);
      return ret;
    }

  private:
    static constexpr size_t BUF_SIZE = 128;
    std::optional<CanFrame> m_buffer[BUF_SIZE];
    std::atomic_size_t m_read;
    std::atomic_size_t m_write;
  };


  int m_socket;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_sor;
  struct sockaddr_in m_broadcast_addr;
  std::array<CyclicBuffer, BUS_COUNT> m_sync_buffers;
  std::shared_ptr<std::thread> m_recv_task;

public: // <- Not really intentional, but probably fine
  explicit TcpCan(
      int socket,
      std::chrono::time_point<std::chrono::high_resolution_clock> sor,
      sockaddr_in broadcast_addr)
      : m_socket(socket), m_sor(sor), m_broadcast_addr(broadcast_addr) {
    m_recv_task = std::make_shared<std::thread>(&TcpCan<BUS_COUNT>::sock_recv_task, this);
  };
};
