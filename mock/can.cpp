#include "tcpcan.h"
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include "canzero/canzero.h"
#include "util/timestamp.h"

static std::unique_ptr<TcpCan<2>> s_tcpcan = nullptr;

void canzero_can0_setup(uint32_t baudrate, canzero_can_filter *filters,
                        int filter_count) {
  if (s_tcpcan.get() == nullptr) {
    s_tcpcan = TcpCan<2>::connect();
  }
}

void canzero_can0_send(canzero_frame *frame) {
  CanFrame can_frame(frame->id, *reinterpret_cast<uint64_t*>(frame->data), frame->dlc);
  s_tcpcan->send<0>(can_frame);
}

int canzero_can0_recv(canzero_frame *frame) {
  std::optional<CanFrame> rx = s_tcpcan->recv<0>();
  if (!rx.has_value()) {
    return 0;
  }
  frame->dlc = rx->dlc;
  frame->id = rx->can_id;
  *reinterpret_cast<uint64_t*>(frame->data) = rx->data;
  return 1;
}

void canzero_can1_setup(uint32_t baudrate, canzero_can_filter *filters,
                        int filter_count) {
  if (s_tcpcan.get() == nullptr) {
    s_tcpcan = TcpCan<2>::connect();
  }
}
void canzero_can1_send(canzero_frame *frame) {
  CanFrame can_frame(frame->id, *reinterpret_cast<uint64_t*>(frame->data), frame->dlc);
  s_tcpcan->send<1>(can_frame);
}
int canzero_can1_recv(canzero_frame *frame) {
  std::optional<CanFrame> rx = s_tcpcan->recv<1>();
  if (!rx.has_value()) {
    return 0;
  }
  frame->dlc = rx->dlc;
  frame->id = rx->can_id;
  *reinterpret_cast<uint64_t*>(frame->data) = rx->data;
  return 1;
}

static Timestamp g_start = Timestamp::now();


static std::thread s_can0_poll_thread;
static std::thread s_can1_poll_thread;
static std::thread s_can_update_thread;

static std::condition_variable s_can_update_sleep_cv;
static std::mutex s_can_update_mutex;

static auto s_sor = std::chrono::high_resolution_clock::now();

uint32_t canzero_get_time() { 
  using namespace std::chrono;
  return duration_cast<milliseconds>(high_resolution_clock::now() - s_sor).count(); 
}

void canzero_request_update(uint32_t time) {
  // pass
}

static std::mutex s_canzero_critical;

void canzero_enter_critical() { s_canzero_critical.lock(); }
void canzero_exit_critical() { s_canzero_critical.unlock(); }
