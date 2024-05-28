#pragma once

#include <FlexCAN_T4.h>
#include <imxrt.h>
#include <inttypes.h>

#include <avr/pgmspace.h>

enum class CanInterface { CAN1, CAN2, CAN3 };

enum CanBaudrate : uint32_t {
  CAN_BAUDRATE_125Kbps = 125000,
  CAN_BAUDRATE_250Kbps = 250000,
  CAN_BAUDRATE_500Kbps = 500000,
  CAN_BAUDRATE_1000Kbps = 1000000,
};

struct CanFilter {
  uint32_t id;
  uint32_t mask;
  bool ide;
};

struct CanBeginInfo {
  CanBaudrate baudrate;

  CanFilter *filters;
  size_t filter_count;

  bool loopback;

  CanBeginInfo()
      : baudrate(CAN_BAUDRATE_1000Kbps), filters(nullptr), filter_count(0),
        loopback(false) {}
};

template <CanInterface INTERFACE> struct CAN {
private:
  struct RxSoftwareFifo {

    static constexpr size_t FIFO_CAP = 128;
    CAN_message_t m_queue[FIFO_CAP];
    unsigned int m_start;
    unsigned int m_end;

    // has to be called from ISR_CANX
    inline void enqueue(const CAN_message_t &msg) {
      m_queue[m_end] = msg;
      m_end = (m_end + 1) % FIFO_CAP;
    }

    inline int dequeue(CAN_message_t &msg) {
      NVIC_DISABLE_IRQ(GetCanIRQ());
      if (m_start == m_end) {
        NVIC_ENABLE_IRQ(GetCanIRQ());
        return 0;
      }
      msg = m_queue[m_start];
      m_start = (m_start + 1) % FIFO_CAP;

      NVIC_ENABLE_IRQ(GetCanIRQ());
      return 1;
    }

    static constexpr int GetCanIRQ() {
      if constexpr (INTERFACE == CanInterface::CAN1) {
        return IRQ_CAN1;
      } else if constexpr (INTERFACE == CanInterface::CAN2) {
        return IRQ_CAN2;
      } else {
        return IRQ_CAN3;
      }
    }
  };

public:
  static void FLASHMEM begin(const CanBeginInfo &begin_info) {
    m_flexcan.begin();
    m_flexcan.setMaxMB(64);
    m_flexcan.setBaudRate(static_cast<uint32_t>(begin_info.baudrate));
    m_flexcan.enableFIFO();
    if (begin_info.filter_count > 0) {
      m_flexcan.setFIFOFilter(REJECT_ALL);
    }

    for (size_t i = 0; i < begin_info.filter_count; ++i) {
      m_flexcan.setFIFOManualFilter(i, begin_info.filters[i].id,
                                    begin_info.filters[i].mask,
                                    begin_info.filters[i].ide ? EXT : STD);
    }
    m_flexcan.enableFIFOInterrupt();
    m_flexcan.onReceive(enqueue);
    if (begin_info.loopback) {
      m_flexcan.enableLoopBack();
    }
    m_flexcan.mailboxStatus();
  }

  static void FASTRUN enqueue(const CAN_message_t &msg) { m_fifo.enqueue(msg); }

  /**
   * Receives a message from the CAN and writes it to @param msg.
   * @returns 1 iff. a message was received and written, otherwise (if not
   * message was received) returns false. NOTE: This function should not be
   * called from a ISR!
   */
  static int FASTRUN recv(CAN_message_t &msg) { return m_fifo.dequeue(msg); }

  /**
   * Sends @param msg on the CAN.
   * @returns 1 iff. the message was successfully send otherwise returns false.
   * NOTE: Sending is not buffered sending a lot might overflow the TX_QUEUE,
   * which leads to dropped messages!
   */
  static int FASTRUN send(const CAN_message_t &msg) { return m_flexcan.write(msg); }

private:
  static constexpr CAN_DEV_TABLE FlexCanModule() {
    if constexpr (INTERFACE == CanInterface::CAN1) {
      return CAN_DEV_TABLE::CAN1;
    } else if constexpr (INTERFACE == CanInterface::CAN2) {
      return CAN_DEV_TABLE::CAN2;
    } else if constexpr (INTERFACE == CanInterface::CAN3) {
      return CAN_DEV_TABLE::CAN3;
    }
  }

  typedef FlexCAN_T4<CAN::FlexCanModule(), RX_SIZE_256, TX_SIZE_16> FlexCAN;

  static FlexCAN m_flexcan;
  static RxSoftwareFifo m_fifo;
};

template <CanInterface INTERFACE>
DMAMEM typename CAN<INTERFACE>::FlexCAN CAN<INTERFACE>::m_flexcan;

template <CanInterface INTERFACE> 
DMAMEM typename CAN<INTERFACE>::RxSoftwareFifo CAN<INTERFACE>::m_fifo;


 typedef CAN<CanInterface::CAN1> Can1;
 typedef CAN<CanInterface::CAN2> Can2;
 typedef CAN<CanInterface::CAN3> Can3;

