#ifndef STSERVOCONTROLLER__H
#define STSERVOCONTROLLER__H

#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "ServoController.h"

#include "STServoPacket.h"


class STServoController : public ServoController<STServoPacket> {
  private:
     static constexpr const char* LOG_TAG = "STServoController";
  public:
    STServoController(ServoBus* bus, int tx_timeout_ms, int rx_timeout_ms);

    virtual int send_raw_command(STServoPacket& cmd);
    virtual int read_raw_response(STServoPacket& rsp);
    private:
      bool m_disable_rx_on_tx;
    // Additional methods specific to STServoController can be added here
};

#endif  // STSERVOCONTROLLER__H
