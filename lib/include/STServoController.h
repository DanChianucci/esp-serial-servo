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
  public:
    STServoController(ServoBus* bus, int tx_timeout_ms, int rx_timeout_ms);

    virtual int send_raw_command(STServoPacket& cmd);
    virtual int read_raw_response(STServoPacket& rsp);
};

#endif  // STSERVOCONTROLLER__H
