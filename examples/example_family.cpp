// Tier 2: STS typed helpers in the family's native units, with an explicit
// EEPROM unlock/lock sequence around a configuration write.
#include <chrono>
#include <cstdio>

#include "example_support.h"
#include "servokit.h"

using namespace servokit;
using namespace example;
using std::chrono::milliseconds;

// Stores an over-temperature threshold across power cycles. The library never
// changes the EEPROM lock on its own: unlock, write, lock again.
static Result store_temperature_limit(STSServo& sts, uint8_t celsius) {
  Result r = sts.unlock_eeprom();
  if (r != Result::OK) return r;
  const Result written = sts.set_overtemp_threshold(celsius);
  // Lock again even if the write failed, so the servo is left write-protected.
  r = sts.lock_eeprom();
  return written != Result::OK ? written : r;
}

int main() {
  servokit_test::FakeSerialPort port;
  const Bytes ack = reply(1);
  for (int i = 0; i < 3; ++i) port.script_reply(ack);  // unlock, threshold, lock
  port.script_reply(ack);                              // target position
  port.script_reply(reply(1, {0x01}));                 // lock register reads 1

  ServoController controller(&port, milliseconds(20), milliseconds(20));
  STSServo sts(&controller, 1);

  EXPECT(store_temperature_limit(sts, 70) == Result::OK);

  // Native units: steps, 4096 per revolution. A range error returns
  // OUT_OF_RANGE and sends nothing.
  EXPECT(sts.set_target_position(2048) == Result::OK);
  EXPECT(sts.set_target_position(STSServo::MAX_TARGET_POSITION + 1) == Result::OUT_OF_RANGE);

  bool locked = false;
  EXPECT(sts.get_eeprom_locked(&locked) == Result::OK);
  EXPECT(locked);
  std::printf("eeprom locked: %s, frames sent: %zu\n", locked ? "yes" : "no", port.tx_log().size());

  EXPECT(port.tx_log().size() == 5);  // the rejected move sent nothing
  return failures() == 0 ? 0 : 1;
}
