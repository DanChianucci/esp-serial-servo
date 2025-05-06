#ifndef SERVOPACKET__H
#define SERVOPACKET__H
#include <cstdint>
#include <iterator>
#include <utility>
#include <vector>
#include <span>
#include <concepts>



class ServoPacket {
 public:

  ServoPacket() = default;

  template <std::input_iterator T>
  ServoPacket(T begin, T end) : raw_buffer(begin, end) {}

  template <std::constructible_from<std::vector<uint8_t>> T>
  ServoPacket(T&& sequence) : raw_buffer(std::forward(sequence)) {}


  template <std::input_iterator T>
  void set_buffer(T begin, T end){
    raw_buffer = std::vector<uint8_t>(begin, end);
  }

  template <typename T> requires std::assignable_from<std::vector<uint8_t>,T>
  void set_buffer(T&& sequence) {
    raw_buffer = std::forward(sequence);
  }


  virtual uint8_t size();
  virtual const std::vector<uint8_t>& get_buffer();
  virtual const uint8_t& operator[](int idx) const;



 protected:
  std::vector<uint8_t> raw_buffer;
};

#endif  // SERVOPACKET__H
