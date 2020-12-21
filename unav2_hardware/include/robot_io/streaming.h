#pragma once
#ifndef STREAMING_H
#define STREAMING_H
#include <pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <vector>
namespace robot_io {

class Streaming {
private:
  Streaming(){};

public:
  static constexpr size_t SIZE_FIELD_LENGHT = 1;
  static inline std::vector<uint8_t> encode(const std::vector<uint8_t> &input) {
    std::vector<uint8_t> ret;
    //ret.reserve(input.size() + 1);
    ret.push_back(static_cast<uint8_t>(input.size()));
    ret.insert(ret.begin() + 1, input.cbegin(), input.cend());
    return ret;
  }

  /**
   * @brief Decode the data in the <input> buffer and store them in the output buffer
   * after checking the size. It also removes the decoded data from the input buffer.
   * @param input incoming data buffer. Data are removed from this vector after being
   * decode into output vector
   * @param output store the data decoded and removed from the input vector
   * @return true if input contains enough data. False otherwise
   */
  static inline bool decode(std::vector<uint8_t> &input, std::vector<uint8_t> &output) {
    auto size = static_cast<size_t>(input.front());
    if (input.size() - 1 < size) {
      return false;
    }

    auto begin = input.begin();
    output.insert(output.cbegin(), begin + 1, begin + 1 + size);

    if (input.size() == size + 1) {
      input.clear();
    } else {
      begin = input.begin();
      input.erase(begin, begin + 1 + size);
    }
    return true;
  }
};
};     // namespace robot_io
#endif /* STREAMING_H */
