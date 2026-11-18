#include "odocore/framing.h"

namespace odocore {

uint16_t crc16_ccitt(const uint8_t* data, size_t length, uint16_t crc) {
  for (size_t i = 0; i < length; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

size_t cobs_encode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity) {
  if (capacity < cobs_max_encoded_size(length)) {
    return 0;
  }
  size_t write = 1;      // next output position
  size_t code_pos = 0;   // where the current block's code byte goes
  uint8_t code = 1;      // 1 + number of data bytes in the current block
  for (size_t read = 0; read < length; ++read) {
    const uint8_t byte = in[read];
    if (byte != 0) {
      out[write++] = byte;
      ++code;
    }
    if (byte == 0 || code == 0xFF) {
      out[code_pos] = code;
      code = 1;
      code_pos = write;
      // A full 254-byte block at the very end of the input needs no new block.
      if (byte == 0 || read + 1 < length) {
        ++write;
      }
    }
  }
  if (code_pos < write) {  // not when a full 254-byte block ended the input
    out[code_pos] = code;
  }
  return write;
}

bool cobs_decode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity,
                 size_t* out_length) {
  size_t read = 0;
  size_t write = 0;
  while (read < length) {
    const uint8_t code = in[read++];
    if (code == 0) {
      return false;
    }
    for (uint8_t i = 1; i < code; ++i) {
      if (read >= length || in[read] == 0 || write >= capacity) {
        return false;
      }
      out[write++] = in[read++];
    }
    if (code != 0xFF && read < length) {
      if (write >= capacity) {
        return false;
      }
      out[write++] = 0;
    }
  }
  if (out_length != nullptr) {
    *out_length = write;
  }
  return true;
}

}  // namespace odocore
