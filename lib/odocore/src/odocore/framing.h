// CRC-16/CCITT-FALSE and COBS (Consistent Overhead Byte Stuffing).
//
// CRC: polynomial 0x1021, initial value 0xFFFF, no reflection, no final XOR.
// Check value: crc16_ccitt("123456789") == 0x29B1.
//
// COBS removes every 0x00 from a packet so that 0x00 can delimit frames on
// the serial line; overhead is one byte per 254 bytes plus one.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace odocore {

uint16_t crc16_ccitt(const uint8_t* data, size_t length, uint16_t crc = 0xFFFFu);

/// Worst-case COBS output size (without the 0x00 delimiter).
constexpr size_t cobs_max_encoded_size(size_t length) { return length + length / 254u + 1u; }

/// Encode `length` bytes into `out`. Returns the encoded size, or 0 if
/// `capacity` is too small (a valid encoding is never empty).
size_t cobs_encode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity);

/// Decode a COBS block (no delimiter). Returns false on malformed input
/// (embedded 0x00, a code byte pointing past the end) or if `capacity` is too
/// small; otherwise stores the decoded size in `*out_length`.
bool cobs_decode(const uint8_t* in, size_t length, uint8_t* out, size_t capacity,
                 size_t* out_length);

}  // namespace odocore
