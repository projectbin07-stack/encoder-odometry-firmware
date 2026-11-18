// CRC, COBS and protocol encode/decode (hand-built cases; the shared JSON
// vectors are checked separately in test_protocol_vectors).
#include <math.h>
#include <string.h>
#include <unity.h>

#include <odocore/framing.h>
#include <odocore/protocol.h>

using namespace odocore;

void setUp() {}
void tearDown() {}

static void test_crc_check_value() {
  const char* text = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(reinterpret_cast<const uint8_t*>(text), 9));
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16_ccitt(nullptr, 0));
  // Chaining: CRC of the second half seeded with the CRC of the first.
  const uint16_t first = crc16_ccitt(reinterpret_cast<const uint8_t*>(text), 5);
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(reinterpret_cast<const uint8_t*>(text) + 5, 4, first));
}

static void test_cobs_small_examples() {
  uint8_t out[16];
  const uint8_t a[] = {0x11, 0x22, 0x00, 0x33};
  const uint8_t a_enc[] = {0x03, 0x11, 0x22, 0x02, 0x33};
  TEST_ASSERT_EQUAL(5, cobs_encode(a, 4, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(a_enc, out, 5);

  const uint8_t z[] = {0x00};
  TEST_ASSERT_EQUAL(2, cobs_encode(z, 1, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT8(0x01, out[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, out[1]);

  TEST_ASSERT_EQUAL(1, cobs_encode(nullptr, 0, out, sizeof(out)));
  TEST_ASSERT_EQUAL_UINT8(0x01, out[0]);

  uint8_t dec[16];
  size_t n = 99;
  TEST_ASSERT_TRUE(cobs_decode(a_enc, 5, dec, sizeof(dec), &n));
  TEST_ASSERT_EQUAL(4, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(a, dec, 4);
}

static void test_cobs_254_byte_blocks() {
  uint8_t data[255];
  for (int i = 0; i < 255; ++i) {
    data[i] = static_cast<uint8_t>(i + 1);  // 01..FF
  }
  uint8_t enc[300];
  uint8_t dec[300];
  size_t n = 0;
  // 254 non-zero bytes: FF 01..FE with no trailing block.
  TEST_ASSERT_EQUAL(255, cobs_encode(data, 254, enc, sizeof(enc)));
  TEST_ASSERT_EQUAL_UINT8(0xFF, enc[0]);
  TEST_ASSERT_EQUAL_UINT8(0xFE, enc[254]);
  TEST_ASSERT_TRUE(cobs_decode(enc, 255, dec, sizeof(dec), &n));
  TEST_ASSERT_EQUAL(254, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(data, dec, 254);
  // 255 non-zero bytes: FF 01..FE 02 FF.
  TEST_ASSERT_EQUAL(257, cobs_encode(data, 255, enc, sizeof(enc)));
  TEST_ASSERT_EQUAL_UINT8(0x02, enc[255]);
  TEST_ASSERT_EQUAL_UINT8(0xFF, enc[256]);
  TEST_ASSERT_TRUE(cobs_decode(enc, 257, dec, sizeof(dec), &n));
  TEST_ASSERT_EQUAL(255, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(data, dec, 255);
}

static void test_cobs_rejects_malformed_and_small_buffers() {
  uint8_t out[8];
  size_t n = 0;
  const uint8_t zero_code[] = {0x00};
  const uint8_t past_end[] = {0x05, 0x11, 0x22};
  const uint8_t zero_inside[] = {0x03, 0x11, 0x00};
  TEST_ASSERT_FALSE(cobs_decode(zero_code, 1, out, sizeof(out), &n));
  TEST_ASSERT_FALSE(cobs_decode(past_end, 3, out, sizeof(out), &n));
  TEST_ASSERT_FALSE(cobs_decode(zero_inside, 3, out, sizeof(out), &n));
  const uint8_t ok[] = {0x03, 0x11, 0x22, 0x02, 0x33};
  TEST_ASSERT_FALSE(cobs_decode(ok, 5, out, 3, &n));  // needs 4 bytes
  const uint8_t data[] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL(0, cobs_encode(data, 4, out, 4));  // needs 5 bytes
}

static void test_velocity_command_packet_layout() {
  Message m;
  m.type = MsgType::kVelocityCommand;
  m.seq = 0;
  m.velocity_command.vx = 0.5f;
  m.velocity_command.vy = 0.0f;
  m.velocity_command.wz = 0.0f;
  uint8_t p[kMaxPacketSize];
  TEST_ASSERT_EQUAL(18, encode_packet(m, p, sizeof(p)));
  // version, type, seq, length; then 0.5f = 0x3F000000 little-endian.
  const uint8_t head[] = {0x01, 0x01, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x3F};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(head, p, 8);
  const uint16_t crc = crc16_ccitt(p, 16);
  TEST_ASSERT_EQUAL_UINT8(crc & 0xFF, p[16]);
  TEST_ASSERT_EQUAL_UINT8(crc >> 8, p[17]);
  TEST_ASSERT_EQUAL(0, encode_packet(m, p, 17));  // too small
}

static void test_body_sizes() {
  const int expected[] = {12, 60, 28, 41, 1, 5, 6, 12};
  for (int t = 1; t <= 8; ++t) {
    TEST_ASSERT_EQUAL_INT(expected[t - 1], body_size(static_cast<uint8_t>(t)));
    TEST_ASSERT_TRUE(static_cast<size_t>(expected[t - 1]) <= kMaxBodySize);
  }
  TEST_ASSERT_EQUAL_INT(-1, body_size(0));
  TEST_ASSERT_EQUAL_INT(-1, body_size(0x7F));
}

static void test_every_type_roundtrips_through_frames() {
  Message msgs[8];
  msgs[0].type = MsgType::kVelocityCommand;
  msgs[0].velocity_command.vx = -0.25f;
  msgs[0].velocity_command.vy = 0.125f;
  msgs[0].velocity_command.wz = 1.5f;
  msgs[1].type = MsgType::kWheelState;
  msgs[1].wheel_state.t_us = 77;
  for (int i = 0; i < 4; ++i) {
    msgs[1].wheel_state.count[i] = -1000 * i;
    msgs[1].wheel_state.velocity[i] = 0.5f * static_cast<float>(i);
    msgs[1].wheel_state.target[i] = 1.5f;
    msgs[1].wheel_state.duty[i] = static_cast<int16_t>(-2500 * i);
  }
  msgs[2].type = MsgType::kOdometry;
  msgs[2].odometry.t_us = 0xFFFFFFFFu;
  msgs[2].odometry.x = 1.0f;
  msgs[2].odometry.theta = -3.0f;
  msgs[3].type = MsgType::kDiagnostics;
  msgs[3].diagnostics.illegal_transitions[3] = 65536;
  msgs[3].diagnostics.flags = kDiagWatchdogExpired | kDiagSaturated;
  msgs[4].type = MsgType::kParamGet;
  msgs[4].param_get.id = 21;
  msgs[5].type = MsgType::kParamSet;
  msgs[5].param_set.id = 9;
  msgs[5].param_set.value = 12.5f;
  msgs[6].type = MsgType::kParamValue;
  msgs[6].param_value.id = 14;
  msgs[6].param_value.status = 2;
  msgs[6].param_value.value = 45.0f;
  msgs[7].type = MsgType::kSetPose;
  msgs[7].set_pose.y = -2.0f;

  FrameReceiver rx;
  for (int k = 0; k < 8; ++k) {
    msgs[k].seq = static_cast<uint8_t>(250 + k);  // wraps past 255
    uint8_t frame[kMaxFrameSize];
    const size_t n = encode_frame(msgs[k], frame, sizeof(frame));
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL_UINT8(0x00, frame[n - 1]);
    for (size_t i = 0; i + 1 < n; ++i) {
      TEST_ASSERT_NOT_EQUAL(0, frame[i]);
    }
    Message out;
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
      got = rx.push(frame[i], &out);
    }
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(msgs[k].type), static_cast<uint8_t>(out.type));
    TEST_ASSERT_EQUAL_UINT8(msgs[k].seq, out.seq);
    uint8_t a[kMaxPacketSize];
    uint8_t b[kMaxPacketSize];
    const size_t na = encode_packet(msgs[k], a, sizeof(a));
    const size_t nb = encode_packet(out, b, sizeof(b));
    TEST_ASSERT_EQUAL(na, nb);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, b, na);
  }
  TEST_ASSERT_EQUAL_UINT32(8, rx.frames_ok());
  TEST_ASSERT_EQUAL_FLOAT(12.5f, msgs[5].param_set.value);
}

static void test_receiver_counts_errors_and_resyncs() {
  Message m;
  m.type = MsgType::kParamGet;
  m.param_get.id = 1;
  uint8_t frame[kMaxFrameSize];
  const size_t n = encode_frame(m, frame, sizeof(frame));
  FrameReceiver rx;
  Message out;
  // Leading delimiters are ignored.
  TEST_ASSERT_FALSE(rx.push(0x00, &out));
  // A corrupted copy: frame is 03 01 05 05 01 01 08 24 00; flipping a bit
  // in a data byte (index 5) keeps the COBS structure but breaks the CRC.
  for (size_t i = 0; i < n; ++i) {
    rx.push(i == 5 ? static_cast<uint8_t>(frame[i] ^ 0x10) : frame[i], &out);
  }
  TEST_ASSERT_EQUAL_UINT32(1, rx.crc_errors());
  // Garbage that is not valid COBS: frame error.
  rx.push(0x05, &out);
  rx.push(0x01, &out);
  rx.push(0x00, &out);
  TEST_ASSERT_EQUAL_UINT32(1, rx.frame_errors());
  // An overlong frame: frame error, then the next good frame still decodes.
  for (int i = 0; i < 200; ++i) {
    rx.push(0x01, &out);
  }
  rx.push(0x00, &out);
  TEST_ASSERT_EQUAL_UINT32(2, rx.frame_errors());
  bool got = false;
  for (size_t i = 0; i < n; ++i) {
    got = rx.push(frame[i], &out);
  }
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL_UINT8(1, out.param_get.id);
}

static void test_duty_to_wire() {
  TEST_ASSERT_EQUAL_INT16(5000, duty_to_wire(0.5f));
  TEST_ASSERT_EQUAL_INT16(-2500, duty_to_wire(-0.25f));
  TEST_ASSERT_EQUAL_INT16(10000, duty_to_wire(1.5f));
  TEST_ASSERT_EQUAL_INT16(-10000, duty_to_wire(-2.0f));
  TEST_ASSERT_EQUAL_INT16(0, duty_to_wire(NAN));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_check_value);
  RUN_TEST(test_cobs_small_examples);
  RUN_TEST(test_cobs_254_byte_blocks);
  RUN_TEST(test_cobs_rejects_malformed_and_small_buffers);
  RUN_TEST(test_velocity_command_packet_layout);
  RUN_TEST(test_body_sizes);
  RUN_TEST(test_every_type_roundtrips_through_frames);
  RUN_TEST(test_receiver_counts_errors_and_resyncs);
  RUN_TEST(test_duty_to_wire);
  return UNITY_END();
}
