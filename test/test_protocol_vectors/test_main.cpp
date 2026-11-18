// Checks the C++ protocol against test/vectors/protocol_vectors.json, the
// same file host/tests/test_protocol.py reads, so both implementations are
// held to identical bytes.
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#include <string>

#include <odocore/framing.h>
#include <odocore/protocol.h>

#include "mini_json.h"

using minijson::Value;
using namespace odocore;

static Value g_vectors;
static std::string g_path;

void setUp() {}
void tearDown() {}

static std::string to_hex(const uint8_t* data, size_t n) {
  static const char* digits = "0123456789abcdef";
  std::string out;
  for (size_t i = 0; i < n; ++i) {
    out.push_back(digits[data[i] >> 4]);
    out.push_back(digits[data[i] & 0x0F]);
  }
  return out;
}

static std::string from_hex(const std::string& hex) {
  std::string out;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<char>(strtol(hex.substr(i, 2).c_str(), nullptr, 16)));
  }
  return out;
}

static const uint8_t* bytes(const std::string& s) {
  return reinterpret_cast<const uint8_t*>(s.data());
}

static bool load_vectors() {
  // The test binary may run from the project root or elsewhere; try a path
  // relative to this source file first, then relative to the working dir.
  std::string here(__FILE__);
  const size_t slash = here.find_last_of("/\\");
  const std::string dir = slash == std::string::npos ? std::string(".") : here.substr(0, slash);
  const std::string candidates[] = {
      dir + "/../vectors/protocol_vectors.json",
      "test/vectors/protocol_vectors.json",
      "../test/vectors/protocol_vectors.json",
  };
  for (const std::string& path : candidates) {
    std::string text;
    if (minijson::read_file(path, &text)) {
      minijson::Parser parser(text);
      if (parser.parse(&g_vectors)) {
        g_path = path;
        return true;
      }
    }
  }
  return false;
}

static float f(const Value& v) { return static_cast<float>(v.number); }

static Message from_fields(const std::string& name, const Value& fl) {
  Message m;
  if (name == "velocity_command") {
    m.type = MsgType::kVelocityCommand;
    m.velocity_command.vx = f(fl["vx"]);
    m.velocity_command.vy = f(fl["vy"]);
    m.velocity_command.wz = f(fl["wz"]);
  } else if (name == "wheel_state") {
    m.type = MsgType::kWheelState;
    m.wheel_state.t_us = static_cast<uint32_t>(fl["t_us"].number);
    for (size_t i = 0; i < 4; ++i) {
      m.wheel_state.count[i] = static_cast<int32_t>(fl["count"].at(i).number);
      m.wheel_state.velocity[i] = f(fl["velocity"].at(i));
      m.wheel_state.target[i] = f(fl["target"].at(i));
      m.wheel_state.duty[i] = static_cast<int16_t>(fl["duty"].at(i).number);
    }
  } else if (name == "odometry") {
    m.type = MsgType::kOdometry;
    m.odometry.t_us = static_cast<uint32_t>(fl["t_us"].number);
    m.odometry.x = f(fl["x"]);
    m.odometry.y = f(fl["y"]);
    m.odometry.theta = f(fl["theta"]);
    m.odometry.vx = f(fl["vx"]);
    m.odometry.vy = f(fl["vy"]);
    m.odometry.wz = f(fl["wz"]);
  } else if (name == "diagnostics") {
    m.type = MsgType::kDiagnostics;
    DiagnosticsMsg& d = m.diagnostics;
    d.t_us = static_cast<uint32_t>(fl["t_us"].number);
    for (size_t i = 0; i < 4; ++i) {
      d.illegal_transitions[i] = static_cast<uint32_t>(fl["illegal_transitions"].at(i).number);
    }
    d.loop_overruns = static_cast<uint32_t>(fl["loop_overruns"].number);
    d.watchdog_trips = static_cast<uint32_t>(fl["watchdog_trips"].number);
    d.max_loop_us = static_cast<uint32_t>(fl["max_loop_us"].number);
    d.rx_crc_errors = static_cast<uint32_t>(fl["rx_crc_errors"].number);
    d.rx_frame_errors = static_cast<uint32_t>(fl["rx_frame_errors"].number);
    d.flags = static_cast<uint8_t>(fl["flags"].number);
  } else if (name == "param_get") {
    m.type = MsgType::kParamGet;
    m.param_get.id = static_cast<uint8_t>(fl["id"].number);
  } else if (name == "param_set") {
    m.type = MsgType::kParamSet;
    m.param_set.id = static_cast<uint8_t>(fl["id"].number);
    m.param_set.value = f(fl["value"]);
  } else if (name == "param_value") {
    m.type = MsgType::kParamValue;
    m.param_value.id = static_cast<uint8_t>(fl["id"].number);
    m.param_value.status = static_cast<uint8_t>(fl["status"].number);
    m.param_value.value = f(fl["value"]);
  } else if (name == "set_pose") {
    m.type = MsgType::kSetPose;
    m.set_pose.x = f(fl["x"]);
    m.set_pose.y = f(fl["y"]);
    m.set_pose.theta = f(fl["theta"]);
  } else {
    TEST_FAIL_MESSAGE(("unknown message name " + name).c_str());
  }
  return m;
}

#define CHECK_FIELD(actual, json_value, label) \
  TEST_ASSERT_TRUE_MESSAGE(static_cast<double>(actual) == (json_value).number, (label).c_str())

// Field-by-field comparison of a decoded message with the JSON fields.
static void check_fields(const Message& m, const std::string& name, const Value& fl,
                         const std::string& label) {
  if (name == "velocity_command") {
    CHECK_FIELD(m.velocity_command.vx, fl["vx"], label);
    CHECK_FIELD(m.velocity_command.vy, fl["vy"], label);
    CHECK_FIELD(m.velocity_command.wz, fl["wz"], label);
  } else if (name == "wheel_state") {
    CHECK_FIELD(m.wheel_state.t_us, fl["t_us"], label);
    for (size_t i = 0; i < 4; ++i) {
      CHECK_FIELD(m.wheel_state.count[i], fl["count"].at(i), label);
      CHECK_FIELD(m.wheel_state.velocity[i], fl["velocity"].at(i), label);
      CHECK_FIELD(m.wheel_state.target[i], fl["target"].at(i), label);
      CHECK_FIELD(m.wheel_state.duty[i], fl["duty"].at(i), label);
    }
  } else if (name == "odometry") {
    CHECK_FIELD(m.odometry.t_us, fl["t_us"], label);
    CHECK_FIELD(m.odometry.x, fl["x"], label);
    CHECK_FIELD(m.odometry.y, fl["y"], label);
    CHECK_FIELD(m.odometry.theta, fl["theta"], label);
    CHECK_FIELD(m.odometry.vx, fl["vx"], label);
    CHECK_FIELD(m.odometry.vy, fl["vy"], label);
    CHECK_FIELD(m.odometry.wz, fl["wz"], label);
  } else if (name == "diagnostics") {
    const DiagnosticsMsg& d = m.diagnostics;
    CHECK_FIELD(d.t_us, fl["t_us"], label);
    for (size_t i = 0; i < 4; ++i) {
      CHECK_FIELD(d.illegal_transitions[i], fl["illegal_transitions"].at(i), label);
    }
    CHECK_FIELD(d.loop_overruns, fl["loop_overruns"], label);
    CHECK_FIELD(d.watchdog_trips, fl["watchdog_trips"], label);
    CHECK_FIELD(d.max_loop_us, fl["max_loop_us"], label);
    CHECK_FIELD(d.rx_crc_errors, fl["rx_crc_errors"], label);
    CHECK_FIELD(d.rx_frame_errors, fl["rx_frame_errors"], label);
    CHECK_FIELD(d.flags, fl["flags"], label);
  } else if (name == "param_get") {
    CHECK_FIELD(m.param_get.id, fl["id"], label);
  } else if (name == "param_set") {
    CHECK_FIELD(m.param_set.id, fl["id"], label);
    CHECK_FIELD(m.param_set.value, fl["value"], label);
  } else if (name == "param_value") {
    CHECK_FIELD(m.param_value.id, fl["id"], label);
    CHECK_FIELD(m.param_value.status, fl["status"], label);
    CHECK_FIELD(m.param_value.value, fl["value"], label);
  } else if (name == "set_pose") {
    CHECK_FIELD(m.set_pose.x, fl["x"], label);
    CHECK_FIELD(m.set_pose.y, fl["y"], label);
    CHECK_FIELD(m.set_pose.theta, fl["theta"], label);
  }
}

static void test_file_loaded() {
  TEST_ASSERT_TRUE_MESSAGE(g_vectors.type == Value::kObject, "protocol_vectors.json not found");
  TEST_ASSERT_EQUAL_INT(kProtocolVersion, static_cast<int>(g_vectors["protocol_version"].number));
  TEST_ASSERT_TRUE(g_vectors["messages"].size() >= 8);
  printf("vectors: %s\n", g_path.c_str());
}

static void test_crc_vectors() {
  const Value& cases = g_vectors["crc16"];
  TEST_ASSERT_TRUE(cases.size() > 0);
  for (size_t i = 0; i < cases.size(); ++i) {
    const std::string& text = cases.at(i)["ascii"].string;
    const uint16_t expected = static_cast<uint16_t>(cases.at(i)["crc"].number);
    TEST_ASSERT_EQUAL_HEX16_MESSAGE(expected, crc16_ccitt(bytes(text), text.size()),
                                    cases.at(i)["name"].string.c_str());
  }
}

static void test_cobs_vectors() {
  const Value& cases = g_vectors["cobs"];
  TEST_ASSERT_TRUE(cases.size() > 0);
  for (size_t i = 0; i < cases.size(); ++i) {
    const std::string& label = cases.at(i)["name"].string;
    const std::string decoded = from_hex(cases.at(i)["decoded"].string);
    uint8_t enc[600];
    const size_t n = cobs_encode(bytes(decoded), decoded.size(), enc, sizeof(enc));
    TEST_ASSERT_EQUAL_STRING_MESSAGE(cases.at(i)["encoded"].string.c_str(), to_hex(enc, n).c_str(),
                                     label.c_str());
    const std::string encoded = from_hex(cases.at(i)["encoded"].string);
    uint8_t dec[600];
    size_t m = 0;
    TEST_ASSERT_TRUE_MESSAGE(cobs_decode(bytes(encoded), encoded.size(), dec, sizeof(dec), &m),
                             label.c_str());
    TEST_ASSERT_EQUAL_STRING_MESSAGE(cases.at(i)["decoded"].string.c_str(), to_hex(dec, m).c_str(),
                                     label.c_str());
  }
}

static void test_message_vectors_encode() {
  const Value& cases = g_vectors["messages"];
  for (size_t i = 0; i < cases.size(); ++i) {
    const Value& c = cases.at(i);
    Message m = from_fields(c["message"].string, c["fields"]);
    m.seq = static_cast<uint8_t>(c["seq"].number);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(c["type"].number), static_cast<int>(m.type));
    uint8_t packet[kMaxPacketSize];
    const size_t np = encode_packet(m, packet, sizeof(packet));
    TEST_ASSERT_EQUAL_STRING_MESSAGE(c["packet"].string.c_str(), to_hex(packet, np).c_str(),
                                     c["name"].string.c_str());
    uint8_t frame[kMaxFrameSize];
    const size_t nf = encode_frame(m, frame, sizeof(frame));
    TEST_ASSERT_EQUAL_STRING_MESSAGE(c["frame"].string.c_str(), to_hex(frame, nf).c_str(),
                                     c["name"].string.c_str());
  }
}

static void test_message_vectors_decode() {
  const Value& cases = g_vectors["messages"];
  FrameReceiver rx;
  for (size_t i = 0; i < cases.size(); ++i) {
    const Value& c = cases.at(i);
    const std::string frame = from_hex(c["frame"].string);
    Message out;
    bool got = false;
    for (char ch : frame) {
      got = rx.push(static_cast<uint8_t>(ch), &out);
    }
    TEST_ASSERT_TRUE_MESSAGE(got, c["name"].string.c_str());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(c["type"].number), static_cast<int>(out.type));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(c["seq"].number), out.seq);
    check_fields(out, c["message"].string, c["fields"], c["name"].string);
  }
  TEST_ASSERT_EQUAL_UINT32(cases.size(), rx.frames_ok());
  TEST_ASSERT_EQUAL_UINT32(0, rx.crc_errors());
  TEST_ASSERT_EQUAL_UINT32(0, rx.frame_errors());
}

static DecodeStatus status_for(const std::string& kind) {
  if (kind == "crc") return DecodeStatus::kBadCrc;
  if (kind == "version") return DecodeStatus::kBadVersion;
  if (kind == "type") return DecodeStatus::kUnknownType;
  if (kind == "length") return DecodeStatus::kBadLength;
  if (kind == "short") return DecodeStatus::kTooShort;
  return DecodeStatus::kOk;
}

static void test_invalid_vectors() {
  const Value& cases = g_vectors["invalid"];
  TEST_ASSERT_TRUE(cases.size() > 0);
  FrameReceiver rx;
  for (size_t i = 0; i < cases.size(); ++i) {
    const Value& c = cases.at(i);
    const std::string packet = from_hex(c["packet"].string);
    Message out;
    const DecodeStatus status = decode_packet(bytes(packet), packet.size(), &out);
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(status_for(c["error"].string)),
                                  static_cast<int>(status), c["name"].string.c_str());
    const std::string frame = from_hex(c["frame"].string);
    for (char ch : frame) {
      TEST_ASSERT_FALSE(rx.push(static_cast<uint8_t>(ch), &out));
    }
  }
  TEST_ASSERT_EQUAL_UINT32(cases.size(), rx.crc_errors() + rx.frame_errors());
  TEST_ASSERT_EQUAL_UINT32(1, rx.crc_errors());
}

int main(int, char**) {
  load_vectors();
  UNITY_BEGIN();
  RUN_TEST(test_file_loaded);
  RUN_TEST(test_crc_vectors);
  RUN_TEST(test_cobs_vectors);
  RUN_TEST(test_message_vectors_encode);
  RUN_TEST(test_message_vectors_decode);
  RUN_TEST(test_invalid_vectors);
  return UNITY_END();
}
