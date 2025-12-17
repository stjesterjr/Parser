#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <functional>

// Approach 2: Callback-based parser
// Raw data is passed to a callback for flexible handling

#pragma pack(push, 1)
struct Object {
  uint8_t id;
  uint16_t position;
  uint16_t velocity;
  uint16_t acceleration;
};

struct Sensor {
  uint8_t sensorId;
  int16_t temperature;
  uint16_t humidity;
};
#pragma pack(pop)

enum class State {
  WAIT_MAGIC_1,
  WAIT_MAGIC_2,
  READ_SIZE_LOW,
  READ_SIZE_HIGH,
  READ_DATA_MARKER,
  READ_DATA,
  READ_CHECKSUM,
  FRAME_COMPLETE,
  ERROR
};

enum class ParseError {
  NONE,
  CHECKSUM_MISMATCH
};

// Callback signature: (marker, data_ptr, data_len)
using FrameCallback = std::function<void(uint8_t, const uint8_t*, size_t)>;

class FrameParser {
private:
  State state = State::WAIT_MAGIC_1;

  uint16_t dataSize = 0;
  uint16_t bytesRead = 0;
  uint8_t marker = 0;
  uint8_t checksum = 0;
  uint8_t calculatedChecksum = 0;

  std::vector<uint8_t> dataBuffer;
  ParseError lastError = ParseError::NONE;
  FrameCallback callback;

public:
  void setCallback(FrameCallback cb) {
    callback = std::move(cb);
  }

  bool feedByte(uint8_t byte) {
    switch (state) {
    case State::WAIT_MAGIC_1:
      if (byte == 0x55) {
        state = State::WAIT_MAGIC_2;
      }
      break;

    case State::WAIT_MAGIC_2:
      if (byte == 0xAA) {
        state = State::READ_SIZE_LOW;
      }
      else if (byte == 0x55) {
        // Stay
      }
      else {
        state = State::WAIT_MAGIC_1;
      }
      break;

    case State::READ_SIZE_LOW:
      dataSize = byte;
      state = State::READ_SIZE_HIGH;
      break;

    case State::READ_SIZE_HIGH:
      dataSize |= (static_cast<uint16_t>(byte) << 8);
      state = State::READ_DATA_MARKER;
      break;

    case State::READ_DATA_MARKER:
      marker = byte;
      dataBuffer.clear();
      dataBuffer.reserve(dataSize - 1);
      bytesRead = 1;
      calculatedChecksum = byte;
      state = State::READ_DATA;
      break;

    case State::READ_DATA:
      dataBuffer.push_back(byte);
      calculatedChecksum += byte;
      bytesRead++;

      if (bytesRead >= dataSize) {
        state = State::READ_CHECKSUM;
      }
      break;

    case State::READ_CHECKSUM: {
      checksum = byte;

      if (calculatedChecksum != checksum) {
        lastError = ParseError::CHECKSUM_MISMATCH;
        state = State::ERROR;
        return true;
      }

      // Invoke callback with raw data
      if (callback) {
        callback(marker, dataBuffer.data(), dataBuffer.size());
      }

      lastError = ParseError::NONE;
      state = State::FRAME_COMPLETE;
      return true;
    }

    case State::FRAME_COMPLETE:
    case State::ERROR:
      break;
    }
    return false;
  }

  void reset() {
    state = State::WAIT_MAGIC_1;
    dataSize = 0;
    marker = 0;
    checksum = 0;
    calculatedChecksum = 0;
    bytesRead = 0;
    dataBuffer.clear();
    lastError = ParseError::NONE;
  }

  uint8_t getMarker() const { return marker; }
  bool hasError() const { return lastError != ParseError::NONE; }
  ParseError getError() const { return lastError; }
};

// Helper to parse structs from raw data
template<typename T>
std::vector<T> parseAs(const uint8_t* data, size_t len) {
  std::vector<T> result;
  size_t count = len / sizeof(T);
  for (size_t i = 0; i < count; ++i) {
    T obj;
    std::memcpy(&obj, data + i * sizeof(T), sizeof(T));
    result.push_back(obj);
  }
  return result;
}

static uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i)
    cs += data[i];
  return cs;
}

int main_callback() {
  std::cout << "=== Callback Parser Demo ===" << std::endl;

  FrameParser parser;

  // Set callback to handle different markers
  parser.setCallback([](uint8_t marker, const uint8_t* data, size_t len) {
    std::cout << "Received frame with marker 0x" << std::hex << (int)marker
      << std::dec << ", " << len << " bytes" << std::endl;

    if (marker == 0x42) {
      auto objects = parseAs<Object>(data, len);
      for (const auto& obj : objects) {
        std::cout << "  Object id=" << (int)obj.id
          << " pos=" << obj.position
          << " vel=" << obj.velocity
          << " acc=" << obj.acceleration << std::endl;
      }
    }
    else if (marker == 0x43) {
      auto sensors = parseAs<Sensor>(data, len);
      for (const auto& s : sensors) {
        std::cout << "  Sensor id=" << (int)s.sensorId
          << " temp=" << s.temperature
          << " humidity=" << s.humidity << std::endl;
      }
    }
    else {
      std::cout << "  Unknown marker, raw bytes: ";
      for (size_t i = 0; i < len; ++i) {
        std::cout << std::hex << (int)data[i] << " ";
      }
      std::cout << std::dec << std::endl;
    }
    });

  // Test with Object frame (marker 0x42)
  std::cout << "\nSending Object frame:" << std::endl;
  uint8_t objData[] = { 0x42, 0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00 };
  std::vector<uint8_t> frame1 = {
      0x55, 0xAA, 0x08, 0x00,
      0x42, 0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,
      calcChecksum(objData, sizeof(objData))
  };
  for (uint8_t b : frame1) parser.feedByte(b);

  // Test with Sensor frame (marker 0x43)
  parser.reset();
  std::cout << "\nSending Sensor frame:" << std::endl;
  uint8_t sensorData[] = { 0x43, 0x05, 0xE8, 0x03, 0x32, 0x00 };
  std::vector<uint8_t> frame2 = {
      0x55, 0xAA, 0x06, 0x00,
      0x43, 0x05, 0xE8, 0x03, 0x32, 0x00,
      calcChecksum(sensorData, sizeof(sensorData))
  };
  for (uint8_t b : frame2) parser.feedByte(b);

  // Test with unknown marker
  parser.reset();
  std::cout << "\nSending unknown frame (marker 0x99):" << std::endl;
  uint8_t unknownData[] = { 0x99, 0xDE, 0xAD, 0xBE, 0xEF };
  std::vector<uint8_t> frame3 = {
      0x55, 0xAA, 0x05, 0x00,
      0x99, 0xDE, 0xAD, 0xBE, 0xEF,
      calcChecksum(unknownData, sizeof(unknownData))
  };
  for (uint8_t b : frame3) parser.feedByte(b);

  return 0;
}

