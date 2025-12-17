#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>

// Approach 1: Template-based parser
// The struct type is known at compile time

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
  READ_OBJECTS,
  READ_CHECKSUM,
  FRAME_COMPLETE,
  ERROR
};

enum class ParseError {
  NONE,
  INVALID_MARKER,
  CHECKSUM_MISMATCH,
  SIZE_MISMATCH
};

template<typename T, uint8_t Marker = 0x42>
class FrameParser {
private:
  State state = State::WAIT_MAGIC_1;

  uint16_t dataSize = 0;
  uint16_t bytesRead = 0;
  uint8_t checksum = 0;
  uint8_t calculatedChecksum = 0;

  std::vector<uint8_t> dataBuffer;
  std::vector<T> parsedObjects;
  ParseError lastError = ParseError::NONE;

public:
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
        // Stay in case of consecutive 0x55
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
      if (byte == Marker) {
        dataBuffer.clear();
        dataBuffer.reserve(dataSize - 1);
        bytesRead = 1;
        calculatedChecksum = byte;
        state = State::READ_OBJECTS;
      }
      else {
        lastError = ParseError::INVALID_MARKER;
        state = State::ERROR;
        return true;
      }
      break;

    case State::READ_OBJECTS:
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

      if (dataBuffer.size() % sizeof(T) != 0) {
        lastError = ParseError::SIZE_MISMATCH;
        state = State::ERROR;
        return true;
      }

      parseObjects();
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
    checksum = 0;
    calculatedChecksum = 0;
    bytesRead = 0;
    dataBuffer.clear();
    parsedObjects.clear();
    lastError = ParseError::NONE;
  }

  const std::vector<T>& getObjects() const { return parsedObjects; }
  bool hasError() const { return lastError != ParseError::NONE; }
  ParseError getError() const { return lastError; }

private:
  void parseObjects() {
    parsedObjects.clear();
    size_t count = dataBuffer.size() / sizeof(T);

    for (size_t i = 0; i < count; ++i) {
      T obj;
      std::memcpy(&obj, &dataBuffer[i * sizeof(T)], sizeof(T));
      parsedObjects.push_back(obj);
    }
  }
};

// Helper to calculate checksum
uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; ++i) cs += data[i];
  return cs;
}

int main() {
  std::cout << "=== Template Parser Demo ===" << std::endl;

  // Parser for Object structs (marker 0x42)
  FrameParser<Object, 0x42> objectParser;

  uint8_t objData[] = { 0x42, 0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00 };
  std::vector<uint8_t> frame1 = {
      0x55, 0xAA,
      0x08, 0x00,
      0x42,
      0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,
      calcChecksum(objData, sizeof(objData))
  };

  std::cout << "\nParsing Object frame:" << std::endl;
  for (uint8_t b : frame1) {
    if (objectParser.feedByte(b)) {
      if (!objectParser.hasError()) {
        for (const auto& obj : objectParser.getObjects()) {
          std::cout << "  Object id=" << (int)obj.id
            << " pos=" << obj.position
            << " vel=" << obj.velocity
            << " acc=" << obj.acceleration << std::endl;
        }
      }
    }
  }

  // Parser for Sensor structs (marker 0x43)
  FrameParser<Sensor, 0x43> sensorParser;

  uint8_t sensorData[] = { 0x43, 0x05, 0xE8, 0x03, 0x32, 0x00 };  // id=5, temp=1000, humidity=50
  std::vector<uint8_t> frame2 = {
      0x55, 0xAA,
      0x06, 0x00,
      0x43,
      0x05, 0xE8, 0x03, 0x32, 0x00,
      calcChecksum(sensorData, sizeof(sensorData))
  };

  std::cout << "\nParsing Sensor frame:" << std::endl;
  for (uint8_t b : frame2) {
    if (sensorParser.feedByte(b)) {
      if (!sensorParser.hasError()) {
        for (const auto& s : sensorParser.getObjects()) {
          std::cout << "  Sensor id=" << (int)s.sensorId
            << " temp=" << s.temperature
            << " humidity=" << s.humidity << std::endl;
        }
      }
    }
  }

  return 0;
}

