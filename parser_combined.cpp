#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <optional>

// Approach 3: Combined template + type ID
// Parse first, then cast to specific type based on marker

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

struct Command {
    uint8_t cmdId;
    uint32_t param;
};
#pragma pack(pop)

// Marker constants
constexpr uint8_t MARKER_OBJECT  = 0x42;
constexpr uint8_t MARKER_SENSOR  = 0x43;
constexpr uint8_t MARKER_COMMAND = 0x44;

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
    CHECKSUM_MISMATCH,
    SIZE_MISMATCH,
    UNKNOWN_MARKER
};

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
                } else if (byte == 0x55) {
                    // Stay
                } else {
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

    // Get marker to determine type
    uint8_t getMarker() const { return marker; }
    
    // Get raw data buffer
    const std::vector<uint8_t>& getRawData() const { return dataBuffer; }

    // Template method to interpret data as specific type
    template<typename T>
    std::vector<T> getAs() const {
        std::vector<T> result;
        if (dataBuffer.size() % sizeof(T) != 0) {
            return result;  // Size mismatch
        }
        
        size_t count = dataBuffer.size() / sizeof(T);
        for (size_t i = 0; i < count; ++i) {
            T obj;
            std::memcpy(&obj, &dataBuffer[i * sizeof(T)], sizeof(T));
            result.push_back(obj);
        }
        return result;
    }

    // Convenience method to get single item (for commands, etc.)
    template<typename T>
    std::optional<T> getFirstAs() const {
        if (dataBuffer.size() < sizeof(T)) {
            return std::nullopt;
        }
        T obj;
        std::memcpy(&obj, dataBuffer.data(), sizeof(T));
        return obj;
    }

    bool hasError() const { return lastError != ParseError::NONE; }
    ParseError getError() const { return lastError; }
    bool isComplete() const { return state == State::FRAME_COMPLETE; }
};

uint8_t calcChecksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; ++i) cs += data[i];
    return cs;
}

// Helper to process frames based on marker
void processFrame(FrameParser& parser) {
    if (parser.hasError()) {
        std::cout << "  Error!" << std::endl;
        return;
    }

    switch (parser.getMarker()) {
        case MARKER_OBJECT: {
            auto objects = parser.getAs<Object>();
            std::cout << "  Received " << objects.size() << " Object(s):" << std::endl;
            for (const auto& obj : objects) {
                std::cout << "    id=" << (int)obj.id
                          << " pos=" << obj.position
                          << " vel=" << obj.velocity
                          << " acc=" << obj.acceleration << std::endl;
            }
            break;
        }
        case MARKER_SENSOR: {
            auto sensors = parser.getAs<Sensor>();
            std::cout << "  Received " << sensors.size() << " Sensor(s):" << std::endl;
            for (const auto& s : sensors) {
                std::cout << "    id=" << (int)s.sensorId
                          << " temp=" << s.temperature
                          << " humidity=" << s.humidity << std::endl;
            }
            break;
        }
        case MARKER_COMMAND: {
            auto cmd = parser.getFirstAs<Command>();
            if (cmd) {
                std::cout << "  Received Command: id=" << (int)cmd->cmdId
                          << " param=" << cmd->param << std::endl;
            }
            break;
        }
        default:
            std::cout << "  Unknown marker: 0x" << std::hex << (int)parser.getMarker() 
                      << std::dec << std::endl;
    }
}

int main() {
    std::cout << "=== Combined Parser Demo ===" << std::endl;

    FrameParser parser;

    // Test 1: Object frame
    std::cout << "\nFrame 1 (Objects):" << std::endl;
    uint8_t objData[] = {
        MARKER_OBJECT,
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,  // Object 1
        0x02, 0xC8, 0x00, 0x64, 0x00, 0x14, 0x00   // Object 2
    };
    std::vector<uint8_t> frame1 = {
        0x55, 0xAA, 0x0F, 0x00,
        MARKER_OBJECT,
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,
        0x02, 0xC8, 0x00, 0x64, 0x00, 0x14, 0x00,
        calcChecksum(objData, sizeof(objData))
    };
    for (uint8_t b : frame1) {
        if (parser.feedByte(b)) processFrame(parser);
    }

    // Test 2: Sensor frame
    parser.reset();
    std::cout << "\nFrame 2 (Sensor):" << std::endl;
    uint8_t sensorData[] = {MARKER_SENSOR, 0x05, 0xE8, 0x03, 0x32, 0x00};
    std::vector<uint8_t> frame2 = {
        0x55, 0xAA, 0x06, 0x00,
        MARKER_SENSOR, 0x05, 0xE8, 0x03, 0x32, 0x00,
        calcChecksum(sensorData, sizeof(sensorData))
    };
    for (uint8_t b : frame2) {
        if (parser.feedByte(b)) processFrame(parser);
    }

    // Test 3: Command frame
    parser.reset();
    std::cout << "\nFrame 3 (Command):" << std::endl;
    uint8_t cmdData[] = {MARKER_COMMAND, 0x01, 0x10, 0x27, 0x00, 0x00};  // cmd=1, param=10000
    std::vector<uint8_t> frame3 = {
        0x55, 0xAA, 0x06, 0x00,
        MARKER_COMMAND, 0x01, 0x10, 0x27, 0x00, 0x00,
        calcChecksum(cmdData, sizeof(cmdData))
    };
    for (uint8_t b : frame3) {
        if (parser.feedByte(b)) processFrame(parser);
    }

    // Test 4: Unknown marker
    parser.reset();
    std::cout << "\nFrame 4 (Unknown):" << std::endl;
    uint8_t unkData[] = {0xFF, 0x01, 0x02, 0x03};
    std::vector<uint8_t> frame4 = {
        0x55, 0xAA, 0x04, 0x00,
        0xFF, 0x01, 0x02, 0x03,
        calcChecksum(unkData, sizeof(unkData))
    };
    for (uint8_t b : frame4) {
        if (parser.feedByte(b)) processFrame(parser);
    }

    return 0;
}

