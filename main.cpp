#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>

#pragma pack(push,1)
struct Object {
  uint8_t id;
  uint16_t position;
  uint16_t velocity;
  uint16_t acceleration;
};
#pragma pack(pop)

// Parser states
enum class State {
    WAIT_MAGIC_1,       // Waiting for 0x55
    WAIT_MAGIC_2,       // Waiting for 0xAA
    READ_SIZE_LOW,      // Reading low byte of data size
    READ_SIZE_HIGH,     // Reading high byte of data size
    READ_DATA_MARKER,   // Waiting for 0x42 marker
    READ_OBJECTS,       // Reading object data
    READ_CHECKSUM,      // Reading checksum byte (last byte of frame)
    FRAME_COMPLETE,     // Frame fully parsed
    ERROR               // Error state
};

// Error types
enum class ParseError {
    NONE,
    INVALID_MARKER,
    CHECKSUM_MISMATCH,
    SIZE_MISMATCH       // Data size not aligned to Object size
};

class FrameParser {
private:
    State state = State::WAIT_MAGIC_1;
    
    uint16_t dataSize = 0;
    uint16_t bytesRead = 0;
    uint8_t checksum = 0;
    uint8_t calculatedChecksum = 0;

    std::vector<uint8_t> dataBuffer;
    std::vector<Object> parsedObjects;
    ParseError lastError = ParseError::NONE;

public:
    // Feed one byte at a time, returns true when frame is complete
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
                    // Stay in case of consecutive 0x55
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
                if (byte == 0x42) {
                    dataBuffer.clear();
                    dataBuffer.reserve(dataSize - 1); // -1 for the 0x42 marker
                    bytesRead = 1; // Count the 0x42 marker
                    calculatedChecksum = byte; // Start checksum with marker
                    state = State::READ_OBJECTS;
                } else {
                    // Invalid marker
                    lastError = ParseError::INVALID_MARKER;
                    state = State::ERROR;
                    return true;
                }
                break;

            case State::READ_OBJECTS:
                dataBuffer.push_back(byte);
                calculatedChecksum += byte; // Sum checksum (last byte)
                bytesRead++;
                
                if (bytesRead >= dataSize) {
                    // Done reading data, now read checksum
                    state = State::READ_CHECKSUM;
                }
                break;

            case State::READ_CHECKSUM: {
                checksum = byte;
                
                // Validate checksum
                if (calculatedChecksum != checksum) {
                    lastError = ParseError::CHECKSUM_MISMATCH;
                    state = State::ERROR;
                    return true;
                }
                
                // Validate size alignment (must fit whole Objects)
                size_t objectDataSize = dataBuffer.size();
                if (objectDataSize % sizeof(Object) != 0) {
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
                // Waiting for reset
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

    const std::vector<Object>& getObjects() const {
        return parsedObjects;
    }

    uint8_t getChecksum() const {
        return checksum;
    }

    uint8_t getCalculatedChecksum() const {
        return calculatedChecksum;
    }

    State getState() const {
        return state;
    }

    ParseError getError() const {
        return lastError;
    }

    bool hasError() const {
        return lastError != ParseError::NONE;
    }

    const char* getErrorString() const {
        switch (lastError) {
            case ParseError::NONE:
                return "No error";
            case ParseError::INVALID_MARKER:
                return "Invalid data marker (expected 0x42)";
            case ParseError::CHECKSUM_MISMATCH:
                return "Checksum mismatch";
            case ParseError::SIZE_MISMATCH:
                return "Data size not aligned to Object size";
            default:
                return "Unknown error";
        }
    }

private:
    void parseObjects() {
        parsedObjects.clear();
        constexpr size_t objectSize = sizeof(Object);
        
        size_t numObjects = dataBuffer.size() / objectSize;
        
        for (size_t i = 0; i < numObjects; ++i) {
            Object obj;
            size_t offset = i * objectSize;
            
            obj.id = dataBuffer[offset];
            std::memcpy(&obj.position, &dataBuffer[offset + 1], sizeof(uint16_t));
            std::memcpy(&obj.velocity, &dataBuffer[offset + 3], sizeof(uint16_t));
            std::memcpy(&obj.acceleration, &dataBuffer[offset + 5], sizeof(uint16_t));
            
            parsedObjects.push_back(obj);
        }
    }
};

// Helper to calculate sum checksum for test data (last byte of sum)
uint8_t calcChecksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; ++i) {
        cs += data[i];
    }
    return cs;
}

void printResult(FrameParser& parser) {
    if (parser.hasError()) {
        std::cout << "ERROR: " << parser.getErrorString() << std::endl;
        std::cout << "  Expected checksum: 0x" << std::hex << (int)parser.getChecksum() 
                  << ", Calculated: 0x" << (int)parser.getCalculatedChecksum() << std::dec << std::endl;
    } else {
        std::cout << "Frame OK!" << std::endl;
        std::cout << "  Checksum: 0x" << std::hex << (int)parser.getChecksum() << std::dec << std::endl;
        std::cout << "  Objects parsed: " << parser.getObjects().size() << std::endl;
        
        for (const auto& obj : parser.getObjects()) {
            std::cout << "    Object #" << (int)obj.id 
                      << ": pos=" << obj.position
                      << ", vel=" << obj.velocity
                      << ", acc=" << obj.acceleration << std::endl;
        }
    }
}

int main_default() {
    FrameParser parser;

    // Test 1: Valid frame with correct checksum
    std::cout << "=== Test 1: Valid frame ===" << std::endl;
    // Data: 0x42 (marker) + Object bytes
    uint8_t data1[] = {0x42, 0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00};
    uint8_t checksum1 = calcChecksum(data1, sizeof(data1));
    
    std::vector<uint8_t> testData1 = {
        0x55, 0xAA,             // Magic
        0x08, 0x00,             // Size = 8
        0x42,                   // Data marker
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,  // Object
        checksum1               // Checksum (last byte)
    };

    for (uint8_t byte : testData1) {
        if (parser.feedByte(byte)) {
            printResult(parser);
        }
    }

    // Test 2: Wrong checksum
    std::cout << "\n=== Test 2: Checksum mismatch ===" << std::endl;
    parser.reset();
    
    std::vector<uint8_t> testData2 = {
        0x55, 0xAA,             // Magic
        0x08, 0x00,             // Size = 8
        0x42,                   // Data marker
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,  // Object
        0xFF                    // WRONG checksum (last byte)
    };

    for (uint8_t byte : testData2) {
        if (parser.feedByte(byte)) {
            printResult(parser);
        }
    }

    // Test 3: Size mismatch (data not aligned to Object size)
    std::cout << "\n=== Test 3: Size mismatch ===" << std::endl;
    parser.reset();
    
    uint8_t data3[] = {0x42, 0x01, 0x64, 0x00, 0x32}; // Only 4 bytes after marker (not 7)
    uint8_t checksum3 = calcChecksum(data3, sizeof(data3));
    
    std::vector<uint8_t> testData3 = {
        0x55, 0xAA,             // Magic
        0x05, 0x00,             // Size = 5 (1 marker + 4 bytes - incomplete object!)
        0x42,                   // Data marker
        0x01, 0x64, 0x00, 0x32, // Incomplete object data
        checksum3               // Checksum (last byte)
    };

    for (uint8_t byte : testData3) {
        if (parser.feedByte(byte)) {
            printResult(parser);
        }
    }

    // Test 4: Invalid data marker
    std::cout << "\n=== Test 4: Invalid marker ===" << std::endl;
    parser.reset();
    
    std::vector<uint8_t> testData4 = {
        0x55, 0xAA,             // Magic
        0x08, 0x00,             // Size
        0x99,                   // WRONG marker (expected 0x42)
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,
        0x12                    // Checksum (last byte)
    };

    for (uint8_t byte : testData4) {
        if (parser.feedByte(byte)) {
            printResult(parser);
        }
    }

    // Test 5: Valid frame with 2 objects
    std::cout << "\n=== Test 5: Valid frame with 2 objects ===" << std::endl;
    parser.reset();
    
    uint8_t data5[] = {
        0x42,  // marker
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,  // Object 1
        0x02, 0xC8, 0x00, 0x64, 0x00, 0x14, 0x00   // Object 2
    };
    uint8_t checksum5 = calcChecksum(data5, sizeof(data5));
    
    std::vector<uint8_t> testData5 = {
        0x55, 0xAA,             // Magic
        0x0F, 0x00,             // Size = 15 (1 marker + 14 bytes)
        0x42,                   // Data marker
        0x01, 0x64, 0x00, 0x32, 0x00, 0x0A, 0x00,
        0x02, 0xC8, 0x00, 0x64, 0x00, 0x14, 0x00,
        checksum5               // Checksum (last byte)
    };

    for (uint8_t byte : testData5) {
        if (parser.feedByte(byte)) {
            printResult(parser);
        }
    }

    return 0;
}

int main_callback();
int main_template();
int main_combined();

int main() {
  //main_default();
  main_callback();
  //main_template();
  //main_combined();
}