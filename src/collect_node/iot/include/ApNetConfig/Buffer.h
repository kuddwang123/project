#pragma once
#include <cstdint>
#include <vector>
#include <mutex>

namespace collect_node_iot {
class Buffer {
public:
    Buffer();
    ~Buffer() {};
    void read(const char* data, size_t len);
    void write(const char* data, size_t len);
    const uint8_t* writeBegin();
    const size_t writeIndex();
    void clearReadBuf();
    bool isReadDone();
    std::string readData();
    void shrinkWriteBuf(size_t index);

public:
    static const int DEF_BUFSIZE = 1024 * 4; //4K buffer

private:
    size_t readIndex_;
    size_t writeIndex_;
    std::vector<uint8_t> readBuf_;
    std::vector<uint8_t> writeBuf_;
    std::mutex readmtx_;
    std::mutex writemtx_;

private:
    
};
} //namespace collect_node_iot