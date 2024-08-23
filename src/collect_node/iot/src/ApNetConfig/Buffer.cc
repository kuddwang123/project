#include "Buffer.h"
#include "log.h"
#include <sys/socket.h>
#include <cassert>
namespace collect_node_iot {
Buffer::Buffer():
    readIndex_(0),
    writeIndex_(0)
{
    readBuf_.assign(DEF_BUFSIZE, 0);
    writeBuf_.assign(DEF_BUFSIZE, 0);
}

void Buffer::read(const char* data, size_t len)
{
    std::unique_lock<std::mutex> lock(readmtx_);
    if (readBuf_.size()-readIndex_ < len) {
        HJ_ERROR("read buffer too large, discard!\n");
        clearReadBuf();
        return;
    }

    std::copy(data, data+len, readBuf_.begin()+readIndex_);
    readIndex_ += len;
}

void Buffer::clearReadBuf()
{
    std::unique_lock<std::mutex> lock(readmtx_);
    readBuf_.assign(readBuf_.size(), 0);
    readIndex_ = 0;
}

bool Buffer::isReadDone()
{
    assert(readIndex_ > 0);
    return readBuf_[readIndex_-1] == '\n';
}

const uint8_t* Buffer::writeBegin()
{
    std::unique_lock<std::mutex> lock(writemtx_);
    return writeBuf_.data();
}

const size_t Buffer::writeIndex()
{
    std::unique_lock<std::mutex> lock(writemtx_);
    return writeIndex_;
}

std::string Buffer::readData()
{
    return std::string(readBuf_.begin(), readBuf_.begin()+readIndex_);
}

void Buffer::write(const char* data, size_t len)
{
    std::unique_lock<std::mutex> lock(writemtx_);
    if (writeBuf_.size()-writeIndex_ < len) {
        HJ_INFO("trigger write size enlarge!\n");
        writeBuf_.resize(writeBuf_.size() + 2*len);
    }

    std::copy(data, data+len, writeBuf_.begin()+writeIndex_);
    writeIndex_ += len;
}

void Buffer::shrinkWriteBuf(size_t index)
{
    assert(index <= writeIndex_);
    std::unique_lock<std::mutex> lock(readmtx_);
    
    if (index == writeIndex_) {
        writeBuf_.assign(writeBuf_.size(), 0);
        writeIndex_ = 0;
    } else {
        size_t origin_size = writeBuf_.size();
        size_t newIndex = writeIndex_ - index;
        std::vector<uint8_t> tmp(origin_size, 0);
        std::copy(writeBuf_.begin()+index, writeBuf_.begin()+writeIndex_, tmp.begin());
        std::swap(writeBuf_, tmp);
        writeIndex_ = newIndex;
    } 
}

} //namespace collect_node_iot