#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/prctl.h>
#include <termios.h>
#include <unistd.h>
#include "i2c.h"

namespace hj_bf {

I2C::I2C(std::string dev): dev_(dev), fd_(-1) {}
I2C::I2C(const I2C& other) {
  dev_ = other.dev_;
  fd_ = other.fd_;
}
I2C& I2C::operator =(const I2C& other) {
  if (this == &other) {
    return *this;
  }
  this->dev_ = other.dev_;
  fd_ = other.fd_;
  return *this;
}

I2C::~I2C() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool I2C::Initialize() {
  if (fd_ > 0) {
    HJ_INFO("device %s already opened", dev_.data());
    return true;
  }
  fd_ = open(dev_.data(), O_RDWR);
  if (fd_ < 0) {
    HJ_ERROR("can not open file %s", dev_.data());
    return false;
  }
  return true;
}

bool I2C::I2cWriteBytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len) {
  bool ret = false;
  uint8_t *outbuf = nullptr;
  if (len > 1) {
    outbuf = new uint8_t[len + 1]();
  } else {
    outbuf = write_buf;
  }

  if (outbuf == nullptr) {
    HJ_ERROR("Error: No memory for buffer");
    return false;
  }

  outbuf[0] = reg_addr;
  memcpy(outbuf + 1, values, len);

  messages[0].addr = slave_addr;
  messages[0].flags = 0;
  messages[0].len = len + 1;
  messages[0].buf = outbuf;

  /* Transfer the i2c packets_ to the kernel and verify it worked */
  packets_.msgs = messages;
  packets_.nmsgs = 1;
  if (ioctl(fd_, I2C_RDWR, &packets_) < 0) {
    if (write_error_count_ == 0) {
      HJ_ERROR("func:%s, line:%d, Error: Unable to send data", __FUNCTION__, __LINE__);
    }
    write_error_count_ = 1;
  } else {
    ret = true;
    write_error_count_ = 0;
  }

  if (len > 1) {
    delete [] outbuf;
    outbuf = nullptr;
  }

  return ret;
}

bool I2C::I2cReadBytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *values, uint8_t len) {
  read_buf[0] = reg_addr;
  messages[0].addr = slave_addr;
  messages[0].flags = 0;
  messages[0].len = sizeof(read_buf);
  messages[0].buf = read_buf;

  /* The data will get returned in this structure */
  messages[1].addr = slave_addr;
  messages[1].flags = I2C_M_RD /* | I2C_M_NOSTART*/;
  messages[1].len = len;
  messages[1].buf = values;

  /* Send the request to the kernel and get the result back */
  packets_.msgs = messages;
  packets_.nmsgs = 2;
  if (ioctl(fd_, I2C_RDWR, &packets_) < 0) {
    if (read_error_count_ == 0) {
      HJ_ERROR("FUNC:%s, line:%d, Error: Unable to send data", __FUNCTION__, __LINE__);
    }
    read_error_count_ = 1;
    return false;
  } else {
    read_error_count_ = 0;
    return true;
  }
}
}  // namespace hj_bf
