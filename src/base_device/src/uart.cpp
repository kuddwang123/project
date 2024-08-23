// @file uart.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-15)

#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/prctl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "uart.h"
#include "log.h"

namespace hj_bf {
Uart::Uart(std::string dev)
    : fd_(-1), dev_(dev), initFlag_(false),
    runFlag_(false), epoll_fd_(-1), timeout_sec_(2) {
  
  baudMap_[300] = B300;
  baudMap_[1200] = B1200;
  baudMap_[2400] = B2400;
  baudMap_[4800] = B4800;
  baudMap_[9600] = B9600;
  baudMap_[19200] = B19200;
  baudMap_[38400] = B38400;
  baudMap_[57600] = B57600;
  baudMap_[115200] = B115200;
  baudMap_[230400] = B230400;
  baudMap_[460800] = B460800;
  baudMap_[500000] = B500000;
  baudMap_[576000] = B576000;
  baudMap_[921600] = B921600;
  baudMap_[1000000] = B1000000;
}

Uart::~Uart() {close();}

void Uart::initialize(int baudrate, int flow_ctrl, int databits, int stopbits, int parity) {
  baud_rate_ = baudrate;
  flow_ctrl_ = flow_ctrl;
  databits_ = databits;
  stopbits_ = stopbits;
  parity_ = parity;

  initFlag_ = true;
}

bool Uart::run() {
  if (runFlag_) {
    HJ_INFO("uart %s is already running!\n", dev_.c_str());
    return true;
  }

  if (!initFlag_) {
    HJ_ERROR("uart is not initialized!\n");
    return false;
  }

  if (!open()) {
    HJ_ERROR("uart open failed!\n");
    return false;
  }

  if (!setuart()) {
    HJ_ERROR("uart set failed!\n");
    return false;
  }

  runFlag_ = true;
  return true;
}

bool Uart::open() {
  fd_ = ::open(dev_.data(), O_RDWR | O_NOCTTY | O_NDELAY);
  HJ_INFO("fd open %s: %d\n", dev_.data(), fd_);

  if (0 > fd_) {
    HJ_ERROR("Can't Open Serial Port");
    return false;
  }

  int ret = fcntl(fd_, F_SETFL, 0); //恢复串口为阻塞状态
  HJ_INFO("fcntl=%d\n", ret);

  if (0 > ret) {
    HJ_ERROR("fcntl failed!\n");
    return false;
  }

  epoll_fd_ = epoll_create(5);
  if (epoll_fd_ < 0) {
    HJ_ERROR("epoll create failed!\n");
    return false;
  }

  struct epoll_event ev;
  ev.events = EPOLLIN;  
  ev.data.fd = fd_;
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd_, &ev) != 0) {
      HJ_ERROR("epoll_ctl fail\n");
      return false;
  }
  return true;
}

void Uart::close() { 
  ::close(fd_); 
  epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd_, NULL);
  ::close(epoll_fd_);
  runFlag_ = false;
  fd_ = -1;
}

bool Uart::reset() {
  if (!initFlag_) {
    HJ_ERROR("no uart para set before!\n");
    return false;
  }

  close();

  if (!open()) {
    HJ_ERROR("reset uart open fail!\n");
    return false;
  }

  if (!setuart()) {
    HJ_ERROR("reset uart set fail!\n");
    return false;
  }

  runFlag_ = true;

  return true;
}

void Uart::flushout() {
  if(fd_ != -1) {
    tcflush(fd_, TCOFLUSH);
  }
}

void Uart::flushin() {
  if(fd_ != -1) {
    tcflush(fd_, TCIFLUSH);
  }
}

void Uart::flushinout() {
  if(fd_ != -1) {
    tcflush(fd_, TCIOFLUSH);
  }
}

bool Uart::setuart() {
  struct termios options;

  if (0 != tcgetattr(fd_, &options)) {
    HJ_ERROR("SetupSerial 1\n");
    return false;
  }

  //设置输入波特率和输出波特率
  if (baudMap_.find(baud_rate_) == baudMap_.end()) {
    HJ_ERROR("baud rate:%d not support!\n", baud_rate_);
    return false;
  }

  if (0 != cfsetispeed(&options, baudMap_[baud_rate_]) ||
      0 != cfsetospeed(&options, baudMap_[baud_rate_])) {
    HJ_ERROR("set baud rate fail:%d %s\n", baud_rate_, strerror(errno));
    return false;
  }
  
  //修改控制模式，保证程序不会占用串口
  options.c_cflag |= CLOCAL;
  //修改控制模式，使得能够从串口中读取输入数据
  options.c_cflag |= CREAD;

  //设置数据流控制
  switch (flow_ctrl_) {
  case 0: //不使用流控制
    options.c_cflag &= ~CRTSCTS;
    break;
  case 1: //使用硬件流控制
    options.c_cflag |= CRTSCTS;
    break;
  case 2: //使用软件流控制
    options.c_cflag |= IXON | IXOFF | IXANY;
    break;
  }
  //设置数据位
  //屏蔽其他标志位
  options.c_cflag &= ~CSIZE;
  switch (databits_) {
  case 5:
    options.c_cflag |= CS5;
    break;
  case 6:
    options.c_cflag |= CS6;
    break;
  case 7:
    options.c_cflag |= CS7;
    break;
  case 8:
    options.c_cflag |= CS8;
    break;
  default:
    HJ_ERROR("Unsupported data size\n");
    return false;
  }
  //设置校验位
  switch (parity_) {
  case 'n':
  case 'N': //无奇偶校验位。
    options.c_cflag &= ~PARENB;
    options.c_iflag &= ~INPCK;
    break;
  case 'o':
  case 'O': //设置为奇校验
    options.c_cflag |= (PARODD | PARENB);
    options.c_iflag |= INPCK;
    break;
  case 'e':
  case 'E': //设置为偶校验
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_iflag |= INPCK;
    break;
  case 's':
  case 'S': //设置为空格
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    break;
  default:
    HJ_ERROR("Unsupported parity\n");
    return false;
  }
  // 设置停止位
  switch (stopbits_) {
  case 1:
    options.c_cflag &= ~CSTOPB;
    break;
  case 2:
    options.c_cflag |= CSTOPB;
    break;
  default:
    HJ_ERROR("Unsupported stop bits\n");
    return false;
  }

  options.c_oflag &= ~OPOST;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~ICRNL;
  options.c_iflag &= ~(IXON);

  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 0;

  tcflush(fd_, TCIFLUSH);

  if (0 != tcsetattr(fd_, TCSANOW, &options)) {
    HJ_ERROR("uart set error:%s\n", strerror(errno));
    return false;
  }

  return true;
}

bool Uart::send(uint8_t *buf, int datalen) {
  if (!runFlag_) {
    HJ_ERROR("uart is not initialize!\n");
    return false;
  }

  int len = write(fd_, buf, datalen);
  if (len != datalen) {
    tcflush(fd_, TCOFLUSH);
    HJ_ERROR("write error!\n");
    return false;
  }

  return true;
}


int Uart::recv(uint8_t* buf) {
  int num_event = 0, i = 0, n = 0;
  num_event = epoll_wait(epoll_fd_, events_, MAX_EVENTS_NUM, timeout_sec_*1000);
  if (num_event < 0) {
    HJ_ERROR("epoll wait error:%s\n", strerror(errno));
    return -1;
  } else if(num_event == 0) {
    HJ_ERROR("epoll wait time out!\n");
    return 0;
  } else {
    for (i = 0; i < num_event; i++) {
        if ((events_[i].data.fd == fd_) &&
           (events_[i].events & EPOLLIN) != 0) {
            n = read(fd_, buf, MAX_PACKAGE_SIZE);
            if (0 >= n) {
              HJ_ERROR("epoll read failed %d\n", n);
              return 0;
            }
            return n;
        }
    }
  }

  HJ_ERROR("no epoll fd match!\n");
  return -1;
}


#if 0
int Uart::recv(uint8_t* buf) {
  uint8_t ret, i, n;
  fd_set readfds;
  struct timeval timeout;
  timeout.tv_sec = 2;  // 设置超时时间为2秒
  timeout.tv_usec = 0;

  FD_ZERO(&readfds);
  FD_SET(fd_, &readfds);
  ret = select(fd_ + 1, &readfds, NULL, NULL, &timeout);
  if (0 > ret) {
    HJ_ERROR("select failed\n");
    return -1;
  } else if (0 == ret) {
    HJ_INFO("select time out!\n");
    return 0;
  } else if (FD_ISSET(fd_, &readfds)) {
    n = read(fd_, buf, MAX_PACKAGE_SIZE);
    if (0 >= n) {
      HJ_ERROR("read failed %d\n", n);
      return 0;
    }
    return n;
  }

  return -1;
}
#endif

/*spi convert to uart*/
SpiUart::SpiUart(std::string dev_path): dev_(dev_path), fd_(-1) {}

SpiUart::~SpiUart() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool SpiUart::UartOpen() {
  if (fd_ > 0) {
    HJ_INFO("uart is already opened!\n");
    return true;
  }

  fd_ = open(dev_.data(), O_RDWR|O_NOCTTY|O_NDELAY);
  if (0 > fd_) {
    HJ_ERROR("Can't Open Serial Port %s", dev_.data());
    return false;
  }
  // 恢复串口为阻塞状态
  int ret = fcntl(fd_, F_SETFL, 0);
  if (0 > ret) {
    HJ_ERROR("fcntl failed!");
    return false;
  }

  return true;
}

bool SpiUart::UartSet(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
  int speed_arr[] = { B115200, B57600, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {115200, 57600, 19200, 9600, 4800, 2400, 1200, 300};

  struct termios options;

  /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
  该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
  if (0 != tcgetattr(fd_, &options)) {
    HJ_ERROR("SetupSerial 1");
    return false;
  }

  // 设置串口输入波特率和输出波特率
  for (u_int32_t i = 0;  i < sizeof(speed_arr) / sizeof(int); i++) {
    if (speed == name_arr[i]) {
      cfsetispeed(&options, speed_arr[i]);
      cfsetospeed(&options, speed_arr[i]);
    }
  }

  // 修改控制模式，保证程序不会占用串口
  options.c_cflag |= CLOCAL;
  // 修改控制模式，使得能够从串口中读取输入数据
  options.c_cflag |= CREAD;
  options.c_cflag &= ~CSIZE;

  // 设置数据流控制
  switch (flow_ctrl) {
  case 0:  // 不使用流控制
    options.c_cflag &= ~CRTSCTS;
    break;
  case 1:  // 使用硬件流控制
    options.c_cflag |= CRTSCTS;
    break;
  case 2:  // 使用软件流控制
    options.c_cflag |= IXON | IXOFF | IXANY;
    break;
  }
  // 设置数据位
  // 屏蔽其他标志位
  options.c_cflag &= ~CSIZE;
  switch (databits) {
  case 5:
    options.c_cflag |= CS5;
    break;
  case 6:
    options.c_cflag |= CS6;
    break;
  case 7:
    options.c_cflag |= CS7;
    break;
  case 8:
    options.c_cflag |= CS8;
    break;
  default:
    HJ_ERROR("Unsupported data size");
    return false;
  }
  // 设置校验位
  switch (parity) {
  case 'n':
  case 'N':  // 无奇偶校验位。
    options.c_cflag &= ~PARENB;
    options.c_iflag &= ~INPCK;
    break;
  case 'o':
  case 'O':  // 设置为奇校验
    options.c_cflag |= (PARODD | PARENB);
    options.c_iflag |= INPCK;
    break;
  case 'e':
  case 'E':  // 设置为偶校验
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_iflag |= INPCK;
    break;
  case 's':
  case 'S':  // 设置为空格
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    break;
  default:
    HJ_ERROR("Unsupported parity");
    return false;
  }
  // 设置停止位
  switch (stopbits) {
  case 1:
    options.c_cflag &= ~CSTOPB;
    break;
  case 2:
    options.c_cflag |= CSTOPB;
    break;
  default:
    HJ_ERROR("Unsupported stop bits");
    return false;
  }

  // 修改输出模式，原始数据输出
  options.c_oflag &= ~OPOST;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  // options.c_lflag &= ~(ISIG | ICANON);
  options.c_iflag &= ~ICRNL;   // 禁止将输入的CR转换为NL
  options.c_iflag &= ~(IXON);  // 清bit位 关闭流控字符
  // 设置等待时间和最小接收字符
  options.c_cc[VTIME] = 10; /* 读取一个字符等待1*(1/10)s */
  options.c_cc[VMIN] = 0; /* 读取字符的最少个数为1 */

  // 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
  tcflush(fd_, TCIFLUSH);

  // 激活配置 (将修改后的termios数据设置到串口中）
  if (0 != tcsetattr(fd_, TCSANOW, &options)) {
    HJ_ERROR("com set error!");
    return false;
  }

  return true;
}

/**
 * @brief spi convert to uart
 * 
 * @param enable 
 * @return bool 
 */
bool SpiUart::LibttyRs485Set(bool enable) {
  struct serial_rs485 rs485conf = {0};

  if (enable) {
    rs485conf.flags |= SER_RS485_ENABLED;
  } else {
    rs485conf.flags &= ~SER_RS485_ENABLED;
  }
  int ret = ioctl(fd_, TIOCSRS485, &rs485conf);
  if (0 > ret) {
    perror("ioctl TIOCSRS485 failed!,");
    HJ_ERROR("LibttyRs485Set failed!dev=%s", dev_.data());
    return false;
  }

  return true;
}

bool SpiUart::Initialize(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
  // open uart
  bool ret = UartOpen();
  if (ret) {
    // 设置串口数据帧格式
    ret = UartSet(speed, flow_ctrl, databits, stopbits, parity);
  } else {
    HJ_ERROR("fd_uls_tof_ uart open failed!");
  }

  return ret;
}

}  // namespace hj_bf
