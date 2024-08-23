// @file heartbeat.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "heartbeat.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<collect_node_ns::HeartBeat>(FUNCTION_NAME);
}
namespace collect_node_ns {
static int g_fd;
static unsigned char m_pRxBuf[MAX_PACKAGE_SIZE];

Queue *queue;
/* 创建队列 */
Queue *createQueue(int size) {
  Queue *queue = (Queue *)malloc(sizeof(Queue));
  queue->buffer = (unsigned char *)malloc(size * sizeof(unsigned char));
  queue->head = 0;
  queue->tail = 0;
  queue->size = size;
  return queue;
}

/* 队列满 */
bool isFull(Queue *queue) { return (queue->tail + 1) % queue->size == queue->head; }

/* 队列空 */
bool isEmpty(Queue *queue) { return queue->head == queue->tail; }

/* 入队列 */
void enqueue(Queue *queue, unsigned char data) {
  if (isFull(queue)) {
    printf("Queue is full!\n");
    return;
  }
  queue->buffer[queue->tail] = data;
  queue->tail = (queue->tail + 1) % queue->size;
}

/* 出队列 */
unsigned char dequeue(Queue *queue) {
  if (isEmpty(queue)) {
    printf("Queue is empty!\n");
    return -1;
  }
  unsigned char data = queue->buffer[queue->head];
  queue->head = (queue->head + 1) % queue->size;
  return data;
}

int uart_open(const char *dev, int *fd) {
  int ret;

  *fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  printf("fd open %s\n", dev);
  if (0 > *fd) {
    printf("Can't Open Serial Port");
    return RET_ERR;
  }
  //恢复串口为阻塞状态
  ret = fcntl(*fd, F_SETFL, 0);
  printf("fcntl=%d\n", ret);
  if (0 > ret) {
    printf("fcntl failed!\n");
    return RET_ERR;
  }
  //测试是否为终端设备
  if (0 == isatty(STDIN_FILENO)) {
    printf("standard input is not a terminal device\n");
    return RET_ERR;
  }
  printf("isatty success!\n");

  return RET_OK;
}

void uart_close(int fd) { close(fd); }

int uart_set(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity) {
  int i, status;
  int speed_arr[] = {B460800, B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {460800, 115200, 19200, 9600, 4800, 2400, 1200, 300};

  struct termios options;

  /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
  该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
  if (0 != tcgetattr(fd, &options)) {
    printf("SetupSerial 1\n");
    return RET_ERR;
  }

  //设置串口输入波特率和输出波特率
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (speed == name_arr[i]) {
      cfsetispeed(&options, speed_arr[i]);
      cfsetospeed(&options, speed_arr[i]);
    }
  }

  //修改控制模式，保证程序不会占用串口
  options.c_cflag |= CLOCAL;
  //修改控制模式，使得能够从串口中读取输入数据
  options.c_cflag |= CREAD;

  //设置数据流控制
  switch (flow_ctrl) {
    case 0:  //不使用流控制
      options.c_cflag &= ~CRTSCTS;
      break;
    case 1:  //使用硬件流控制
      options.c_cflag |= CRTSCTS;
      break;
    case 2:  //使用软件流控制
      options.c_cflag |= IXON | IXOFF | IXANY;
      break;
  }
  //设置数据位
  //屏蔽其他标志位
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
      printf("Unsupported data size\n");
      return RET_ERR;
  }
  //设置校验位
  switch (parity) {
    case 'n':
    case 'N':  //无奇偶校验位。
      options.c_cflag &= ~PARENB;
      options.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':  //设置为奇校验
      options.c_cflag |= (PARODD | PARENB);
      options.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':  //设置为偶校验
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':  //设置为空格
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      printf("Unsupported parity\n");
      return RET_ERR;
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
      printf("Unsupported stop bits\n");
      return RET_ERR;
  }

  //修改输出模式，原始数据输出
  options.c_oflag &= ~OPOST;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  // options.c_lflag &= ~(ISIG | ICANON);
  options.c_iflag &= ~ICRNL;   //禁止将输入的CR转换为NL
  options.c_iflag &= ~(IXON);  //清bit位 关闭流控字符
  //设置等待时间和最小接收字符
  options.c_cc[VTIME] = 0; /* 读取一个字符等待1*(1/10)s */
  options.c_cc[VMIN] = 0;  /* 读取字符的最少个数为1 */

  //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
  tcflush(fd, TCIFLUSH);

  //激活配置 (将修改后的termios数据设置到串口中）
  if (0 != tcsetattr(fd, TCSANOW, &options)) {
    printf("com set error!\n");
    return RET_ERR;
  }

  return RET_OK;
}

int uart_init(int fd, int speed, int flow_ctrl, int databits, int stopbits, int parity) {
  int ret;
  //设置串口数据帧格式
  ret = uart_set(fd, speed, flow_ctrl, databits, stopbits, parity);
  return ret;
}

int uart_send(int fd, char *buf, int data_len) {
  int len = 0;
  len = write(fd, buf, data_len);
  if (len != data_len) {
    tcflush(fd, TCOFLUSH);
    return RET_ERR;
  }
  return RET_OK;
}

static void *uart_recv_thread(void *arg) {
  int ret, i, n;
  fd_set readfds;
  unsigned char buf[MAX_PACKAGE_SIZE];
  while (1) {
    FD_ZERO(&readfds);
    FD_SET(g_fd, &readfds);

    ret = select(g_fd + 1, &readfds, NULL, NULL, NULL);
    if (0 > ret) {
      printf("select failed\n");  //失败
      continue;
    } else if (0 == ret) {
      printf("select time out!\n");
    } else if (FD_ISSET(g_fd, &readfds)) {
      n = read(g_fd, (void *)buf, MAX_PACKAGE_SIZE);  //配置为非阻塞
      if (0 >= n) {
        printf("read failed %d\n", n);
        continue;
      }
      for (i = 0; i < n; i++) {
        enqueue(queue, buf[i]);
      }
    }
  }
  return NULL;
}

void WritePackage(uint8_t *buf, uint8_t len) {
  uint8_t i;
  uint8_t *pBuf;
  uint8_t SendBuf[64];
  uint8_t sendlen;
  uint8_t CheckSum = 0;

  pBuf = SendBuf;
  *pBuf++ = FRAMEHEAD;
  *pBuf++ = FRAMEHEAD;

  for (i = 0; i < len; i++) {
    if ((buf[i] == (uint8_t)FRAMECTRL) || (buf[i] == (uint8_t)FRAMEHEAD) || (buf[i] == (uint8_t)FRAMETAIL)) {
      *pBuf++ = FRAMECTRL;
    }
    *pBuf++ = buf[i];
    CheckSum += buf[i];
  }

  // checksum
  if ((CheckSum == (uint8_t)FRAMECTRL) || (CheckSum == (uint8_t)FRAMEHEAD) || (CheckSum == (uint8_t)FRAMETAIL)) {
    *pBuf++ = FRAMECTRL;
  }
  *pBuf++ = CheckSum;

  // Send Tail USART_FRAMETAIL USART_FRAMETAIL
  *pBuf++ = FRAMETAIL;
  *pBuf++ = FRAMETAIL;

  sendlen = pBuf - SendBuf;

  uart_send(g_fd, (char *)SendBuf, sendlen);
}
void sendHeartbeat(const hj_bf::HJTimerEvent &) {
  static u_int8_t heart_buf[] = {0x01, 0x0, 0x0, 0x0, 0xff, 0xe8, 0x03, 0x00, 0x00, 0x05, 0xff, 0xff, 0xff};
  WritePackage(heart_buf, sizeof(heart_buf));
}
HeartBeat::~HeartBeat() { uart_close(g_fd); }
HeartBeat::HeartBeat(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read config
  int ret;
  queue = createQueue(LEN_QUE_RECV);
  std::string dev_name = "/dev/ttyS9";
  ret = uart_open(dev_name.c_str(), &g_fd);  //打开串口，返回文件描述符
  if (ret == RET_OK) {
    ret = uart_init(g_fd, 460800, 0, 8, 1, 'N');
    if (ret == RET_OK) {
      loop_timer_ = hj_bf::HJCreateTimer("heartbeat_timer", 900 * 1000, sendHeartbeat);
    } else {
      std::cerr << "uart_init fail!" << std::endl;
    }
  } else {
    std::cerr << "uart_open fail!" << std::endl;
  }
}
}  // namespace collect_node_ns
