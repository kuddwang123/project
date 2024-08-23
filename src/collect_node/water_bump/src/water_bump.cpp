// @file demo.cpp
// @brief
//
// Copyright 2023 HJ Technology Co.Ltd. All rights reserved.
// Authors: 609384347@qq.com (wangqing, 2023-12-22)
#include "water_bump.h"

HJ_REGISTER_FUNCTION(factory) {
  std::cerr << "minos register factory" << FUNCTION_NAME << std::endl;
  factory.registerCreater<collect_node_ns::WaterBump>(FUNCTION_NAME);
}
namespace collect_node_ns {
/* 创建队列 */
Queue *createQueue(int size) {
  Queue *queue = (Queue *)malloc(sizeof(Queue));
  queue->buffer = (unsigned char *)malloc(size * sizeof(unsigned char));
  queue->head = 0;
  queue->tail = 0;
  queue->size = size;
  return queue;
}
void deleteQueue(Queue *queue){
  if (nullptr != queue) {
    if (nullptr != queue->buffer) {
      free(queue->buffer);
    }
    free(queue);
  }
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

int uart_open(char *dev, int *fd) {
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

void WaterBump::WritePackage(uint8_t *buf, uint8_t len) {
  if (false == driver_ok_) {
    return;
  }
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

  uart_send(fd_, (char *)SendBuf, sendlen);
}

void WaterBump::kbdCtrlChatterCallback(const hj_interface::Kbd::ConstPtr &msg) {
  HJ_INFO("kbd_ctrl receive ");

  u_int8_t send_speed[] = {0x10, 0x0,  0x0,  0x0,  0xff, 0xff, 0xff, 0xff, 0xff,
                           0xff, 0xff, 0xff, 0x08, 0x0,  0xff, 0xff, 0xff};
  u_int8_t pump[] = {0x32, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x06, 0x0, 0xff, 0xff, 0xff};
  u_int8_t turn_motor[] = {0x30, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x05, 0x0, 0xff, 0xff, 0xff};
  u_int8_t airbag[] = {0x34, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0x03, 0x0, 0xff, 0xff, 0xff};
  u_int8_t fan[] = {0x36, 0x0, 0x0, 0x0, 0xff, 0x01, 0x0, 0xff, 0xff, 0xff};
  u_int8_t flip_cover[] = {0x38, 0x0, 0x0, 0x0, 0xff, 0xff, 0x02, 0x0, 0xff, 0xff, 0xff};

  u_int8_t type = (u_int8_t)msg->type;

  static u_int8_t pump_status;

  u_int8_t turn_motor_status = 1;  //(u_int8_t)msg->turn_motor_status;
  static u_int8_t airbag_status;

  if (type == WHEEL) {
    int32_t lw_speed = (int32_t)msg->left_msg;
    int32_t rw_speed = (int32_t)msg->right_msg;
    HJ_INFO("lw_speed = %d, rw_speed = %d", lw_speed, rw_speed);
    memcpy(send_speed + 4, &lw_speed, sizeof(int32_t));
    memcpy(send_speed + 8, &rw_speed, sizeof(int32_t));
    WritePackage(send_speed, sizeof(send_speed) / sizeof(u_int8_t));
  }

  if (type == PUMP_SWITCH) {
    pump_status = (u_int8_t)msg->pump_status;
    HJ_INFO("pump_status = %d", pump_status);
    if (0 == pump_status) {
      memcpy(pump + 4, &pump_status, sizeof(u_int8_t));
      WritePackage(pump, sizeof(pump) / sizeof(u_int8_t));
    }
  }

  if ((type == PUMP_FLOWRATE) || (type == PUMP_DUTY_CYCLE)) {
    u_int8_t pump_speed = (u_int8_t)msg->pump_speed;
    HJ_INFO("pump_status = %d, pump_speed = %d , send to mcu", pump_status, pump_speed);
    memcpy(pump + 4, &pump_status, sizeof(u_int8_t));
    memcpy(pump + 5, &pump_speed, sizeof(u_int8_t));
    WritePackage(pump, sizeof(pump) / sizeof(u_int8_t));
  }

  if (type == LEFT_SPOUT_VEER) {
    int16_t turn_motor_ll = (int16_t)msg->turn_motor_l;
    memcpy(turn_motor + 4, &turn_motor_status, sizeof(u_int8_t));
    memcpy(turn_motor + 5, &turn_motor_ll, sizeof(int16_t));
    memset(turn_motor + 7, 0x00, 2);
    HJ_INFO("turn_motor_status = %d, turn_motor_l = %d , send to mcu", turn_motor_status, turn_motor_ll);
    WritePackage(turn_motor, sizeof(turn_motor) / sizeof(u_int8_t));
  }

  if (type == RIGHT_SPOUT_VEER) {
    int16_t turn_motor_rr = (int16_t)msg->turn_motor_r;
    memcpy(turn_motor + 4, &turn_motor_status, sizeof(u_int8_t));
    memcpy(turn_motor + 7, &turn_motor_rr, sizeof(int16_t));
    memset(turn_motor + 5, 0x00, 2);
    HJ_INFO("turn_motor_status = %d, turn_motor_r = %d , send to mcu", turn_motor_status, turn_motor_rr);
    WritePackage(turn_motor, sizeof(turn_motor) / sizeof(u_int8_t));
  }

  if (type == DIVE_MOTOR) {
    airbag_status = (u_int8_t)msg->airbag_status;
    HJ_INFO("airbag_status = %d", airbag_status);
  }

  if (type == INFLACTION_TIME) {
    u_int16_t airbag_time = (u_int16_t)msg->airbag_time;
    HJ_INFO("airbag_status = %d, airbag_time = %d , send to mcu", airbag_status, airbag_time);
    memcpy(airbag + 4, &airbag_status, sizeof(u_int8_t));
    memcpy(airbag + 5, &airbag_time, sizeof(u_int16_t));
    WritePackage(airbag, sizeof(airbag) / sizeof(u_int8_t));
  }

  if (type == SLEF_CLEAN_SWITCH) {
    u_int8_t fan_status = (u_int8_t)msg->fan_status;
    HJ_INFO("fan_status = %d , send to mcu", fan_status);
    memcpy(fan + 4, &fan_status, sizeof(u_int8_t));
    WritePackage(fan, sizeof(fan) / sizeof(u_int8_t));
    usleep(10 * 1000);
  }

  if (type == UPPER_SUCTION_SWITCH) {
    int16_t flip_cover_angle = (int16_t)msg->flip_cover_angle;
    HJ_INFO("flip_cover_angle = %d , send to mcu", flip_cover_angle);
    memcpy(flip_cover + 4, &flip_cover_angle, sizeof(int16_t));
    WritePackage(flip_cover, sizeof(flip_cover) / sizeof(u_int8_t));
  }
}

WaterBump::~WaterBump() {
  close(fd_);
  deleteQueue(queue_);
}
WaterBump::WaterBump(const rapidjson::Value &json_conf) : hj_bf::Function(json_conf) {
  // read your config
  int ret;
  queue_ = createQueue(LEN_QUE_RECV);

  ret = uart_open("/dev/ttyS4", &fd_);  //打开串口，返回文件描述符
  if (RET_OK != ret) {
    HJ_ERROR("minos can't open /dev/ttyS4 ");
  }
  ret = uart_init(fd_, 460800, 0, 8, 1, 'N');
  if (RET_ERR == ret) {
    HJ_ERROR("ret init = %d", ret);
  } else {
    driver_ok_ = true;
  }
  sub_kbd_ctrl_ = hj_bf::HJSubscribe("kbd_ctrl", 1, &WaterBump::kbdCtrlChatterCallback, this);
  // your code
  std::cerr << "minos just a WaterBump" << std::endl;
}
}  // namespace collect_node_ns
