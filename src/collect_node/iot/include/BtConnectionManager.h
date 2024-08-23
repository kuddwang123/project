/**
 * @ Author: lou.pan
 * @ Create Time: 2024-04-25 16:58:30
 * @ Modified by: lou.pan
 * @ Modified time: 2024-06-21 18:27:07
 * @ Description: Manage BlueTooth connection
 */

#pragma once

#include <string>
#include <memory>
#include <thread>
#include <boost/function.hpp>
#include <condition_variable>
#include <atomic>
#include <mutex>
#include <boost/signals2.hpp>
#include "rapidjson/document.h"
#include "node_factory.h"
#include "std_msgs/String.h"
#include "Defines.h"

namespace collect_node_iot {

class BtConnectionManager;

typedef boost::function<void(const std::string&, const std::string&)> btDataJsonCb;
#if 0
typedef boost::function<void(bool)> btConnCb;
#endif

typedef std::shared_ptr<BtConnectionManager> BtConnectionManagerPtr;

enum bt_conn_type {
  BT_IDLE = 0,
  BT_CONN = 1,
  BT_DISCONN = 2
};

class BtConnectionManager
{
public:
    static const BtConnectionManagerPtr& instance();
    BtConnectionManager();
    ~BtConnectionManager();

  //连接初始化，设置参数和连接回调
    bool initialize();
  
  //获取与App连接状态
    bool isConnected() const;
  
  //上报蓝牙数据
    void btRpt(const std::string& key, const std::string& payload);
  
  //发送蓝牙数据
    void btSend(const std::string& key, const std::string& payload, int res);

  //注册蓝牙数据业务层处理回调
    void setBtDataHandler(const btDataJsonCb& cb) { btDataJsonCb_ = cb; }

  //注册蓝牙连接状态回调
    boost::signals2::connection addConStatListener(const ConStaChangeCb& slot);
    
private:
    bool isRun_;
    hj_bf::HJSubscriber bt_data_sub_;
    hj_bf::HJSubscriber bt_state_sub_;
    hj_bf::HJPublisher bt_data_pub_;    

    bt_conn_type connState_;
    std::string dataBuf_;
    std::condition_variable cond_;
    std::mutex mtx_;
    std::thread btDataParseThread_;
    btDataJsonCb btDataJsonCb_;
    boost::signals2::signal<void(bool)> conchange_signal_;

private:
  // Invoked when a bt connection with app has changed
    void btConnStateCb(const std_msgs::String::ConstPtr&);
  
  // Invoked when bt data received
    void btDataRecvCb(const std_msgs::String::ConstPtr&);
  
  // Bluetooth data parse thread
    void btDataParseThread();

  // Bluetooth data valid parse
    bool btDataParser(const std::string in, std::string& rnout, std::string& plout);

  // Checksum calculate
    uint16_t checksum_calculate(const std::string&);
    
  // Send bt data seperately
    void btMsgPub(const std::string&);
    
private:
    static const int MAX_BT_CACHE_LENGTH = 1024;
    static const int BLUETOOTH_MTU = 400;
};

}