/*
 * @Author: your name
 * @Date: 2020-05-07 17:31:54
 * @LastEditTime: 2020-05-14 09:24:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/hardware/socket_monitor.h
 */
#pragma once

#include "modules/monitor/common/recurrent_runner.h"
#include "cyber/common/macros.h"
#include <future>
#include <string>
#include <mutex>

namespace apollo{
namespace monitor{
 

class SocketMonitor : public RecurrentRunner { 
public:
    SocketMonitor(); 
    int Init();
    bool isOK();
    void RunOnce(const double current_time) override;
    const std::string &GetErr() { return err_msg_; }
   private:
    int ReceiveData();
    int doAccept();
    void StartRecevie();
    std::future<int> future_;

    int conn_fd_ = -1;
    int listen_fd_ = -1;
    bool Heartbeat_ = false;
    bool Accept_ = false;
    bool start_ = false;
    bool fatal_ = false;
    std::string err_msg_;
    std::mutex mutex_;
    bool IsManuCtrl_ = false;

    //manu ctrl
    // std::shared_ptr<cyber::Reader<apollo::manuctrl::manuctrlMsg>> manuctrl_msg_reader_;
    // apollo::manuctrl::manuctrlMsg manuctrl_msg_;
};
}  // namespace monitor
}