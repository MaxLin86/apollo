 
#include "modules/monitor/hardware/socket_monitor.h"

#include "cyber/common/log.h"
#include "modules/monitor/common/monitor_manager.h"
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include "cyber/task/task.h"
#include "modules/monitor/software/summary_monitor.h"
#include "modules/common/util/map_util.h"
#include "absl/strings/str_cat.h"
#define MAX_READ_LINE 1024
DEFINE_string(socket_monitor_name, "SocketMonitor",
              "Name of the CAN monitor.");
 
DEFINE_double(socket_monitor_interval, 1,
              "Socket status checking interval seconds.");
DEFINE_string(socket_component_name, "Socket",
              "Name of the Socket CAN component in SystemStatus.");
// DEFINE_string(socket_component_name, "SocketCAN",
//               "Name of the Socket CAN component in SystemStatus.");
namespace apollo {
namespace monitor {
     
int SocketMonitor::Init(){
//   manuctrl_msg_reader_= node_->CreateReader<manuctrlMsg>(
//       FLAGS_manuctrl_topic,
//       [this](const std::shared_ptr<manuctrlMsg>& cmd) {
//         ADEBUG << "Received manuctrl data: run serial callback.";
//         std::lock_guard<std::mutex> lock(mutex_);
//         manuctrl_msg_.CopyFrom(*cmd);
//       }); 
 
  int ret = -1;
  int server_ip_port = 9876;

  struct sockaddr_in t_sockaddr;
  memset(&t_sockaddr, 0, sizeof(t_sockaddr));
  t_sockaddr.sin_family = AF_INET;
  t_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  t_sockaddr.sin_port = htons(server_ip_port);

  listen_fd_ = socket(AF_INET, SOCK_DGRAM, 0); 
  if (listen_fd_ < 0) {
    err_msg_ = absl::StrCat("socket error ", strerror(errno), " errno=", errno);
    return -1; 
    }

    ret = bind(listen_fd_,(struct sockaddr *) &t_sockaddr,sizeof(t_sockaddr));
    if (ret < 0) {
      err_msg_ =
          absl::StrCat("bind socket error ", strerror(errno), " errno=", errno);
      return -1; 
    } 
    // struct timeval timeout = {1,0}; 
    struct timeval timeout = {0,500000}; 
    ret =setsockopt(listen_fd_,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(struct timeval));
    if (ret < 0) {
      err_msg_ =
          absl::StrCat("setsockopt error ", strerror(errno), " errno=", errno);
      return -1; 
    }
    // ret = listen(listen_fd_, 1024);
    // if (ret < 0) {
    //   err_msg_ = absl::StrCat("listen error ", strerror(errno), " errno=", errno);
    //   return -1; 
    // } 
    return 0;
}


int SocketMonitor::doAccept(){ 
  conn_fd_ = accept(listen_fd_, (struct sockaddr *)NULL, NULL); 
  if (conn_fd_ < 0) {
    err_msg_ = absl::StrCat("accpet error ", strerror(errno), " errno=", errno);  
    Accept_ = false;
    return -1;
    }  
    future_ = cyber::Async(&SocketMonitor::ReceiveData, this); 
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    Accept_ = true;
    return 0;
}

bool SocketMonitor::isOK(){   
  { 
    std::lock_guard<std::mutex> lg(mutex_); 
    if (Heartbeat_) return true;
  }
  if (!start_ && future_.valid()) {
    future_.get();
  }
  // if (!Accept_ && future_.valid()) {
  //   future_.get();
  // }
    return false;
}

void SocketMonitor::StartRecevie(){ 
    future_ = cyber::Async(&SocketMonitor::ReceiveData, this); 
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    start_ = true;
}

int SocketMonitor::ReceiveData()
{ 
  char buf[MAX_READ_LINE]; 
  socklen_t len;
  int count;
  struct sockaddr_in clent_addr; 
  for (;;) { 
    memset(buf, 0, MAX_READ_LINE);
    len = sizeof(clent_addr);
    count = recvfrom(listen_fd_, buf, MAX_READ_LINE, 0,
                     (struct sockaddr *)&clent_addr,
                     &len);  
    if (count == -1) {
      err_msg_ = "lost heart beat recieve data fail!";
      Heartbeat_ = false;
      start_ = false;
      return -1;
    }

    buf[count] = '\0';
    std::string recv_data(buf);
    if (recv_data.compare("FATAL") == 0) {
      err_msg_ = "heart beat get FATAL command";
      Heartbeat_ = false; 
      continue;
    } else if (recv_data.compare("hello") == 0) {
      err_msg_ = "success heart beat "; 
      Heartbeat_ = true;
      continue;
    }  
    }
}
// int SocketMonitor::ReceiveData(){ 
//   int recv_len = -1;
//   char buff[MAX_READ_LINE]; 
//   for (;;) {
//     recv_len = recv(conn_fd_, buff, MAX_READ_LINE, 0);
//     if (recv_len <=0) { 
//       err_msg_.append("recv error:"); 
//       { 
//         std::lock_guard<std::mutex> lg(mutex_);
//         Heartbeat_ = false;
//       }
//       Accept_ = false;
//       close(conn_fd_);
//       conn_fd_ = -1;
//       return -1;
//     } 
//     buff[recv_len] = '\0';
//     std::string recv_data(buff); 
//     if (recv_data.compare("FATAL") == 0){ 
//       err_msg_ = "get FATAL command"; 
//         Heartbeat_ = false; 
//       continue;
//     } 
//     else if(recv_data.compare("hello") == 0){ 
//       err_msg_ = "socket ok"; 
//         Heartbeat_ = true; 
//       continue;
//     } 
//     }

//     return 0;
// }

SocketMonitor::SocketMonitor()
    : RecurrentRunner(FLAGS_socket_monitor_name,
                      FLAGS_socket_monitor_interval) {
//   SocketCommunication::Instance()->Init();

  Init();
} 

void SocketMonitor::RunOnce(const double current_time) {
  // if (!Accept_) doAccept();
  if (!start_) StartRecevie(); 
  auto manager =  MonitorManager::Instance();
  Component* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(),
      FLAGS_socket_component_name);
  if (component == nullptr) { 
    return;
  }
  auto* status = component->mutable_other_status();
  status->clear_status();
  if (!isOK()) { 
    SummaryMonitor::EscalateStatus(ComponentStatus::FATAL,
                                   ErrorCode::MONITOR_OTHER_ERROR,
                                  GetErr() , status);
  }else{
    SummaryMonitor::EscalateStatus(ComponentStatus::OK, ErrorCode::OK, GetErr(),
                                   status);
  }
}  
}
}