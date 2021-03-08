#include <thread>
#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/remoteManage/proto/remoteManage.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

using apollo::cyber::Component;
using apollo::common::Point2D;
using apollo::cyber::TimerComponent;


#define     TMC_MAX_TASK_COORD	(8096)

namespace apollo{
namespace remoteManage{

  class remoteManage : public apollo::cyber::TimerComponent {
 public:
  ~remoteManage() {}
  bool Init() override;
  bool Proc() override;

private:
  bool InitSocket();
  void StopSocket();
  void socketConnect();
  void socketWrite();
  void socketRead();

  void InitActData();
  void InitPacketPulse();
  void InitPacketStatus();
  void packPulse();
  void packStatus();
  bool analysisPacket(uint8_t * pData, uint32_t * pnDataLen);
  
 private:
	bool			bFlagQuitModule;
	bool			bFlagConnect;
	std::mutex	 mutex_;

  int32_t act_fdConnectSocket;            // tcp client connect socket to TMC
  uint8_t act_nPacketPulse[106];
  uint8_t act_nPacketAbnormal[79];
  uint8_t act_nPacketStatus[72];
  
  uint32_t act_nRecvDataLen;
  uint8_t act_nRecvDataBuf[65536];
  uint8_t act_nID[64];
  uint8_t act_nNumCoord;
  double act_dbCoord[4096 * 2];
  actTask act_task;

  actNet_conf  actNetConfig;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<apollo::cyber::Writer<actTask>> task_writer_;
  canbus::Chassis  chassis_info_;

  std::unique_ptr<std::thread>  thread_socket_connect_;
  std::unique_ptr<std::thread>  thread_socket_write_;
  std::unique_ptr<std::thread>  thread_socket_read_;

};

CYBER_REGISTER_COMPONENT(remoteManage)

}  // namespace remoteManage
}  // namespace apollo
