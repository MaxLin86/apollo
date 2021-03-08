#include <thread>
#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/remoteManage/proto/remoteManage.pb.h"
#include "modules/canbus/proto/chassis.pb.h"

using apollo::cyber::Component;
using apollo::cyber::TimerComponent;

namespace apollo{
namespace remoteManage{

#define		TMC_MAX_COORD_NUM		(8096)

enum
{
	ES_NORMAL = 0,		     // 正常接收一个报文
	ES_ERROR, 			  // 报文格式错误
	ES_INCOMPLETE,	          // 报文接收不完整
};

  class remoteManage : public apollo::cyber::TimerComponent {
 public:
  ~remoteManage() {}
  bool Init() override;
  bool Proc() override;

private:
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
  uint8_t extractPacketTask(uint8_t * pData, uint32_t nDataLen);
  uint8_t extractPacketControl(uint8_t * pData, uint32_t nDataLen);

  template <typename T>
  bool extractValue(uint8_t * pData, uint32_t nDataLen, T * pnValue);
  template <typename T>
  bool extractArrayValue(uint8_t * pData, uint32_t nDataLen, T * pnValue, uint32_t nArrayNum);
  
 private:
	bool			bFlagQuitModule;
	bool			bFlagConnect;
	std::mutex	 mutex_;

  int32_t act_fdConnectSocket;            // tcp client connect socket to TMC
  uint8_t act_nPacketPulse[106];
  uint8_t act_nPacketAbnormal[79];
  uint8_t act_nPacketStatus[72];
  
  uint32_t act_nRecvDataLen;
  uint8_t act_nRecvDataBuf[655360];
  uint8_t act_nID[64];
  uint8_t act_nNumCoord;
  double act_dbCoord[TMC_MAX_COORD_NUM * 2];
  uint8_t act_nControlType;
  uint16_t act_nDistance;

  actNet_conf  actNetConfig;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  canbus::Chassis  chassis_info_;

  std::unique_ptr<std::thread>  thread_socket_connect_;
  std::unique_ptr<std::thread>  thread_socket_write_;
  std::unique_ptr<std::thread>  thread_socket_read_;

};

CYBER_REGISTER_COMPONENT(remoteManage)

}  // namespace remoteManage
}  // namespace apollo
