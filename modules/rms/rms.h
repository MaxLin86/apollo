
#include <thread>
#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/rms/proto/rms.pb.h"
#include "modules/rms/proto/rms_conf.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/manuctrl/proto/manuctrl.pb.h"
#include "modules/tmc/proto/tmc.pb.h"
#include "modules/urgencystop/proto/aeb_cmd.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"

using apollo::cyber::Component;

namespace apollo{
namespace rms{

#define                 RMS_VERSION_PROTO                                          (23)
#define                 RMS_PACKET_LEN_MAX                                       (655360)
#define                 RMS_PACKET_LEN_CONTROL                            (31)
#define                 RMS_MAX_SEND_BUF_SIZE					(1024)
#define                 RMS_CAR_ID_LEN                                                    (64)
#define                 RMS_MAX_TASK_COORD                                      (80960)
#define                 SENSOR_UT_RADAR_OBJ_NUM                                 (4)
#define			SENSOR_WM_RADAR_OBJ_NUM					(100)

enum
{
        LS_UNUNITED = 0,                  /* 未连接状态 */
        LS_CONNECTING  = 1,           /* 已连接状态 */
        LS_CONNECT_REQ = 2,        /* 请求连接状态 */
        LS_HEARTBEAT_REQ = 3,    /* 心跳包请求a状态 */
        LS_CLOSE_REQ = 4,               /* 请求关闭状态 */
};

enum
{
        AS_MISS_HEAD = 1,                         /* 缺少包头错误 */
        AS_DATA_INCOMPLETE = 2,         /* 包长不完整 */
        AS_FORMAL_ERROR = 3,              /* 包格式错误 */
        AS_DATA_SUCCESS = 4,                /* 包数据解析完整 */
};

enum 
{
        TS_NONE = 0,                                     /* 无任务 */
        TS_RUNNING = 1,                             /* 正在进行 */
        TS_ABNORMAL = 2,                         /* 任务异常 */
        TS_DONE = 3,                                    /* 任务完成 */
	TS_PLACE = 4,
	TS_ALIGNING = 5,
};

enum
{
        DM_IDLE = 0,                                      /* 空闲模式 */
        DM_AUTO = 1,                                    /* 自驾模式 */
        DM_REMOTE = 2,                              /* 远控模式 */
        DM_MANUAL = 3,                              /* 遥控模式 */
        DM_AEBS = 4,                                     /* 紧急制动 */
        DM_MISS = 5,                                      /* 控制异常 */
};

enum
{
	DT_MANUAL = 1,
	DT_AUTODRIVE = 2,
};

enum
{
	TF_OLD = 0,
	TF_NEW = 1,
};

enum
{
	TT_NONE = 0,
	TT_RECV_BOX = 1,
	TT_SEND_BOX = 2,
};

enum
{
	BT_20 = 1,
	BT_20_20 = 2,
	BT_40 = 3,
	BT_45 = 4,
};

enum
{
	BP_FRONT = 1,
	BP_CENTER = 2,
	BP_REAR = 3,
};

struct packetControl
{
        uint8_t         nVersion;
        uint8_t         nCmd;
        uint16_t       nServerRandData;
        uint16_t       nClientRandData;
        uint32_t       nChassisIP;
        uint16_t       nChassisPort;
        uint32_t       nServerIP;
        uint16_t       nServerPort;
        uint8_t          nChassisProto;
        uint32_t       nActIP;
        uint16_t       nActPort;
        uint8_t          nActProto;
        uint16_t       nServerPeriod;
        uint16_t       nClientPeriod;
        uint8_t          nErrorCode;

        packetControl()
        {
                nVersion = RMS_VERSION_PROTO;
                nCmd = 0;
                nServerRandData = 0;
                nClientRandData = 0;
                nChassisIP = ntohl(inet_addr("10.28.63.36"));
                nChassisPort = 10002;
                nServerIP = 0;
                nServerPort = 0;
                nChassisProto = 17;
                nActIP = ntohl(inet_addr("10.28.63.36"));                /* 后续修改为从TMC获取 */
                nActPort = 10000;
                nActProto = 17;
                nServerPeriod = 20;
                nClientPeriod = 50;
                nErrorCode = 0;
        }
};

struct messageIPCStatus
{
        uint8_t         nCode;
        uint16_t      nLen;
        uint16_t      nDelay;
        uint8_t         nCpuUsed;                 /* cpu利用率 */
        uint8_t         nMemoryUsed;        /* 内存利用率 */
        uint8_t         nCpuTemp;               /* cpu 温度 */

        messageIPCStatus()
        {
                nCode = 1;
                nLen = 5;
                nDelay = 0;
                nCpuUsed = 0;
                nMemoryUsed = 0;
                nCpuTemp = 0;
        }
};

struct messageErrorCode
{
        uint8_t         nCode;
        uint16_t      nLen;
        uint8_t         nDTC;
        uint8_t         nErrorType;

        messageErrorCode()
        {
                nCode = 2;
                nLen = 2;
                nDTC = 0;
                nErrorType = 0;
        }
};

struct messageSensorStatus
{
        uint8_t         nCode;
        uint16_t      nLen;
        uint8_t         nWmRadar;
        uint8_t         nUtRadar;
        uint8_t         nIMU;
        uint8_t         nCamera;

        messageSensorStatus()
        {
                nCode = 3;
                nLen = 4;
                nWmRadar = 0;
                nUtRadar = 0;
                nIMU = 0;
                nCamera = 0;
        }
};

struct messageModuleStatus
{
        uint8_t         nCode;
        uint16_t      nLen;
        uint8_t         nRadar;
        uint8_t         nTmc;
        uint8_t         nGuardian;
        uint8_t         nCanbus;
        uint8_t         nPlanning;
        uint8_t         nPerception;

        messageModuleStatus()
        {
                nCode = 4;
                nLen = 6;
                nRadar = 0;
                nTmc = 0;
                nGuardian = 0;
                nCanbus = 0;
                nPlanning = 0;
                nPerception = 0;
        }
};

struct messageUtRadar
{
        uint8_t         nCode;
        uint16_t      nLen;
        uint8_t         nRadarID[ SENSOR_UT_RADAR_OBJ_NUM ];
        uint8_t         nDistance[ SENSOR_UT_RADAR_OBJ_NUM ];

        messageUtRadar()
        {
                nCode = 10;
                nLen = 0;
                memset( nRadarID, 0, sizeof(uint8_t) * SENSOR_UT_RADAR_OBJ_NUM );
                memset( nDistance, 0, sizeof(uint8_t) * SENSOR_UT_RADAR_OBJ_NUM );
        }
};

struct messageWmRadar
{
        uint8_t         nCode;
        uint16_t      nLen;
        uint8_t         nCoordX[ SENSOR_WM_RADAR_OBJ_NUM ];
        uint8_t         nCoordY[ SENSOR_WM_RADAR_OBJ_NUM ];
        uint8_t         nCoordZ[ SENSOR_WM_RADAR_OBJ_NUM ];
        uint8_t         nRateX[ SENSOR_WM_RADAR_OBJ_NUM ];
        uint8_t         nRateY[ SENSOR_WM_RADAR_OBJ_NUM ];

        messageWmRadar()
        {
                nCode = 11;
                nLen = 0;
                memset( nCoordX, 0, sizeof(uint8_t) * SENSOR_WM_RADAR_OBJ_NUM );
                memset( nCoordY, 0, sizeof(uint8_t) * SENSOR_WM_RADAR_OBJ_NUM );
                memset( nCoordZ, 0, sizeof(uint8_t) * SENSOR_WM_RADAR_OBJ_NUM );
                memset( nRateX, 0, sizeof(uint8_t) * SENSOR_WM_RADAR_OBJ_NUM );
                memset( nRateY, 0, sizeof(uint8_t) * SENSOR_WM_RADAR_OBJ_NUM );
        }
};

struct packetActInfo
{
        uint8_t         nVersion;
        uint8_t         nRcStatus;
        uint8_t         nControlHost;
        uint8_t         nAebsEnable;
        uint16_t      nPacketSequence;
        uint16_t      nRandom;
        messageIPCStatus             stIpcStatus;
        messageErrorCode            stErrorCode;
        messageSensorStatus      stSensorStatus;
        messageModuleStatus     stModuleStatus;
        messageUtRadar                stRadarUT;
        messageWmRadar             stRadarWm;
        uint8_t        nCheckSum[16];

        packetActInfo()
        {
                nVersion = RMS_VERSION_PROTO;
                nRcStatus = 0;
                nControlHost = 0;
                nAebsEnable = 1;
                nPacketSequence = 0;
                nRandom = 0;
                memset( nCheckSum, 0, sizeof(uint8_t) * 16);
        }
};

struct packetData
{
        uint8_t         nVersion;
        uint32_t       nServerRandData;
};


class netRms : public apollo::cyber::TimerComponent {
public:
        ~netRms() {}
        bool Init() override;
        bool Proc() override;

private:
        void InitData();
        void InitChannel();
	void resetTmcStatus();
        void netTmcConnect(void);
        void netTmcDataSend(void);
        void netTmcDataRecv(void);
        void netRemoteConnect(void);
        void netRemoteDataSend(void);
        void netRemoteDataRecv(void);

        bool sendTmcHeartbeatPacket();
        bool sendTmcCarStatusPacket();
        bool sendRcControlPacket( uint32_t nCmd );
        bool recvRcControlPacket(uint8_t * pnData, uint32_t nDataLen);
	bool sendActInfoPacket();

        bool analysisPacketRaw(uint8_t * pData, uint32_t * pnDataLen);
        bool analysisPacketTask( uint8_t * pData );
        bool analysisPacketControl( uint8_t * pData );
	bool analysisPacketResponse( uint8_t * pData );

	void GetCarChassisInfo();
	void GetCarDriveModeInfo();
        void GetCarLocationInfo();

        bool assignSint8( int8_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignUint8( uint8_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignSint16( int16_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignUint16( uint16_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignSint32( int32_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignUint32( uint32_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignUint64( uint64_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignFloat( float nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );
        bool assignDouble( double nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize );

        template <typename T>
        bool IsModuleNormal(T & type, double &dbPreTimeHeader,  double &dbPreTimeRead, float dfInternalTime);
        template <typename T>
        bool setPacketData(T nValue, uint8_t * pnData, uint32_t * pnDataLen);
        template <typename T>
        bool getPacketData(T * pnValue, uint8_t * pnData, uint32_t * pnDataLen);

        inline double RadToDeg (double rad) { return (rad / M_PI * 180.0); };
	inline double DegToRad (double deg) { return (deg / 180.0 * M_PI); };
	inline void ConvertCoord(double * pdbCoordX, double * pdbCoordY, double dbLon, double dbLat);
	inline double PointsDistance(double dbPoint1X, double dbPoint1Y, double dbPoint2X, double dbPoint2Y);

private:
        bool                            bIsTmcConnect_ = false;                                        /* 标示Tmc连接是否建立*/
        int32_t                      fdTmcSock_ = -1;                                                       /* Tmc传输socket*/
        fd_set                         fdTmcRead_;                                                               /* TMC链接读取事件集*/
        fd_set                         fdTmcException_;                                                     /* TMC链接异常事件集 */
        uint32_t                   nTmcTimeoutCount_ = 0;                                             /* Tmc链接超时次数 */
        uint8_t                      bufRecvData_[RMS_PACKET_LEN_MAX];                   /* Tmc接收数据缓冲区 */
        uint32_t                   nRecvDataLen_ = 0;                                                         /* TMC接收数据长度 */
        double                      dbHeartbeatPreSendTime_ = 0.0;                                 /* TMC心跳包上次发送时间 */
        double                      dbCarStatusPreSendTime_ =0.0;                                 /* TMC车辆状态包上次发送时间 */
	bool			    bNewTask_ = true;

        bool                            bIsRcConnect_ = false;                                            /* 标示远控TCP连接是否建立*/
        int32_t                      nRcLinkStatus_ = LS_UNUNITED;                      /* 远控TCP请求状态 */
        int32_t                      fdRcSockConnect_ = -1;                                         /* 远控TCPa传输socket*/
        int32_t                      fdRcSockData_ = -1;                                                 /* 远控UDP传输socket*/
        fd_set                         fdRcRead_;                                                                  /* 远控链接读取事件集*/
        fd_set                         fdRcException_;                                                        /* 远控链接异常事件集 */
        packetControl        stRcPackControl_;                                                    /* 远控控制报文数据 */
	packetActInfo         stRcPacketActInfo_;                                                /* 远控Act回传信息报文数据 */
        uint32_t                   nRcTimeoutCount_;                                                /* 远控链接超时次数 */
        uint8_t                      bufRcControl_[RMS_PACKET_LEN_CONTROL];     /* 远控控制报文数组 */
	sockaddr_in		   sockaddrTmc_;

        uint64_t                   nGKey_ = 0;                                                                     /* TMC下发任务GKEY值 */
	uint64_t		   nSubGKey_ = 0;
	uint64_t		   nTaskGKey_ = 0;
	uint64_t		   nLastGKey_ = 0;
        uint8_t                      nIDCar_[RMS_CAR_ID_LEN];                                  /* 集卡车的ID号 */
        double                      dbCurDstLon_ = 0.0;                                                   /* TMC下发任务的目的地坐标X */
        double                      dbCurDstLat_ = 0.0;                                                   /* TMC下发任务的目的地坐标Y */
	double			    dbCurDstCoordX_ = 0.0;
	double			    dbCurDstCoordY_ = 0.0;
	double			    dbLastDstLon_ = 0.0;
	double			    dbLastDstLat_ = 0.0;
	double			    dbPreDstLon_ = 0.0;
	double			    dbPreDstLat_ = 0.0;
	double			    dbPreCarCoordX_ = 0.0;
	double			    dbPreCarCoordY_ = 0.0;
        uint32_t                    nNumTaskCoord_ = 0;                                               /* TMC下发任务坐标个数 */
        double                      dbTaskCoord_[RMS_MAX_TASK_COORD];       /* TMC下发任务坐标 */
        uint8_t                     nControlType_ = 0;
        int16_t                     nControlDistance_ = 0;
	int16_t			    nGuidanceDistance_ = 10;
	int16_t			    nDistanceThreshold_ = 1000;
	double			    dbControlSpeed_ = 0.0;
        uint8_t                     nCarStatus_ = 0;
        uint8_t                     nTaskStatus_ = TS_NONE;
	uint8_t			    nTaskType_ = 0;
        uint8_t                     nDriveMode_ = DM_IDLE;
	uint8_t			    nDriveType_ = DT_AUTODRIVE;
	uint8_t			    nMachineType_ = 0;
	uint16_t		    nMachineNum_ = 0;
	uint8_t			    nMachineStatus_ = 0;
	uint8_t			    nBoxType_ = BT_40;
	uint8_t			    nBoxPosition_ = BP_CENTER;
	uint8_t			    nBoxPrePosition_ = BP_CENTER;

	double			    dbCarLon_ = 0.0;
	double			    dbCarLat_ = 0.0;
        double                      dbCarCoordX_ = 0.0;                                                  /* 集卡车GPS的经度值 */
        double                      dbCarCoordY_ = 0.0;                                                  /* 集卡车GPS的纬度值 */
        double                      dbCarDir_ = 0.0;                                                           /* 集卡车的航向角 */
        float                           dfCarSpeed_ = 0.0f;                                                    /* 集卡车的车速 */
        float                           dfCarThrottle_ = 0.0f;                                                /* 集卡车的油门值 */
        float                           dfCarBrake_ = 0.0f;                                                     /* 集卡车的刹车值 */
	uint32_t                   nCarTotalMileage_ = 0;                                             /* 集卡车的行驶总里程 */
        double                     dbCarBatteryTotalSoc_ = 0;                                      /* 集卡车的总的电池电量（单位：kwh） */
        double                     dbCarBatterySoc_ = 0;                                                /* 集卡车的电池剩余电量（单位：kwh） */
        uint32_t                   nCarBatteryTemp_ = 0;                                            /* 集卡车的电池温度 */
        canbus::Chassis::GearPosition infoGear_ = apollo::canbus::Chassis::GEAR_PARKING;         /* 集卡车的档位信息 */

        double                      dbChassisLastTimeHeader_ = 0.0;
        double                      dbChassisLastTimeRead_ = 0.0;
        double                      dbGuardianLastTimeHeader_ = 0.0;
        double                      dbGuardianLastTimeRead_ = 0.0;
	double                      dbTmcLastTimeHeader_ = 0.0;
        double                      dbTmcLastTimeRead_ = 0.0;
        double                      dbManualLastTimeHeader_ = 0.0;
        double                      dbManualLastTimeRead_ = 0.0;
        double                      dbLocationLastTimeHeader_ = 0.0;
        double                      dbLocationLastTimeRead_ = 0.0;

        std::mutex                                                     mutex_;
	std::mutex                                                     mutex_chassis_;
	std::mutex                                                     mutex_guardian_;
	std::mutex						       mutex_tmc_;
	std::mutex						       mutex_manuctrl_;
        std::mutex                                                     mutex_location_;
	std::mutex                                                     mutex_planning_;
	std::mutex						       mutex_task_;
        msgRmsConf                                               rms_conf_;
        canbus::Chassis                                          info_Chassis_;
	canbus::ChassisDetail					 info_Chassis_Detail_;
        guardian::GuardianCommand             info_Guardian_;
	tmc::tmcMsg					info_Tmc_;
	manuctrl::manuctrlMsg                          info_Manuctrl_;
        urgency::AEBCommand                          info_Aebs_;
        drivers::ContiRadar                                   info_Radar_;
        control::ControlCommand                    info_Control_;
        control::PadMessage                                info_Pad_;
	control::PadMessage			     info_Alignement_;
        drivers::gnss::GnssBestPose                  info_Gps_;
        planning::ADCTrajectory                         info_Planning_;
	planning::PadMessage				info_Planning_pad_;
        localization::LocalizationEstimate      info_Location_;
        relative_map::MapMsg                             info_Map_;
        perception::PerceptionObstacles        info_Perception_;
        drivers::Image                                              info_Camera_1_;
        drivers::Image                                              info_Camera_3_;
        apollo::rms::msgTaskCoord                   info_task_coord_;

        std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>> reader_Chassis_;
	std::shared_ptr<apollo::cyber::Reader<apollo::canbus::ChassisDetail>> reader_Chassis_Detail_;
        std::shared_ptr<apollo::cyber::Reader<apollo::guardian::GuardianCommand>> reader_Guardian_;
	std::shared_ptr<apollo::cyber::Reader<apollo::tmc::tmcMsg>> reader_Tmc_;
        std::shared_ptr<apollo::cyber::Reader<apollo::manuctrl::manuctrlMsg>> reader_Manuctrl_;
        std::shared_ptr<apollo::cyber::Reader<apollo::urgency::AEBCommand>> reader_Aebs_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::ContiRadar>> reader_Radar_;
        std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlCommand>> reader_Control_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::gnss::GnssBestPose>> reader_Gps_;
        std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> reader_Planning_;
        std::shared_ptr<apollo::cyber::Reader<apollo::localization::LocalizationEstimate>> reader_Location_;
        std::shared_ptr<apollo::cyber::Reader<apollo::relative_map::MapMsg>> reader_Map_;
        std::shared_ptr<apollo::cyber::Reader<apollo::perception::PerceptionObstacles>> reader_Perception_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Image>> reader_Camera_1_;
        std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Image>> reader_Camera_3_;
        std::shared_ptr<apollo::cyber::Writer<apollo::rms::msgTaskCoord>> writer_task_;
        std::shared_ptr<apollo::cyber::Writer<apollo::control::PadMessage>> writer_pad_;
	std::shared_ptr<apollo::cyber::Writer<apollo::control::PadMessage>> writer_alignement_;
	std::shared_ptr<apollo::cyber::Writer<apollo::planning::PadMessage>> writer_planning_pad_;

        std::unique_ptr<std::thread>  thread_net_tmcConnect_;
        std::unique_ptr<std::thread>  thread_net_tmcDataSend_;
        std::unique_ptr<std::thread>  thread_net_tmcDataRecv_;
        std::unique_ptr<std::thread>  thread_net_remoteConnect_;
        std::unique_ptr<std::thread>  thread_net_remoteDataSend_;
        std::unique_ptr<std::thread>  thread_net_remoteDataRecv_;
};

CYBER_REGISTER_COMPONENT(netRms)

}  // namespace tmc
}  // namespace apollo
