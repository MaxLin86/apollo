#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <cstdint>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <time.h>

#include "modules/rms/rms.h"
#include "modules/rms/proto/rms_conf.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/message_util.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/msf/common/util/frame_transform.h"



namespace apollo {
namespace rms {

using apollo::common::time::Clock;
using apollo::common::Point2D;
using apollo::canbus::Chassis;
using apollo::canbus::ChassisDetail;
using apollo::guardian::GuardianCommand;
using apollo::tmc::tmcMsg;
using apollo::manuctrl::manuctrlMsg;
using apollo::urgency::AEBCommand;
using apollo::drivers::ContiRadar;
using apollo::control::ControlCommand;
using apollo::control::PadMessage;
using apollo::control::DrivingAction;
using apollo::drivers::gnss::GnssBestPose;
using apollo::planning::ADCTrajectory;
using apollo::localization::LocalizationEstimate;
using apollo::relative_map::MapMsg;
using apollo::perception::PerceptionObstacles;
using apollo::drivers::Image;

bool netRms::Init()
{
        if (!GetProtoConfig(&rms_conf_)) 
        {
                AERROR << "Unable to load rms conf file: " << ConfigFilePath();
                return false;
        }

        AERROR << "ConfigFilePath = " << ConfigFilePath();
        AERROR << "Tmc servers ip = " << rms_conf_.server_tmc_ip().c_str();
        AERROR << "Tmc servers port = " << rms_conf_.server_tmc_port();
        AERROR << "Rc servers ip = " << rms_conf_.server_rc_ip().c_str();
        AERROR << "Rc servers port = " << rms_conf_.server_rc_port();

        InitData();
        InitChannel();

        signal( SIGPIPE, SIG_IGN );

        thread_net_tmcConnect_.reset(new std::thread(&netRms::netTmcConnect, this));
        thread_net_tmcDataRecv_.reset(new std::thread(&netRms::netTmcDataRecv, this));
        //thread_net_remoteConnect_.reset(new std::thread(&netRms::netRemoteConnect, this));
        //thread_net_remoteDataSend_.reset(new std::thread(&netRms::netRemoteDataSend, this));
        //thread_net_remoteDataRecv_.reset(new std::thread(&netRms::netRemoteDataRecv, this));

        AERROR << "Init function over!";

        return true;
}

bool netRms::Proc()
{
        //AERROR << "chassis header time = " << std::fixed << info_Chassis_.header().timestamp_sec();
        //AERROR << "Guardian header time = " << std::fixed << info_Guardian_.header().timestamp_sec();
        //AERROR << "Manuctrl header time = " << std::fixed << info_Manuctrl_.header().timestamp_sec();
        //AERROR << "Aebs header time = " << std::fixed << info_Aebs_.header().timestamp_sec();
        //AERROR << "ContiRadar header time = " << std::fixed << info_Radar_.header().timestamp_sec();
        //AERROR << "Control header time = " << std::fixed << info_Control_.header().timestamp_sec();
        //AERROR << "Gps header time = " << std::fixed << info_Gps_.header().timestamp_sec();
        //AERROR << "Planning header time = " << std::fixed << info_Planning_.header().timestamp_sec();
        //AERROR << "Localization header time = " << std::fixed << info_Location_.header().timestamp_sec();
        //AERROR << "Map header time = " << std::fixed << info_Map_.header().timestamp_sec();
        //AERROR << "Perception header time = " << std::fixed << info_Perception_.header().timestamp_sec();
        //AERROR << "Camera_1 header time = " << std::fixed << info_Camera_1_.header().timestamp_sec();
        //AERROR << "Camera_3 header time = " << std::fixed << info_Camera_3_.header().timestamp_sec();
        
       double dbAlignDistance = 9.0;

	GetCarChassisInfo();
	GetCarDriveModeInfo();
        GetCarLocationInfo();

        if( infoGear_ == apollo::canbus::Chassis::GEAR_PARKING && 
              fabs( dfCarBrake_ - 65.0f ) < 0.0001f && 
              nTaskGKey_ != 0 &&
	      nGuidanceDistance_ < nDistanceThreshold_ &&
	      (nTaskStatus_ == TS_ALIGNING) )
        {
		AERROR << "the car place is over, the task is done!";
		//assert( fabs(dbLastDstLon_) > 0.00001 && fabs(dbLastDstLat_) > 0.00001 );
		dbPreDstLon_ = dbLastDstLon_;
		dbPreDstLat_ = dbLastDstLat_;
		dbPreCarCoordX_ = dbCarCoordX_;
		dbPreCarCoordY_ = dbCarCoordY_;
                nTaskStatus_ = TS_DONE;
		nGuidanceDistance_ = 100;
        }

	if( nTaskStatus_ == TS_RUNNING )
        {       
                switch( nMachineType_ )
                {
                case 1:		dbAlignDistance = 9.0;	break;
                case 2: 	dbAlignDistance = 9.0;	break;
                case 3: 	dbAlignDistance = 9.0;	break;
                case 4: 	dbAlignDistance = 23.0;	break;
                default: 	dbAlignDistance = 9.0;	break;
                }

                if( PointsDistance(dbCurDstCoordX_, dbCurDstCoordY_, dbCarCoordX_, dbCarCoordY_) < dbAlignDistance )
                {
                        nTaskStatus_ = TS_PLACE;
                        AERROR << "the car has been in place!";
                }

		AERROR << "Align distance = " << std::fixed << PointsDistance(dbCurDstCoordX_, dbCurDstCoordY_, dbCarCoordX_, dbCarCoordY_);
        }

	if( ( nTaskStatus_ == TS_NONE ) && 
	    ( fabs(dbPreDstLon_ - dbCurDstLon_) < 0.00001 && fabs(dbPreDstLat_ - dbCurDstLat_) < 0.00001 ) &&
	    ( fabs(dbPreCarCoordX_ - dbCarCoordX_) > 3.0 || fabs(dbPreCarCoordY_ - dbCarCoordY_) > 3.0 ) )
	{
		if( fabs(dbPreCarCoordX_) > 0.01 || fabs(dbPreCarCoordY_) > 0.01 )
		{
			dbPreDstLon_ = 0.0;
			dbPreDstLat_ = 0.0;
			dbPreCarCoordX_ = 0.0;
			dbPreCarCoordY_ = 0.0;
			AERROR << "the car has been gone out of dst more than 10 meter!";
		}
	}

        usleep(1);

        return true;
}

void netRms::InitData()
{
        uint32_t nIndex = 0;
        memset(bufRecvData_, 0, sizeof(bufRecvData_));
        memset(bufRcControl_, 0, sizeof(bufRcControl_));
        memset(nIDCar_, 0, sizeof(nIDCar_));
        memset(dbTaskCoord_, 0, sizeof(dbTaskCoord_));

	dbCarBatteryTotalSoc_ = 149.76;

        for( nIndex = 0; nIndex < RMS_CAR_ID_LEN; nIndex++ )
        {
                nIDCar_[ nIndex ] = stol( rms_conf_.car_id( nIndex ) );
        }
}

void netRms::InitChannel()
{
	cyber::ReaderConfig reader_config;

        reader_Chassis_ = node_->CreateReader<Chassis> 
        (
                FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) 
                {
                        ADEBUG << "Received chassis data: run chassis callback.";
                        std::lock_guard<std::mutex> lock(mutex_);
                        info_Chassis_.CopyFrom(*chassis);
                }
        );

	reader_Chassis_Detail_ = node_->CreateReader<ChassisDetail>
        (
                FLAGS_chassis_detail_topic, [this](const std::shared_ptr<ChassisDetail>& chassis_detail)
                {
                        ADEBUG << "Received chassis detail data: run chassis detail callback.";
                        std::lock_guard<std::mutex> lock(mutex_chassis_);
                        info_Chassis_Detail_.CopyFrom(*chassis_detail);
                }
        );

        reader_Guardian_ = node_->CreateReader<GuardianCommand> 
        (
                FLAGS_guardian_topic, [this](const std::shared_ptr<GuardianCommand>&  guardian) 
                {
                        ADEBUG << "Received guardian data: run guardian callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Guardian_.CopyFrom( *guardian );
                }
        );

	reader_Tmc_ = node_->CreateReader<tmcMsg>
        (
                FLAGS_tmc_topic, [this](const std::shared_ptr<tmcMsg>&  tmc)
                {
                        ADEBUG << "Received tmc data: run tmc callback.";
                        std::lock_guard<std::mutex> lock( mutex_tmc_ );
                        info_Tmc_.CopyFrom( *tmc );
                }
        );

        reader_Manuctrl_ = node_->CreateReader<manuctrlMsg> 
        (
                FLAGS_manuctrl_topic, [this](const std::shared_ptr<manuctrlMsg>&  manuctrl) 
                {
                        ADEBUG << "Received manuctrl data: run manuctrl callback.";
                        std::lock_guard<std::mutex> lock( mutex_manuctrl_ );
                        info_Manuctrl_.CopyFrom( *manuctrl );
                }
        );

        reader_Aebs_ = node_->CreateReader<AEBCommand> 
        (
                FLAGS_aeb_command_topic, [this](const std::shared_ptr<AEBCommand>&  aebs) 
                {
                        ADEBUG << "Received aebs data: run aebs callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Aebs_.CopyFrom( *aebs );
                }
        );

        reader_Radar_ = node_->CreateReader<ContiRadar> 
        (
                FLAGS_front_radar_topic, [this](const std::shared_ptr<ContiRadar>&  contiRadar) 
                {
                        ADEBUG << "Received contiRadar data: run contiRadar callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Radar_.CopyFrom( *contiRadar );
                }
        );
	//reader_config.channel_name = FLAGS_front_radar_topic;
	//reader_config.pending_queue_size = 10;
	//reader_Radar_ = node_->CreateReader<ContiRadar>(reader_config, nullptr);
	//CHECK(reader_Radar_ != nullptr);

        reader_Control_ = node_->CreateReader<ControlCommand> 
        (
                FLAGS_control_command_topic, [this](const std::shared_ptr<ControlCommand>&  control) 
                {
                        ADEBUG << "Received control data: run contiRadar control.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Control_.CopyFrom( *control );
                }
        );

        reader_Gps_ = node_->CreateReader<GnssBestPose> 
        (
                FLAGS_gnss_best_pose_topic, [this](const std::shared_ptr<GnssBestPose>&  gps) 
                {
                        ADEBUG << "Received gps data: run gps callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Gps_.CopyFrom( *gps );
                }
        );

        reader_Planning_ = node_->CreateReader<ADCTrajectory> 
        (
                FLAGS_planning_trajectory_topic, [this](const std::shared_ptr<ADCTrajectory>&  planning) 
                {
                        ADEBUG << "Received planning data: run planning callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Planning_.CopyFrom( *planning );
                }
        );

        reader_Location_ = node_->CreateReader<LocalizationEstimate> 
        (
                FLAGS_localization_topic, [this](const std::shared_ptr<LocalizationEstimate>&  localization) 
                {
                        ADEBUG << "Received localization data: run localization callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Location_.CopyFrom( *localization );
                }
        );

        reader_Map_ = node_->CreateReader<MapMsg> 
        (
                FLAGS_relative_map_topic, [this](const std::shared_ptr<MapMsg>&  map) 
                {
                        ADEBUG << "Received map data: run map callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Map_.CopyFrom( *map );
                }
        );

        reader_Perception_ = node_->CreateReader<PerceptionObstacles> 
        (
                FLAGS_perception_obstacle_topic, [this](const std::shared_ptr<PerceptionObstacles>&  perception) 
                {
                        ADEBUG << "Received perception data: run perception callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Perception_.CopyFrom( *perception );
                }
        );

        reader_Camera_1_ = node_->CreateReader<Image> 
        (
                FLAGS_camera_front_1_topic , [this](const std::shared_ptr<Image>&  camera) 
                {
                        ADEBUG << "Received camera_1 data: run camera_1 callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Camera_1_.CopyFrom( *camera );
                }
        );

        reader_Camera_3_ = node_->CreateReader<Image> 
        (
                FLAGS_camera_front_3_topic , [this](const std::shared_ptr<Image>&  camera) 
                {
                        ADEBUG << "Received camera_3 data: run camera_3 callback.";
                        std::lock_guard<std::mutex> lock( mutex_ );
                        info_Camera_3_.CopyFrom( *camera );
                }
        );

        writer_task_ = node_->CreateWriter<msgTaskCoord>(FLAGS_rms_task_topic);
        writer_pad_ = node_->CreateWriter<PadMessage>(FLAGS_pad_topic);
	writer_alignement_ = node_->CreateWriter<PadMessage>(FLAGS_alignement_topic);
	writer_planning_pad_ = node_->CreateWriter<planning::PadMessage>(FLAGS_planning_pad_topic);
}

void netRms::resetTmcStatus()
{
	nGKey_ = 0;
	nSubGKey_ = 0;
	nTaskGKey_ = 0;
	nTaskStatus_ = TS_NONE;
	nTmcTimeoutCount_ = 0;
	memset( bufRecvData_, 0, sizeof( bufRecvData_ ) );
	nRecvDataLen_ = 0;

	AERROR << "reset tmc status!";
}

void netRms::netTmcConnect(void)
{
        int32_t ret = -1;
	int32_t opt = 1;
        double dbCurTime = 0.0;
        struct sockaddr_in server_address; 

        signal( SIGPIPE, SIG_IGN );

        if( bIsTmcConnect_ )
        {
                close(fdTmcSock_);
                fdTmcSock_ = -1;
                bIsTmcConnect_ = false;
		resetTmcStatus();
        }

        while( 1 )
        {
                if( !bIsTmcConnect_ )
                {
                        AERROR << "connect socket = " << fdTmcSock_;
                        fdTmcSock_ = socket(AF_INET, SOCK_STREAM, 0);

                        if(fdTmcSock_ < 0)
                        {
                                AERROR << "socket创建异常，退出程序！errno = " << errno;
                                return ;
                        }

			if (setsockopt(fdTmcSock_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
                        {
                                AERROR << "设置socket属性失败! errno = " << errno;
                                return ;
                        }

                        server_address.sin_family = AF_INET; 
                        server_address.sin_addr.s_addr = inet_addr(rms_conf_.server_tmc_ip().c_str());
                        server_address.sin_port = htons(rms_conf_.server_tmc_port());

                        while( 1 )
                        {
                                AERROR << "try to connect tmc. server ip = " << rms_conf_.server_tmc_ip() << " port = " << (uint32_t)rms_conf_.server_tmc_port();
                                ret = connect(fdTmcSock_, (struct sockaddr *)&server_address, sizeof(struct sockaddr_in));

                                if( ret == 0 )
                                {
                                        bIsTmcConnect_ = true;
                                        AERROR << "Connect is Success!";
                                        break;
                                }
                                else
                                {
                                        AERROR << "connect errno = " << errno;
                                        usleep(3000000);
                                }
                        }

                        while( 1 )
                        {
                                dbCurTime = Clock::NowInSeconds() * 1000.0;

                                // 发送心跳报文
                                if(dbCurTime - dbHeartbeatPreSendTime_ > 1000.0)
                                {
					std::lock_guard<std::mutex> lock(mutex_task_);
                                        dbHeartbeatPreSendTime_ = dbCurTime;
					nTmcTimeoutCount_++;

					if( nTmcTimeoutCount_ > 3 )
					{
						nTmcTimeoutCount_ = 0;
						AERROR << "the heartbeat packet timeout!";
						break;
					}

                                        if( !sendTmcHeartbeatPacket() )
                                        {
                                                break;
                                        }

                                        switch( nTaskStatus_ )
                                        {
					case TS_DONE:
                                                nGKey_ = 0;
						nSubGKey_ = 0;
						nTaskGKey_ = 0;
                                                nTaskStatus_ = TS_NONE;
						break;

					case TS_PLACE:
						if( nMachineStatus_ == 1 )
						{
							nTaskStatus_ = TS_ALIGNING;
						}
						break;
					default:
						break;
                                        }
                                }

                                // 发送车辆状态报文
                                if( dbCurTime - dbCarStatusPreSendTime_ > 2000.0 )
                                {
                                        dbCarStatusPreSendTime_ = dbCurTime;

                                        if( !sendTmcCarStatusPacket() )
                                        {
                                                break;
                                        }
                                }

                                usleep(1);
                        }

                        AERROR << "tmc close socket  = " << fdTmcSock_;
                        close(fdTmcSock_);
                        fdTmcSock_ = -1;
                        bIsTmcConnect_ = false;
			resetTmcStatus();
                }
                else
                {
                        usleep(1000000);
                }
        }
}

void netRms::netTmcDataRecv(void)
{
        int32_t nReadLen = 0;
        uint8_t   bufRecv[1024];

        signal( SIGPIPE, SIG_IGN );

        while( 1 )
        {
                while ( bIsTmcConnect_ )
                {
                        nReadLen = read( fdTmcSock_, bufRecv, sizeof(bufRecv));

                        if( nReadLen < 0 )
                        {
                                AERROR << "read errno = " << errno;
                                break;
                        }

                        if( nReadLen > 0 )
                        {
                                AERROR << "Tmc connect have something to read!  nReadLen = " << nReadLen;
                                memcpy(&bufRecvData_[nRecvDataLen_], bufRecv, nReadLen);
                                nRecvDataLen_ += nReadLen;

                                while( 1 )
                                {
                                        if( !analysisPacketRaw(bufRecvData_, &nRecvDataLen_) )
                                        {
                                                break;
                                        }
                                }
                        }

                        usleep(1000);
                }

                usleep(1000);
        }
}

void netRms::netRemoteConnect(void)
{
        int32_t ret = -1;
        int32_t nSelectCount = 0;
        int32_t nReadLen = -1;
        bool       bError = false;
        struct sockaddr_in server_address;  
        uint8_t bufRecv[1024];
        timeval tv;

        if( bIsRcConnect_ )
        {
                close(fdRcSockConnect_);
                fdRcSockConnect_ = -1;
                bIsRcConnect_ = false;
        }

        while( !bIsRcConnect_ )
        {
                fdRcSockConnect_ = socket(AF_INET, SOCK_STREAM, 0);
                AERROR << "connect socket = " << fdRcSockConnect_;

                if(fdRcSockConnect_ < 0)
                {
                        AERROR << "socket创建异常，退出程序！errno = " << errno;
                        return ;
                }

                server_address.sin_family = AF_INET; 
                server_address.sin_addr.s_addr = inet_addr(rms_conf_.server_rc_ip().c_str());
                server_address.sin_port = htons(rms_conf_.server_rc_port());

                while( true )
                {
                        AERROR << "try to connect remote control. server ip = " << rms_conf_.server_rc_ip();
                        ret = connect(fdRcSockConnect_, (struct sockaddr *)&server_address, sizeof(struct sockaddr_in));

                        if( ret == 0 )
                        {
                                bIsRcConnect_ = true;
                                nRcLinkStatus_ = LS_UNUNITED;
                                AERROR << "Connect is Success!";
                                break;
                        }
                        else
                        {
                                AERROR << "connect errno = " << errno;
                                usleep(10000000);
                        }
                }

                while(1)
                {
                        bError = false;
                        FD_ZERO(&fdRcRead_);
                        FD_ZERO(&fdRcException_);
                        FD_SET(fdRcSockConnect_, &fdRcRead_); //添加描述符  
                        FD_SET(fdRcSockConnect_, &fdRcException_); //添加描述符  

                        // 发送报文
                        switch( nRcLinkStatus_ )
                        {
                        case LS_UNUNITED:
                        case LS_CONNECT_REQ:
                                nRcLinkStatus_ = LS_CONNECT_REQ;
                                ret = sendRcControlPacket(0);
                                break;
                        case LS_CONNECTING:
                        case LS_HEARTBEAT_REQ:
                                nRcLinkStatus_ = LS_HEARTBEAT_REQ;
                                ret = sendRcControlPacket(2);
                                break;
                        case LS_CLOSE_REQ:
                                ret = sendRcControlPacket(4);
                                ret = false;    /* 设置为false 直接退出链接 */
                                break;
                        default:
                                AERROR << "the rc link status is invalid!";
                                assert(0);
                                break;
                        }

                        // 接收报文
                        if( ret == true )
                        {
                                tv.tv_sec = 3;
                                tv.tv_usec = 0;
                                nSelectCount = select(fdRcSockConnect_+1, &fdRcRead_, NULL, &fdRcException_, &tv);
                                AERROR << "select result = " << nSelectCount;

                                if( nSelectCount < 0 )
                                {
                                        AERROR << "程序select异常退出，errno = " << errno;
                                        bError = true;
                                }
                                else
                                {
                                        while( nSelectCount > 0 )
                                        {
                                                if( FD_ISSET(fdRcSockConnect_, &fdRcException_) )
                                                {
                                                        AERROR << "connect have something to exception!";
                                                        bError = true;
                                                        break;
                                                }

                                                if( FD_ISSET(fdRcSockConnect_, &fdRcRead_) )
                                                {
                                                        AERROR << "connect have something to read!";
                                                        nReadLen = recv(fdRcSockConnect_, bufRecv, sizeof(bufRecv) - 1, 0);

                                                        if(nReadLen <= 0)
                                                        {
                                                                AERROR << "read errno = " << errno;
                                                                bError = true;
                                                                break;
                                                        }

                                                        if( (nReadLen == RMS_PACKET_LEN_CONTROL) && ( bufRecv[0] == 1 ) )
                                                        {
                                                                recvRcControlPacket(bufRecv, nReadLen);
                                                        }
                                                }

                                                nSelectCount--;
                                        }

                                        if( (nRcLinkStatus_ == LS_CONNECT_REQ ) || ( nRcLinkStatus_ == LS_HEARTBEAT_REQ ) )
                                        {
                                                nRcTimeoutCount_++;
                                                AERROR << "lost heartbeat packet! the count = " << nRcTimeoutCount_;

                                                if( nRcTimeoutCount_ >= 3 )
                                                {
                                                        AERROR << "lost heartbeat packet is equal to 3, to close connect! ";
                                                        bError = true;
                                                }
                                        }
                                        else
                                        {
                                                nRcTimeoutCount_ = 0;
                                        }
                                }
                        }
                        else
                        {
                                bError = true;
                        }

                        if( bError )
                        {
                                close(fdRcSockConnect_);
                                AERROR << "close socket  = " << fdRcSockConnect_;
                                fdRcSockConnect_ = -1;
                                bIsRcConnect_ = false;
                                nRcLinkStatus_ = LS_UNUNITED;
                                nRcTimeoutCount_ = 0;
                                break ;
                        }
                }
        }
}

void netRms::netRemoteDataSend(void)
{

}

void netRms::netRemoteDataRecv(void)
{

}

bool netRms::sendTmcHeartbeatPacket()
{
        AERROR << "ready to send heartbeat data!";

        uint32_t nIndex = 0;
        uint16_t nCarSpeed = (uint16_t) ( dfCarSpeed_ + 0.5f );
        uint32_t nPacketSize = 0;
        uint8_t    nDataBuf[ RMS_MAX_SEND_BUF_SIZE ];
        bool bRet = true;

        bRet = assignUint8( RMS_VERSION_PROTO, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( 0x10, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint32( 110, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint64( nGKey_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint64( nSubGKey_, &nDataBuf[nPacketSize], &nPacketSize );

        for(nIndex = 0; nIndex < 64; nIndex++)
        {
                bRet = assignUint8( nIDCar_[nIndex], &nDataBuf[nPacketSize], &nPacketSize );
        }

        bRet = assignDouble( dbCarLon_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignDouble( dbCarLat_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignDouble( dbCarDir_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint16( nCarSpeed, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( nCarStatus_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( nTaskStatus_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( nDriveType_, &nDataBuf[nPacketSize], &nPacketSize );
	bRet = assignUint8( bIsRcConnect_, &nDataBuf[nPacketSize], &nPacketSize );

        if( bRet )
        {
                if( send( fdTmcSock_, nDataBuf, nPacketSize, 0 ) < 0 )
                {
                        AERROR << "tmc send data is error, errno = " << errno;
                        bRet = false;
                }
        }

        return bRet;
}

bool netRms::sendTmcCarStatusPacket()
{
        AERROR << "ready to send car status data!";

        uint32_t nIndex = 0;
        uint32_t nPacketSize = 0;
        uint8_t    nDataBuf[ RMS_MAX_SEND_BUF_SIZE ];
        bool bRet = true;

        bRet = assignUint8( RMS_VERSION_PROTO, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( 0x12, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint32( 0, &nDataBuf[nPacketSize], &nPacketSize );

        for(nIndex = 0; nIndex < 64; nIndex++)
        {
                bRet = assignUint8( nIDCar_[nIndex], &nDataBuf[nPacketSize], &nPacketSize );
        }

        bRet = assignUint16( 0x02, &nDataBuf[nPacketSize], &nPacketSize );

        // 填充保养信息包
        bRet = assignUint8( 0x10, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint16( 12, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint16( 20, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( 12, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( 22, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint32( nCarTotalMileage_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint16( 20, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( 12, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint8( 22, &nDataBuf[nPacketSize], &nPacketSize );

        // 填充电池信息包
        bRet = assignUint8( 0x11, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint16( 18, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignDouble( dbCarBatteryTotalSoc_, &nDataBuf[nPacketSize], &nPacketSize );
	bRet = assignDouble( dbCarBatterySoc_, &nDataBuf[nPacketSize], &nPacketSize );
        bRet = assignUint16( nCarBatteryTemp_, &nDataBuf[nPacketSize], &nPacketSize );

        //设置包长
        bRet = assignUint32( nPacketSize - 6, &nDataBuf[2], &nIndex );

        if( bRet )
        {
                if( send( fdTmcSock_, nDataBuf, nPacketSize, 0 ) < 0 )
                {
                        AERROR << "tmc send data is error, errno = " << errno;
                        bRet = false;
                }
        }

        return bRet;
}

bool netRms::sendRcControlPacket( uint32_t nCmd )
{
        AERROR << "Send Control Cmd = " << nCmd;
        bool ret = true;
        uint32_t nPackLen = RMS_PACKET_LEN_CONTROL;
        uint32_t nRemainLen = RMS_PACKET_LEN_CONTROL;

        if( nCmd > 5 ) 
        {
                AERROR << "Send Control Packet Cmd Error!"; 
                assert(0);
        }

        if( nCmd == 2 )
        {
                usleep(5000000);
        }

        // 封装数据包
        if( ret == true )
        {
                stRcPackControl_.nCmd = nCmd;
                ret = setPacketData(stRcPackControl_.nVersion, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nCmd, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nServerRandData, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nClientRandData, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nChassisIP, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nChassisPort, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nServerIP, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nServerPort, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nChassisProto, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nActIP, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nActPort, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nActProto, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nServerPeriod, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nClientPeriod, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
                ret = setPacketData(stRcPackControl_.nErrorCode, &bufRcControl_[nPackLen - nRemainLen], &nRemainLen);
        }

        if( ret == true )
        {
                if( send( fdRcSockConnect_, bufRcControl_, nPackLen, 0 ) != nPackLen )
                {
			AERROR << "Send remote Control data failed! errno = " << errno;
                        ret = false;
                }
        }
	else
	{
		AERROR << "Packet remote control data failed!";
	}

        return ret;
}

bool netRms::sendActInfoPacket()
{
        //AERROR << "Send Act Info";
        bool ret = true;
        uint32_t nIndex = 0;
        uint8_t nSendBuf[1024];
        uint32_t nSendDataSize = 0;
        uint32_t nPackLen = sizeof( nSendBuf );
        uint32_t nRemainLen = nPackLen;
        ssize_t stSize;

        /* 打包IPC回传信息包包头部分 */
        ret = setPacketData(stRcPacketActInfo_.nVersion, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.nRcStatus, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.nControlHost, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.nAebsEnable, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.nPacketSequence, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.nRandom, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

        /* 打包IPC状态包部分 */
        ret = setPacketData(stRcPacketActInfo_.stIpcStatus.nCode, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stIpcStatus.nLen, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stIpcStatus.nDelay, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stIpcStatus.nCpuUsed, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stIpcStatus.nMemoryUsed, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stIpcStatus.nCpuTemp, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

         /* 打包错误代码部分 */
        ret = setPacketData(stRcPacketActInfo_.stErrorCode.nCode, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stErrorCode.nLen, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stErrorCode.nDTC, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stErrorCode.nErrorType, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

        /* 打包传感器状态部分 */
        ret = setPacketData(stRcPacketActInfo_.stSensorStatus.nCode, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stSensorStatus.nLen, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stSensorStatus.nWmRadar, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stSensorStatus.nUtRadar, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stSensorStatus.nIMU, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stSensorStatus.nCamera, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

        /* 打包模块状态部分 */
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nCode, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nLen, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nRadar, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nTmc, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nGuardian, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nCanbus, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nPlanning, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stModuleStatus.nPerception, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

        /* 打包超声波雷达状态部分 */
        ret = setPacketData(stRcPacketActInfo_.stRadarUT.nCode, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stRadarUT.nLen, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

        /* 打包毫米波雷达状态部分 */
        ret = setPacketData(stRcPacketActInfo_.stRadarWm.nCode, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        ret = setPacketData(stRcPacketActInfo_.stRadarWm.nLen, &nSendBuf[nPackLen - nRemainLen], &nRemainLen);

        /* 打包校验和 */
        for( nIndex = 0; nIndex < 16; nIndex++ )
        {
                ret = setPacketData(stRcPacketActInfo_.nCheckSum[nIndex], &nSendBuf[nPackLen - nRemainLen], &nRemainLen);
        }

        // 发送IPC信息到TMC服务端
        if ( ret == true )
        {
                nSendDataSize = nPackLen - nRemainLen;
                stSize = sendto(fdRcSockData_,
                                                nSendBuf,
                                                nSendDataSize,
                                                MSG_CONFIRM,
                                                (const struct sockaddr *) &sockaddrTmc_,
                                                sizeof( sockaddrTmc_ ));

                if( stSize != nSendDataSize )
                {
                        AERROR << "Send remote IPC info data failed! errno = " << errno;
                        ret = false;
                }
        }
        else
        {
                AERROR << "Packet remote IPC info data failed!";
        }

        return ret;
}

bool netRms::recvRcControlPacket(uint8_t * pnData, uint32_t nDataLen)
{
        bool ret = true;
        uint32_t nRemainLen = nDataLen;
        packetControl  stControlData;

        uint32_t nRecvCmd = 0;

        ret = getPacketData(&stControlData.nVersion, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nCmd, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nServerRandData, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nClientRandData, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nChassisIP, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nChassisPort, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nServerIP, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nServerPort, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nChassisProto, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nActIP, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nActPort, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nActProto, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nServerPeriod, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nClientPeriod, &pnData[nDataLen - nRemainLen], &nRemainLen);
        ret = getPacketData(&stControlData.nErrorCode, &pnData[nDataLen - nRemainLen], &nRemainLen);

        if( ret == true )
        {
                switch(nRcLinkStatus_)
                {
                case LS_CONNECT_REQ:
                        nRecvCmd = stControlData.nCmd;
                        AERROR << "receive cmd = " << nRecvCmd;
                        if( stControlData.nCmd == 1 )
                        {
                                nRcLinkStatus_ = LS_CONNECTING;
                                stRcPackControl_.nServerIP = stControlData.nServerIP;
                                stRcPackControl_.nServerPort = stControlData.nServerPort;
                        }
                        break;

                case LS_HEARTBEAT_REQ:
                        nRecvCmd = stControlData.nCmd;
                        AERROR << "receive cmd = " << nRecvCmd;
                        if( stControlData.nCmd == 3 )
                        {
                                nRcLinkStatus_ = LS_CONNECTING;
                        }
                        break;

                default:
                        break;
                }
        }
        else
        {
                AERROR << "the ret = false";
        }

        return ret;
}

bool netRms::analysisPacketRaw(uint8_t * pData, uint32_t * pnDataLen)
{
        uint32_t nDataLen = *pnDataLen;
        uint32_t nIndex = 0;
        uint8_t nCmd = 0;
        uint32_t nLen = 0;
        bool bRet = true;

        if(nDataLen < 6)
        {
                bRet = false;
        }

        if(bRet == true)
        {
                while( pData[nIndex] != RMS_VERSION_PROTO )
                {
                        if(nIndex >= nDataLen)
                        {
                                *pnDataLen = 0;
                                bRet = false;
                                break;
                        }
                        else
                        {
                                nIndex++;
                        }
                }
        }

        if(bRet == true)
        {
                nCmd = pData[nIndex + 1];
                memcpy( &nLen, &pData[nIndex + 2], 4 );

                if( nDataLen < ( nLen + 6 ) )
                {
			AERROR << "olddata1[" << nIndex << "] = " << (uint32_t)pData[nIndex] << ", olddata2[" << nDataLen << "] = " << (uint32_t)pData[nDataLen-1];
                        memcpy( pData, &pData[nIndex], nDataLen - nIndex );
			AERROR << "newdata1[0] = " << (uint32_t)pData[0] << "newdata2[" << nDataLen - nIndex - 1 << "] = " << (uint32_t)pData[nDataLen - nIndex - 1];
                        *pnDataLen = nDataLen - nIndex;
                        AERROR << "nDataLen(" << nDataLen << " < nLen + 6(" << nLen + 6 << ")!";
                        bRet = false;
                }
        }

        if(bRet == true)
        {
                switch( nCmd )
                {
                case 0x20:      bRet = analysisPacketTask( &pData[nIndex] );            break;
                case 0x21:      bRet = analysisPacketControl( &pData[nIndex] );		break;
		case 0x22:	bRet = analysisPacketResponse( &pData[nIndex] );	break;
                default:	break;
                }

		/* 清除心跳未应答 */
                if( (nCmd == 0x22) && (bRet == true) )
                {
                        //AERROR << "clear heartbeat response!";
                        nTmcTimeoutCount_ = 0;
                }

		if( bRet == true )
                {
			if( (nDataLen - nIndex - nLen - 6) > 0 )
			{
				AERROR << "olddata1[" << nIndex + nLen + 6 << "] = " << (uint32_t)pData[nIndex + nLen + 6] << ", olddata2[" << nDataLen << "] = " << (uint32_t)pData[nDataLen-1];
                		memcpy(pData, &pData[nIndex + nLen + 6], nDataLen - nIndex - nLen - 6);
				AERROR << "nDataLen - nIndex - nLen - 6 = " << nDataLen - nIndex - nLen - 6;
				AERROR << "newdata1[0] = " << (uint32_t)pData[0] << "newdata2[" << nDataLen - nIndex - nLen - 7 << "] = " << pData[nDataLen - nIndex - nLen - 7];
			}

                        *pnDataLen = nDataLen - nIndex - nLen - 6;
                }
                else
                {
			if( (nDataLen - nIndex - 1) > 0 )
			{
				AERROR << "olddata1[" << nIndex + 1 << "] = " << (uint32_t)pData[nIndex + 1] << ", olddata2[" << nDataLen << "] = " << (uint32_t)pData[nDataLen-1];
                        	memcpy(pData, &pData[nIndex + 1], nDataLen - nIndex - 1);
				AERROR << "newdata1[0] = " << (uint32_t)pData[0] << "newdata2[" << nDataLen - nIndex - 2 << "] = " << pData[nDataLen - nIndex - 2];
			}

                	*pnDataLen = nDataLen - nIndex - 1;
		}
        }

        return bRet;
}

bool netRms::analysisPacketTask( uint8_t * pData )
{
        uint32_t nIndex = 0;
        uint64_t nGKey = 0;
	uint64_t nSubGKey = 0;
        uint8_t nIDCarRead[RMS_CAR_ID_LEN];
        Point2D  pointDst;
        Point2D * pPoint2D;
	double dbDstUtmX = 0.0;
	double dbDstUtmY = 0.0;
	bool bTaskRepeat = false;
        bool bRet = true;

        memcpy( &nGKey, &pData[6], sizeof(uint64_t) );
	memcpy( &nSubGKey, &pData[14], sizeof(uint64_t) );
        memcpy( nIDCarRead, &pData[22], RMS_CAR_ID_LEN );

        for( nIndex = 0; nIndex < RMS_CAR_ID_LEN; nIndex++ )
        {
                if( nIDCarRead[nIndex] != nIDCar_[nIndex] )
                {
                        AERROR << "the id of act is Error!  recv car id[" << nIndex << "] = (" << (uint32_t)nIDCarRead[nIndex] << ")     active car id[" << nIndex << "] = (" << (uint32_t)nIDCar_[nIndex] << ")";
                        bRet = false;
                        break;
                }
        }

        if( bRet == true )
        {
                memcpy( &nMachineType_, &pData[86], sizeof(uint8_t) );
                memcpy( &nMachineNum_, &pData[87], sizeof(uint8_t) );
                memcpy( &nMachineStatus_, &pData[89], sizeof(uint8_t) );
		memcpy( &nTaskType_, &pData[90], sizeof(uint8_t) );
		memcpy( &nBoxType_, &pData[91], sizeof(uint8_t) );
		memcpy( &nBoxPosition_, &pData[92], sizeof(uint8_t) );
                memcpy( &dbCurDstLon_, &pData[93], sizeof(double) );
                memcpy( &dbCurDstLat_, &pData[101], sizeof(double) );
                memcpy( &nNumTaskCoord_, &pData[109], sizeof(uint16_t) );
                AERROR << "the packet is recv success! the num of Coord is " << nNumTaskCoord_;
	        AERROR << "the dst coord = [" << std::fixed << dbCurDstLon_ << ", " << std::fixed << dbCurDstLat_ << "]";

                if( nNumTaskCoord_ > RMS_MAX_TASK_COORD )
                {
                        AERROR << "the num of task coord is " << nNumTaskCoord_ << ", more than max num = " << RMS_MAX_TASK_COORD << "!";
                        bRet = false;
                }

		ConvertCoord( &dbDstUtmX, &dbDstUtmY, dbCurDstLon_, dbCurDstLat_ );
        }

        if( bRet == true )
        {
		std::lock_guard<std::mutex> lock(mutex_task_);

                info_task_coord_.clear_task_coord();
                common::util::FillHeader(node_->Name(), &info_task_coord_);
                info_task_coord_.mutable_dst_coord()->set_x( dbCurDstLon_ );
                info_task_coord_.mutable_dst_coord()->set_y( dbCurDstLat_ );

		if( nTaskType_ == 0 )
		{
			dbLastDstLon_ = 0.0;
                        dbLastDstLat_ = 0.0;
                        info_task_coord_.mutable_dst_coord()->set_x( 0.0 );
                        info_task_coord_.mutable_dst_coord()->set_y( 0.0 );
                        info_task_coord_.set_task_flag( TF_NEW );
                        writer_task_->Write(info_task_coord_);
                        nGKey_ = 0;
                        nSubGKey_ = 0;
                        nTaskGKey_ = 0;
                        nTaskStatus_ = TS_NONE;

                        AERROR << "----------the tmc had canceled the task!----------";
		}
		else if( ( nTaskType_ > 0 ) && ( nTaskType_ < 9 ) )
                {
			info_task_coord_.set_task_type( nTaskType_ );
			info_task_coord_.set_machine_type( nMachineType_ );
			info_task_coord_.set_machine_num( nMachineNum_ );
			info_task_coord_.set_machine_status( nMachineStatus_ );
			info_task_coord_.set_box_type( nBoxType_ );
			info_task_coord_.set_box_position( nBoxPosition_ );

			switch( nMachineType_ )
			{
			case 1:	nDistanceThreshold_ = 15;	break;
			case 2:	nDistanceThreshold_ = 5;	break;
			default:nDistanceThreshold_ = 1000;	break;
			}

		 	if( fabs(dbPreDstLon_ - dbCurDstLon_) > 0.000001 || fabs(dbPreDstLat_ - dbCurDstLat_) > 0.0000001 )
                        {
                                bTaskRepeat = false;
                        }
                        else
                        {
                                if( nBoxPosition_ == nBoxPrePosition_ )
                                {
                                        bTaskRepeat = true;
                                }
                                else
                                {
                                        bTaskRepeat = false;
                                }
                        }

                        if( bTaskRepeat )
                        {
                                nGKey_ = nGKey;
                                nSubGKey_ = nSubGKey;
                                nTaskGKey_ = nGKey_ + nSubGKey_;
                                nTaskStatus_ = TS_DONE;
                                AERROR << "the task is repeat!";
                        }
                        else
                        {
                                for( nIndex = 0; nIndex < nNumTaskCoord_; nIndex++ )
                                {
                                        memcpy(&dbTaskCoord_[nIndex * 2], &pData[111 + nIndex * 16], sizeof(double));
                                        memcpy(&dbTaskCoord_[nIndex * 2 + 1], &pData[111 + nIndex * 16 + 8], sizeof(double));
                                        pPoint2D = info_task_coord_.add_task_coord();
                                        pPoint2D->set_x( dbTaskCoord_[nIndex * 2] );
                                        pPoint2D->set_y( dbTaskCoord_[nIndex * 2 + 1] );
                                        AERROR << "dbTaskCoord_[" << nIndex <<"] = (" << std::fixed << dbTaskCoord_[nIndex * 2] << ", " << std::fixed << dbTaskCoord_[nIndex * 2 + 1] << ")!";
                                }

                                if( nGKey_ == nGKey && nSubGKey_ == nSubGKey )
                                {
                                        info_task_coord_.set_task_flag( TF_OLD );
                                        AERROR << "the task is old!";
                                }
                                else
                                {
                                        info_task_coord_.set_task_flag( TF_NEW );
                                        nTaskStatus_ = TS_RUNNING;

                                        AERROR << "the new GKey = " << nGKey << "  the old GKey = " << nGKey_;
                                        AERROR << "the new subGKey = " << nSubGKey << "the old SubGKey = " << nSubGKey_;
                                        AERROR << "the task is new!";
                                }

				nBoxPrePosition_ = nBoxPosition_;
                                dbLastDstLon_ = dbCurDstLon_;
                                dbLastDstLat_ = dbCurDstLat_;
                                ConvertCoord( &dbCurDstCoordX_, &dbCurDstCoordY_, dbCurDstLon_, dbCurDstLat_ );
                                AERROR << "the dst lonlat = [" << std::fixed << dbCurDstLon_ << ", " << std::fixed << dbCurDstLat_ << "]";
                                AERROR << "the dst coord = [" << std::fixed << dbCurDstCoordX_ << ", " << std::fixed << dbCurDstCoordY_ << "]";
                                writer_task_->Write(info_task_coord_);

                                info_Pad_.clear_action();
                                common::util::FillHeader(node_->Name(), &info_Pad_);
                                info_Pad_.set_action( apollo::control::DrivingAction::RESET );
                                info_Pad_.set_moving_distance( 0 );
                                writer_pad_->Write(info_Pad_);

                                nGKey_ = nGKey;
                                nSubGKey_ = nSubGKey;
                                nTaskGKey_ = nGKey_ + nSubGKey_;
                                AERROR << "the car had recv task!";
                        }
                }
                else
                {
                        AERROR << "the task had received is abnormal! " << nNumTaskCoord_ << std::fixed << dbCurDstLon_ << std::fixed << dbCurDstLat_;
			bRet = false;
		}
        }

        return bRet;
}

bool netRms::analysisPacketControl( uint8_t * pData )
{
        uint32_t nIndex = 0;
        uint8_t nIDCarRead[RMS_CAR_ID_LEN];
        bool bAction = true;
        bool bRet = true;

        memcpy(nIDCarRead, &pData[6], RMS_CAR_ID_LEN);

        for(nIndex = 0; nIndex < RMS_CAR_ID_LEN; nIndex++)
        {
                if(nIDCarRead[nIndex] != nIDCar_[nIndex])
                {
                        AERROR << "the id of act is Error!  recv car id[" << nIndex << "] = (" << (uint32_t)nIDCarRead[nIndex] << ")     active car id[" << nIndex << "] = (" << (uint32_t)nIDCar_[nIndex] << ")";
                        bRet = false;
                        break;
                }
        }

        if( bRet == true )
        {
                memcpy(&nControlType_, &pData[70], sizeof(nControlType_));
                memcpy(&nControlDistance_, &pData[71], sizeof(nControlDistance_));
		memcpy(&dbControlSpeed_, &pData[73], sizeof(dbControlSpeed_));
                AERROR << "nControlType = " << (uint32_t)nControlType_ << ",  nControlDistance = " << nControlDistance_ << "!";
		AERROR << "dbControlSpeed = " << std::fixed << dbControlSpeed_;

		if( nControlType_ < 4 )
		{
                	switch ( nControlType_ )
                	{
                	case 0: bAction = false; nControlDistance_ = 0;  break;
                	case 1: bAction = true;  nControlDistance_ = 0;  break;
                	case 2: bAction = true;  nControlDistance_ = nControlDistance_;  break;
                	case 3: bAction = true;  nControlDistance_ = -nControlDistance_;  break;
                	default:        break;
                	}

                	info_Pad_.clear_action();
                	common::util::FillHeader(node_->Name(), &info_Pad_);
                	if( bAction )
                	{
                        	info_Pad_.set_action( apollo::control::DrivingAction::RESET );
                	}
                	info_Pad_.set_moving_distance( nControlDistance_ );

                	writer_pad_->Write(info_Pad_);
		}
		else if( nControlType_ < 6 )
		{
			switch( nControlType_ )
			{
			case 4:	nGuidanceDistance_ = nControlDistance_;		break;
			case 5:	nGuidanceDistance_ = - nControlDistance_;	break;
			default:nGuidanceDistance_ = 100;			break;
			}

			info_Alignement_.Clear();
			common::util::FillHeader( node_->Name(), &info_Alignement_ );
			info_Alignement_.set_moving_distance( nGuidanceDistance_ * 0.01 );

			if( ((nGuidanceDistance_ > -200) && (nGuidanceDistance_ < -4)) || 
			    ((nGuidanceDistance_ > 4) && (nGuidanceDistance_ < 200)) )
			{
				writer_alignement_->Write( info_Alignement_ );
			}
		}
		else
		{
			if( nControlType_ == 6 )
			{
				if( (dbControlSpeed_ >= 0) && (dbControlSpeed_ <= 30.0) )
				{
					info_Planning_pad_.Clear();
					common::util::FillHeader( node_->Name(), &info_Planning_pad_ );
					info_Planning_pad_.set_speed_limit_mps( dbControlSpeed_ / 3.6 );
					writer_planning_pad_->Write( info_Planning_pad_ );
				}
			}
			else
			{
				AERROR << "the control type is invalid! the control type = " << (uint32_t)nControlType_;
			}
		}
        }

        return bRet;
}

bool netRms::analysisPacketResponse( uint8_t * pData )
{
	uint32_t nIndex = 0;
	bool bRet = true;
	uint8_t nIDCarRead[RMS_CAR_ID_LEN];

        memcpy(nIDCarRead, &pData[6], RMS_CAR_ID_LEN);

        for(nIndex = 0; nIndex < RMS_CAR_ID_LEN; nIndex++)
        {
                if(nIDCarRead[nIndex] != nIDCar_[nIndex])
                {
                        AERROR << "the id of act is Error!  recv car id[" << nIndex << "] = (" << (uint32_t)nIDCarRead[nIndex] << ")     active car id[" << nIndex << "] = (" << (uint32_t)nIDCar_[nIndex] << ")";
                        bRet = false;
                        break;
                }
        }

	return bRet;
}

void netRms::GetCarChassisInfo()
{
        std::lock_guard<std::mutex> lock( mutex_chassis_ );

        if ( IsModuleNormal(info_Chassis_, dbChassisLastTimeHeader_, dbChassisLastTimeRead_, 30) )
        {
                dfCarSpeed_ =  info_Chassis_.speed_mps() * 3.6f ;
                dfCarThrottle_ = info_Chassis_.throttle_percentage();
                dfCarBrake_ = info_Chassis_.brake_percentage();
                infoGear_ = info_Chassis_.gear_location();
        }

        nCarTotalMileage_ = info_Chassis_Detail_.ch().vcu_vehicleinfo4_98f005d0().vcu_vehcummileage();        
        dbCarBatterySoc_ = info_Chassis_Detail_.ch().vcu_vehicleinfo1_98f001d0().vcu_batterysoc() * 0.01 * dbCarBatteryTotalSoc_;
        nCarBatteryTemp_ = info_Chassis_Detail_.ch().vcu_vehicleinfo3_98f003d0().vcu_batterytemp();

	switch( info_Chassis_Detail_.ch().vcu_vehicleinfo1_98f001d0().vcu_modulestatus() )
        {
        case		apollo::canbus::Vcu_vehicleinfo1_98f001d0::VCU_MODULESTATUS_L2_FAULT: nCarStatus_ = 1;	break;
        case		apollo::canbus::Vcu_vehicleinfo1_98f001d0::VCU_MODULESTATUS_L3_FAULT: nCarStatus_ = 2;	break;
        default:	nCarStatus_ = 0;									break;
        }
}

void netRms::GetCarDriveModeInfo()
{
        std::lock_guard<std::mutex> lock( mutex_guardian_ );
        bool bManual = false;

        if ( IsModuleNormal( info_Guardian_, dbGuardianLastTimeHeader_, dbGuardianLastTimeRead_, 60 ) )
        {
                switch( info_Guardian_.command_from() )
                {
                case guardian::MONITOR:                 nDriveMode_ = DM_IDLE;            break;
                case guardian::AUTO_DRIVING:      nDriveMode_ = DM_AUTO;         break;
                case guardian::REMOTE_CTRL:       nDriveMode_ = DM_REMOTE;   break;
                case guardian::MANU_CTRL:            nDriveMode_ = DM_MANUAL;   break;
                case guardian::AEBS:                           nDriveMode_ = DM_AEBS;          break;
                case guardian::MISS_MESSAGE:     nDriveMode_ = DM_MISS;           break;
                default:
                        AERROR << "the message from command of guardian is invalid!";
                        assert(0);
                        break;
                }
        }

        if ( IsModuleNormal( info_Tmc_, dbTmcLastTimeHeader_, dbTmcLastTimeRead_, 75 ) )
        {
                if ( info_Tmc_.flag() )
                {
                        bManual = true;
                }
        }

        if ( IsModuleNormal( info_Manuctrl_, dbManualLastTimeHeader_, dbManualLastTimeRead_, 75 ) )
        {
                if ( info_Manuctrl_.flag() )
                {
                        bManual = true;
                }
        }

        if( bManual )
        {
                nDriveType_ = DT_MANUAL;
        }
        else
        {
                nDriveType_ = DT_AUTODRIVE;
        }
}

void netRms::GetCarLocationInfo()
{
        apollo::localization::msf::WGS84Corr temp_wgs;
        std::lock_guard<std::mutex> lock( mutex_location_ );

        if( IsModuleNormal( info_Location_, dbLocationLastTimeHeader_, dbLocationLastTimeRead_, 100) )
        {
                apollo::localization::msf::FrameTransform::UtmXYToLatlon(
                                info_Location_.pose().position().x(),
                                info_Location_.pose().position().y(),
                                49,
                                false,
                                &temp_wgs );
                dbCarLon_ = RadToDeg(temp_wgs.log);
                dbCarLat_ = RadToDeg(temp_wgs.lat);
                dbCarCoordX_ = info_Location_.pose().position().x();
                dbCarCoordY_ = info_Location_.pose().position().y();
                dbCarDir_ = RadToDeg( info_Location_.pose().heading() );
        }
        else
        {
                dbCarLon_ = 0.0;
                dbCarLat_ = 0.0;
		dbCarCoordX_ = 0.0;
		dbCarCoordY_ = 0.0;
        }
}

template <typename T>
bool netRms::IsModuleNormal(T & type, double & dbPreTimeHeader,  double & dbPreTimeRead, float dfInternalTime)
{
        bool bRet = false;
        double dbCurTime = Clock::NowInSeconds() * 1000.0;

        if( type.has_header() )
        {
                if( type.header().timestamp_sec() > 0.0 )
                {
                        if( (type.header().timestamp_sec() - dbPreTimeHeader) > 0.00001 )
                        {
                                bRet = true;
                                dbPreTimeHeader = type.header().timestamp_sec();
                                dbPreTimeRead = dbCurTime;
                        }
                        else
                        {
                                if( ( dbCurTime - dbPreTimeRead ) < dfInternalTime)
                                {
                                        bRet = true;
                                }
                        }
                }
        }

        return bRet;
}

template <typename T>
bool netRms::setPacketData(T nValue, uint8_t * pnData, uint32_t * pnDataLen)
{
        bool ret = true;
        uint32_t nLen = 0;

        if( ( pnData == NULL ) || ( pnDataLen == NULL ) )
        {
                AERROR << "set packet data param is invalid!";
                ret = false;
        }

        if( ret == true )
        {
                nLen = *pnDataLen;

                if(sizeof(nValue) > nLen)
                {
                        AERROR << "set packet data len is less than value!";
                        ret = false;
                }
        }

        if( ret == true )
        {
                memcpy(pnData, &nValue, sizeof(nValue));
                *pnDataLen = nLen - sizeof(nValue);
        }

        return ret;
}

template <typename T>
bool netRms::getPacketData(T * pnValue, uint8_t * pnData, uint32_t * pnDataLen)
{
        bool ret = true;
        uint32_t nLen = 0;

        if( (pnValue == NULL) || ( pnData == NULL ) || ( pnDataLen == NULL ) )
        {
                AERROR << "get packet data param is invalid!";
                ret = false;
        }

        if( ret == true )
        {
                nLen = *pnDataLen;

                if(sizeof(*pnValue) > nLen)
                {
                        AERROR << "get packet data len is less than value!";
                        ret = false;
                }
        }

        if( ret == true )
        {
                memcpy(pnValue, pnData, sizeof(*pnValue));
                *pnDataLen = nLen - sizeof(*pnValue);
        }

        return ret;
}

bool netRms::assignSint8( int8_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( int8_t ) ) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        *pnDataBuf = nValue;
        *pnRestBufSize += sizeof( int8_t );

        return true;
}

bool netRms::assignUint8( uint8_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( uint8_t ) ) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        *pnDataBuf = nValue;
        *pnRestBufSize += sizeof( uint8_t );

        return true;
}

bool netRms::assignSint16( int16_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( int16_t )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        nValue = htons( nValue );
        memcpy( pnDataBuf, &nValue, sizeof( int16_t ) );
        *pnRestBufSize += sizeof( int16_t );

        return true;
}

bool netRms::assignUint16( uint16_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( uint16_t )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        nValue = htons( nValue );
        memcpy( pnDataBuf, &nValue, sizeof( uint16_t ) );
        *pnRestBufSize += sizeof( uint16_t );

        return true;
}

bool netRms::assignSint32( int32_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( int32_t )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        nValue = htonl( nValue );
        memcpy( pnDataBuf, &nValue, sizeof( int32_t ) );
        *pnRestBufSize += sizeof( int32_t );

        return true;
}

bool netRms::assignUint32( uint32_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( uint32_t )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        nValue = htonl( nValue );
        memcpy( pnDataBuf, &nValue, sizeof( uint32_t ) );
        *pnRestBufSize += sizeof( uint32_t );

        return true;
}

bool netRms::assignUint64( uint64_t nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( uint64_t )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        memcpy( pnDataBuf, &nValue, sizeof(uint64_t) );
        *pnRestBufSize += sizeof( uint64_t );

        return true;
}

bool netRms::assignFloat( float nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( float )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        memcpy( pnDataBuf, &nValue, sizeof(float) );
        *pnRestBufSize += sizeof( float );

        return true;
}

bool netRms::assignDouble( double nValue, uint8_t * pnDataBuf, uint32_t * pnRestBufSize )
{
        if (pnDataBuf == NULL || pnRestBufSize == NULL)
        {
                AERROR << "Assign Value have Invalid pointer!";
                return false;
        }

        if( ( *pnRestBufSize + sizeof( double )) >= RMS_MAX_SEND_BUF_SIZE )
        {
                AERROR << "Assign Value rest size of buf is less than value size!";
                return false;
        }

        memcpy( pnDataBuf, &nValue, sizeof(double) );
        *pnRestBufSize += sizeof( double );

        return true;
}

inline void netRms::ConvertCoord(double * pdbCoordX, double * pdbCoordY, double dbLon, double dbLat)
{
        apollo::localization::msf::UTMCoor utm_xy;

        if( fabs( dbLon ) < 0.00001 || fabs( dbLat ) < 0.00001 )
        {
                *pdbCoordX = 0.0;
                *pdbCoordY = 0.0;
        }
        else
        {
                apollo::localization::msf::FrameTransform::LatlonToUtmXY(
                                        DegToRad( dbLon ),
                                        DegToRad( dbLat ),
                                        &utm_xy );
                *pdbCoordX = utm_xy.x;
                *pdbCoordY = utm_xy.y;
        }
}

inline double netRms::PointsDistance(double dbPoint1X, double dbPoint1Y, double dbPoint2X, double dbPoint2Y)
{
        double dbDistanceX = fabs( dbPoint2X - dbPoint1X );
        double dbDistanceY = fabs( dbPoint2Y - dbPoint1Y );
        return sqrt( dbDistanceX * dbDistanceX + dbDistanceY * dbDistanceY );
}

}  // namespace rms
}  // namespace apollo
