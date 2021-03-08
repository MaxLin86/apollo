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

#include "modules/remoteManage/remoteManage.h"
#include "modules/common/util/message_util.h"
#include "modules/common/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace remoteManage {

using apollo::common::time::Clock;
using apollo::canbus::Chassis;

bool remoteManage::Init()
{
	ADEBUG << "this is remoteManage init function!\n" ;

	if (!GetProtoConfig(&actNetConfig)) 
	{
		AERROR << "Unable to load tmc conf file: " << ConfigFilePath();
		return false;
	}

	chassis_reader_ = node_->CreateReader<Chassis>(
	FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
		ADEBUG << "Received chassis data: run chassis callback.";
		std::lock_guard<std::mutex> lock(mutex_);
		chassis_info_.CopyFrom(*chassis);
	});

	InitActData();
	InitPacketPulse();
	InitPacketStatus();

	act_fdConnectSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (act_fdConnectSocket < 0) 
	{
		AERROR << "Socket creation failed";
		return false;
	}

	int32_t opt = 1;
	if (setsockopt(act_fdConnectSocket, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) 
	{ 
		AERROR << "Set socket failed";
		return false;
	}

	thread_socket_connect_.reset(new std::thread(&remoteManage::socketConnect, this));
	thread_socket_write_.reset(new std::thread(&remoteManage::socketWrite, this));
	thread_socket_read_.reset(new std::thread(&remoteManage::socketRead, this));

	return true;
}

bool remoteManage::Proc()
{
  ADEBUG << "this is remoteManage proc function!\n" ;

  AINFO << "vcu status = %d!\n" << chassis_info_.vcu_status();

  return true;
}

void remoteManage::StopSocket()
{
  close(act_fdConnectSocket);
  act_fdConnectSocket = -1;
}

void remoteManage::socketConnect()
{
	AERROR << "this is socketConnect function!\n" ;

	struct sockaddr_in client_addr;
	client_addr.sin_family  = AF_INET;
	client_addr.sin_addr.s_addr = inet_addr(actNetConfig.str_tmc_addr().c_str());
	client_addr.sin_port = htons(actNetConfig.str_tmc_port());

	if (act_fdConnectSocket < 0) 
	{
		AERROR << "Socket fd invalid";
		return;
	}

	while(bFlagQuitModule == false)
	{
		if(bFlagConnect == false)
		{
			if(connect(act_fdConnectSocket, (struct sockaddr*)&client_addr, sizeof(client_addr))  <  0)
			{
				AERROR << "connect socket failed, try per 1s to connect tmc ip %s!\r\n" << actNetConfig.str_tmc_addr().c_str();
				printf("connect socket failed, try per 1s to connect tmc ip %s!\r\n", actNetConfig.str_tmc_addr().c_str());
			}
			else
			{
				bFlagConnect = true;
				AERROR << "connect socket success!\r\n";
				printf("connect socket success!\r\n");
			}
		}

		usleep(1000000);
	}

	StopSocket();
	bFlagConnect = false;
}

void remoteManage::socketWrite()
{
	AERROR << "this is socketWrite function!\n" ;

	uint32_t nTimeCount = 0;

	packPulse();
	packStatus();

	while(bFlagQuitModule == false)
	{
		if(bFlagConnect == true)
		{
			if(send(act_fdConnectSocket, act_nPacketPulse, 106, 0) < 0)
			{
				AERROR << "send pulse packet error!\r\n" << errno; 
				bFlagConnect = false;
				continue;
			}

			if(nTimeCount++%100 == 0)
			{
				if(send(act_fdConnectSocket, act_nPacketStatus, 72, 0) < 0)
				{
					AERROR << "send status packet error!\r\n" << errno;
					bFlagConnect = false;
					continue;
				}
			}
		}
		else
		{
			usleep(1000000);
		}

		usleep(50000);
	}
}

void remoteManage::socketRead()
{
	AERROR << "this is socketRead function!\n" ;

	uint32_t nDataLen = 0;
	uint8_t nRecvBuf[1024];

	while(bFlagQuitModule == false)
	{
		if(bFlagConnect == true)
		{
			nDataLen =  recv (act_fdConnectSocket, nRecvBuf,  1024, MSG_WAITALL);

			AERROR << "Recv nDataLen = " << nDataLen;

			if(nDataLen > 0)
			{
				memcpy(&act_nRecvDataBuf[act_nRecvDataLen], nRecvBuf, nDataLen);
				act_nRecvDataLen += nDataLen;

				while( analysisPacket(act_nRecvDataBuf, &act_nRecvDataLen)  == true)
				{
					continue;
				}

				AERROR << "act_nRecvDataLen = " << act_nRecvDataLen;
			}
			else if(nDataLen < 0)
			{
				AERROR << "the connect is disable!\r\n";
				bFlagConnect = false;
				continue;
			}
			else
			{
				usleep(1000);
			}
			
		}
		else
		{
			usleep(1000000);
		}
	}
}

void remoteManage::InitActData()
{
  uint32_t nIndex = 0;

  bFlagQuitModule = false;
  bFlagConnect = false;

  act_fdConnectSocket = -1;
  memset(act_nPacketPulse, 0, sizeof(act_nPacketPulse));
  memset(act_nPacketAbnormal, 0, sizeof(act_nPacketAbnormal));
  memset(act_nPacketStatus, 0, sizeof(act_nPacketStatus));

  act_nRecvDataLen = 0;
  act_nNumCoord = 0;
  memset(act_nRecvDataBuf, 0, sizeof(act_nRecvDataBuf));
  memset(act_nID, 0, sizeof(act_nID));
  memset(act_dbCoord, 0, sizeof(act_dbCoord));

  for(nIndex = 0; nIndex < 64; nIndex++)
  {
    act_nID[nIndex] = 0x79;
  }
}

void remoteManage::InitPacketPulse()
{
  uint32_t nIndex = 0;

  act_nPacketPulse[0] = 0x10;
  act_nPacketPulse[1] = 0x10;
  act_nPacketPulse[2] = 0;
  act_nPacketPulse[3] = 0;
  act_nPacketPulse[4] = 0;
  act_nPacketPulse[5] = 100;
  act_nPacketPulse[6] = 0;
  act_nPacketPulse[7] = 0;
  act_nPacketPulse[8] = 0;
  act_nPacketPulse[9] = 0;
  act_nPacketPulse[10] = 0;
  act_nPacketPulse[11] = 0;
  act_nPacketPulse[12] = 0;
  act_nPacketPulse[13] = 0;

  for(nIndex = 0; nIndex < 64; nIndex++)
  {
    act_nPacketPulse[14+nIndex] = act_nID[nIndex];
  }

  for(nIndex = 0; nIndex < 24; nIndex++)
  {
    act_nPacketPulse[78+nIndex] = 0;
  }
  act_nPacketPulse[102] = 0;
  act_nPacketPulse[103] = 0;
  act_nPacketPulse[104] = 0;
  act_nPacketPulse[105] = 0;
}

void remoteManage::InitPacketStatus()
{
  uint32_t nIndex = 0;

  act_nPacketStatus[0] = 0x10;
  act_nPacketStatus[1] = 0x12;
  act_nPacketStatus[2] = 0;
  act_nPacketStatus[3] = 0;
  act_nPacketStatus[4] = 0;
  act_nPacketStatus[5] = 66;

  for(nIndex = 0; nIndex < 64; nIndex++)
  {
    act_nPacketStatus[6+nIndex] = nIndex + 1;
  }

  act_nPacketStatus[70] = 0;
  act_nPacketStatus[71] = 0;
}

void remoteManage::packPulse()
{
  double dbCoordX = 113.86356f;
  double dbCoordY = 22.49938f;
  double dbAngleDir = 300.789f;
  short nSpeed = 11;
  uint8_t nStatusCar = 1;
  uint8_t nStatusTask = 1;

  memcpy(&act_nPacketPulse[78],  &dbCoordX,  sizeof(double));
  memcpy(&act_nPacketPulse[86],  &dbCoordY,  sizeof(double));
  memcpy(&act_nPacketPulse[94],  &dbAngleDir,  sizeof(double));
  memcpy(&act_nPacketPulse[102],  &nSpeed,  sizeof(short));
  act_nPacketPulse[104] = nStatusCar;
  act_nPacketPulse[105] = nStatusTask;
}

void remoteManage::packStatus()
{
  
}

bool remoteManage::analysisPacket(uint8_t * pData, uint32_t * pnDataLen)
{
	AERROR << "this is analysisPacket function!" << *pnDataLen;
	uint32_t nDataLen = *pnDataLen;
	uint32_t nIndex = 0;
	uint32_t nIndexVersion = 0;
	uint8_t nVersion = 0;
	uint8_t nCmd = 0;
	uint32_t nLen = 0;
	bool bResult = true;
	uint8_t nExtractStatus = ES_NORMAL;

	do
	{
		bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nVersion);
		if(bResult == false)
		{
			*pnDataLen = 0;
			break;
		}
		nIndex++;
	}
	while(nVersion != 0x10);

	if(bResult == true)
	{
		nIndexVersion = nIndex - 1;
		bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nCmd);
		nIndex++;
	}
	else
	{
		*pnDataLen = nDataLen - nIndexVersion;
	}

	if(bResult == true)
	{
		bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nLen);
		nIndex+=4;
	}
	else
	{
		*pnDataLen = nDataLen - nIndexVersion;
	}

	if(bResult == true)
	{
		if(nDataLen < nLen)
		{
			*pnDataLen = nDataLen - nIndexVersion;	  
			bResult = false;
		}

		switch(nCmd)
		{
		case 0x20:
			nExtractStatus = extractPacketTask(&pData[nIndex], nDataLen - nIndex);
			break;

		case 0x21:
			nExtractStatus = extractPacketControl(&pData[nIndex], nDataLen - nIndex);
			break;

		default:
			bResult = false;
			*pnDataLen = nDataLen - nIndexVersion - 1;
			break;
		}
	}
	else
	{
		*pnDataLen = nDataLen - nIndexVersion;
	}

	switch(nExtractStatus)
	{
	case ES_NORMAL: 	 *pnDataLen = nDataLen - nIndex - nLen - 6; 	bResult = true;   break;
	case ES_ERROR:	   	    *pnDataLen = nDataLen - nIndexVersion - 1;	 bResult = false;  break;
	case ES_INCOMPLETE: *pnDataLen = nDataLen - nIndexVersion;	   bResult = false;  break;
	default:				  assert(0);
	}

	return bResult;
}

uint8_t remoteManage::extractPacketTask(uint8_t * pData, uint32_t nDataLen)
{
	bool bResult = true;
	uint8_t nExtractStatus = ES_NORMAL;
	uint32_t nIndex = 0;
	uint32_t nIndexID = 0;
	ulong nGkey = 0;
	uint8_t nActID[64] = {0};
	uint16_t nCoordNum = 0;
	double dbCoord[TMC_MAX_COORD_NUM * 2];

	bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nGkey);
	nIndex+=8;

	if(bResult == true)
	{
		bResult = extractArrayValue(&pData[nIndex], nDataLen - nIndex, nActID, 64);
		nIndex+=64;
	}

	if(bResult == true)
	{
		for(nIndexID = 0; nIndexID < 64; nIndexID++)
		{
			if(nActID[nIndexID] != act_nID[nIndexID])
			{
				bResult = false;
				nExtractStatus = ES_ERROR;
				AERROR << "the act's id received is error!\r\n";
				break;
			}
		}

		bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nCoordNum);
		nIndex+=2;
	}

	if(bResult == true)
	{
		printf("nCoordNum = %d,  nLen = %d\r\n", nCoordNum,  nDataLen - nIndex);

		if(nCoordNum > TMC_MAX_COORD_NUM)
		{
			bResult = false;
			nExtractStatus = ES_ERROR;
			AERROR << "the num of coordiate in task data is better than max_num!" << nCoordNum;
		}
		else
		{
			bResult = extractArrayValue(&pData[nIndex], nDataLen - nIndex, dbCoord, nCoordNum * 2);
			nIndex+=nCoordNum * sizeof(double) * 2;
		}
	}

	if(bResult == true)
	{
		act_nNumCoord = nCoordNum;
		memset(act_dbCoord, 0, sizeof(double) * TMC_MAX_COORD_NUM * 2);
		AERROR << "the task packet is recv success! the num of Coord is" << act_nNumCoord;

		for(nIndexID = 0; nIndexID < nCoordNum; nIndexID++)
		{
			act_dbCoord[nIndexID * 2] = dbCoord[nIndexID * 2];
			act_dbCoord[nIndexID * 2 + 1] = dbCoord[nIndexID * 2 + 1];
			printf("act_dbCoord[%d] = (%.5f, %.5f)!\r\n", nIndexID, act_dbCoord[nIndexID * 2], act_dbCoord[nIndexID * 2 + 1]);
		}
	}
	else
	{
		if(nExtractStatus == ES_NORMAL)
		{
			nExtractStatus = ES_INCOMPLETE;
		}
	}

	return nExtractStatus;
}

uint8_t remoteManage::extractPacketControl(uint8_t * pData, uint32_t nDataLen)
{
	bool bResult = true;
	uint8_t nExtractStatus = ES_NORMAL;
	uint32_t nIndex = 0;
	uint32_t nIndexID = 0;
	uint8_t nActID[64] = {0};
	uint8_t nTypeCtrl = 0;
	uint16_t nDistance = 0;

	bResult = extractArrayValue(&pData[nIndex], nDataLen - nIndex, nActID, 64);
	nIndex+=64;

	if(bResult == true)
	{
		for(nIndexID = 0; nIndexID < 64; nIndexID++)
		{
			if(nActID[nIndexID] != act_nID[nIndexID])
			{
				bResult = false;
				nExtractStatus = ES_ERROR;
				AERROR << "the act's id received is error!\r\n";
				break;
			}
		}

		bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nTypeCtrl);
		nIndex++;
	}

	if(bResult == true)
	{
		if(nTypeCtrl > 3)
		{
			bResult = false;
			nExtractStatus = ES_ERROR;
			AERROR << "the value of nTypeCtrl is error!\r\n";
		}

		bResult = extractValue(&pData[nIndex], nDataLen - nIndex, &nDistance);
		nIndex+=2;
	}

	if(bResult == true)
	{
		act_nControlType = nTypeCtrl;
		act_nDistance = nDistance;

		printf("the MicroControl Packet is recv success! (%d,  %d)\r\n", act_nControlType, act_nDistance);
	}
	else
	{
		if(nExtractStatus == ES_NORMAL)
		{
			nExtractStatus = ES_INCOMPLETE;
		}
	}

	return nExtractStatus;
}

template <typename T>
bool remoteManage::extractValue(uint8_t * pData, uint32_t nDataLen, T * pnValue)
{
	if((nDataLen < sizeof(T)) || (pData == NULL) || (pnValue == NULL))
	{
		return false;
	}

	memcpy(pnValue, pData, sizeof(T));
	return true;
}

template <typename T>
bool remoteManage::extractArrayValue(uint8_t * pData, uint32_t nDataLen, T * pnValue, uint32_t nArrayNum)
{
	if((nDataLen < sizeof(T) * nArrayNum) || (pData == NULL) || (pnValue == NULL))
	{
		return false;
	}

	memcpy(pnValue, pData, sizeof(T) * nArrayNum);
	return true;
}


}  // namespace remoteManage
}  // namespace apollo
