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

  if (!GetProtoConfig(&actNetConfig)) {
    AERROR << "Unable to load tmc conf file: " << ConfigFilePath();
    return false;
  }

  bFlagQuitModule = false;
  bFlagConnect = false;

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received chassis data: run chassis callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        chassis_info_.CopyFrom(*chassis);
      });

  InitActData();
  InitPacketPulse();
  InitPacketStatus();

  return true;
}

bool remoteManage::Proc()
{
  ADEBUG << "this is remoteManage proc function!\n" ;

  AINFO << "vcu status = %d!\n" << chassis_info_.vcu_status();

  return true;
}

bool remoteManage::InitSocket()
{
  act_fdConnectSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (act_fdConnectSocket == -1) {
      AERROR << "Socket creation failed";
      return false;
  }

  thread_socket_connect_.reset(new std::thread(&remoteManage::socketConnect, this));
  return true;
}

void remoteManage::StopSocket()
{
  close(act_fdConnectSocket);
  act_fdConnectSocket = -1;
}

void remoteManage::socketConnect()
{
  ADEBUG << "this is socketConnect function!\n" ;

  printf("socket connect run!\n");

  struct sockaddr_in client_addr;
  client_addr.sin_family  = AF_INET;
  client_addr.sin_addr.s_addr = inet_addr(actNetConfig.str_tmc_addr().c_str());
  client_addr.sin_port = htons(actNetConfig.str_tmc_port());

  if (act_fdConnectSocket < 0) 
  {
    AERROR << "Socket fd invalid";
    return;
  }

    while(!bFlagQuitModule)
    {
       while(!bFlagConnect)
       {
	       if(connect(act_fdConnectSocket, (struct sockaddr*)&client_addr, sizeof(client_addr)) < 0)
		{
			//AERROR << "connect socket failed";
			fflush(NULL);
			printf("connect socket failed, try per 1s to connect tmc ip %s!\r\n", actNetConfig.str_tmc_addr().c_str());
			usleep(1000000);
		}
		else
		{
			/* code */
			fflush(NULL);
			printf("connect socket success!\r\n");
			bFlagConnect = true;

			thread_socket_write_.reset(new std::thread(&remoteManage::socketWrite, this));
			thread_socket_read_.reset(new std::thread(&remoteManage::socketRead, this));
		}
       }

       printf("sleep connect thread!\r\n");
       usleep(10000000);
    }
}

void remoteManage::socketWrite()
{
  ADEBUG << "this is socketConnect function!\n" ;

  uint32_t nTimeCount = 0;

  packPulse();
  packStatus();

  while(1)
  {
    usleep(50000);
    send(act_fdConnectSocket, act_nPacketPulse, 106, 0);

    if(nTimeCount++%20 == 0)
    {
      send(act_fdConnectSocket, act_nPacketStatus, 72, 0);
      fflush(NULL);
      //printf("send nTimeCount = %d\n", nTimeCount);
    }
  }
}

void remoteManage::socketRead()
{
  ADEBUG << "this is socketConnect function!\n" ;

  uint32_t nDataLen = 0;
  uint8_t nRecvBuf[1024];

  while(1)
  {
    nDataLen =  recv (act_fdConnectSocket, nRecvBuf,  1024, MSG_WAITALL);
    fflush(NULL);
    printf("recv data len = %d!\n", nDataLen);

    if(nDataLen > 0)
    {
      memcpy(&act_nRecvDataBuf[act_nRecvDataLen], nRecvBuf, nDataLen);
      act_nRecvDataLen += nDataLen;

      while(act_nRecvDataLen > 78)
      {
          if( !analysisPacket(act_nRecvDataBuf, &act_nRecvDataLen) )
          {
            break;
          }
      }
    }
    else
    {
      fflush(NULL);
      printf("the connect is disable!\n");
      //bFlagConnect = false;
      return;
    }
  }
}

void remoteManage::InitActData()
{
  uint32_t nIndex = 0;

  act_fdConnectSocket = 0;
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
  double dbCoordX = 100.123f;
  double dbCoordY = 200.456f;
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
  uint32_t nDataLen = *pnDataLen;
  uint32_t nIndex = 0;
  uint32_t nIndexID = 0;
  uint32_t nIndexCoord = 0;
  uint32_t nLen = 0;
  uint64_t nGKey = 0;
  uint8_t   nActID[64];
  uint16_t nNumCoord = 0;

  while(pData[nIndex] != 0x10)
  {
    if(nIndex >= nDataLen)
    {
      *pnDataLen = 0;
      return false;
    }
    else
    {
      nIndex++;
    }
  }

  printf("nIndex = %d!\r\n", nIndex);

  if(nIndex + 78 >= nDataLen)
  {
    memcpy(pData, &pData[nIndex], nDataLen - nIndex);
    *pnDataLen = nDataLen - nIndex;
    printf("nIndex + 78 >= nDataLen = %d!\r\n", nDataLen);
    return false;
  }

  //nCmd = pData[nIndex + 1];
  memcpy(&nLen, &pData[nIndex + 2], 4);
  memcpy(&nGKey, &pData[nIndex +6], 8);
  memcpy(nActID, &pData[nIndex + 14], 64);

  for(nIndexID = 0; nIndexID < 64; nIndexID++)
  {
    if(nActID[nIndexID] != act_nID[nIndexID])
    {
      memcpy(pData, &pData[nIndex + 1], nDataLen - nIndex - 1);
      *pnDataLen = nDataLen - nIndex - 1;
      fflush(NULL);
      printf("the id of act is Error!  recvID[%d] = (%d)     actID[%d] = (%d)\r\n", nIndexID, nActID[nIndexID], nIndexID, act_nID[nIndexID]);
      return false;
    }
  }

  if((nLen + 6) > nDataLen)
  {
    memcpy(pData, &pData[nIndex], nDataLen - nIndex);
    *pnDataLen = nDataLen - nIndex;
    fflush(NULL);
    printf("the len is less of dataLen! nLen = %d     nDataLen = %d!\r\n", nLen, nDataLen);
    return false;
  }

  memcpy(&nNumCoord, &pData[nIndex + 78], sizeof(nNumCoord));
  fflush(NULL);
  printf("the packet is recv success! the num of Coord is %d\r\n", nNumCoord);

  for(nIndexCoord = 0; nIndexCoord < nNumCoord; nIndexCoord++)
  {
     memcpy(&act_dbCoord[nIndexCoord * 2], &pData[nIndex + 80 + nIndexCoord * 16], sizeof(double));
     memcpy(&act_dbCoord[nIndexCoord * 2 + 1], &pData[nIndex + 80 + nIndexCoord * 16 + 8], sizeof(double));

     printf("act_dbCoord[%d] = (%.6f, %.6f)!\r\n", nIndexCoord, act_dbCoord[nIndexCoord * 2], act_dbCoord[nIndexCoord * 2 + 1]);
  }

  memcpy(pData, &pData[nIndex + nLen + 6], nDataLen - nIndex - nLen - 6);
  *pnDataLen = nDataLen - nIndex - nLen - 6;

  return true;
}


}  // namespace remoteManage
}  // namespace apollo
