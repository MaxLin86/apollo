/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/ 
 

#include <memory>
#include<iostream>

#include <typeinfo>
#include "gtest/gtest.h"

#include "cyber/init.h"
// #include "cyber/proto/unit_test.pb.h" 

#include <string>
#include <thread> 

#include "cyber/common/global_data.h"
#include "cyber/cyber.h"
#include "cyber/init.h" 

#include "benewake.h"


#include     <stdio.h>      /*标准输入输出定义*/  
#include     <stdlib.h>     /*标准函数库定义*/  
#include     <unistd.h>     /*Unix 标准函数定义*/  
#include     <sys/types.h>    
#include     <sys/stat.h>     
#include     <fcntl.h>      /*文件控制定义*/  
#include     <termios.h>    /*PPSIX 终端控制定义*/  
#include     <errno.h>      /*错误号定义*/
#include <array>

namespace apollo {
 
namespace lidar {

std::array<int,9> speed_arr= { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
std::array<int,9> name_arr= { 115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};
// uint32_t speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
// uint32_t name_arr[] =  { 115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};

/**
*@brief  Set Serial Port BitRate
*@param  lidar_fd_     Type : int The File Description of Serial Port
*@param  speed  Type : int  Serial Speed
*@return  void
*/
//修改波特率的代码如下：
void benewake::set_speed(int lidar_fd_, int speed){ 
    int   status;
    struct termios   Opt;
    tcgetattr(lidar_fd_, &Opt);
    for( uint32_t i = 0; i < speed_arr.size(); i++ )
    // for( i=0; i < (sizeof(speed_arr) / sizeof(uint32_t)); i++ )
    {
        if (speed == name_arr[i])
        {
            tcflush(lidar_fd_, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(lidar_fd_, TCSANOW, &Opt);
            if (status != 0)
            {
                perror("tcsetattr lidar_fd_");
                return;
            }
            tcflush(lidar_fd_,TCIOFLUSH);
        }
    }
}
/**
*@brief   Set Serial Port Databits, Stopbits and Parity.
*@param  lidar_fd_     Type:  int The File Description of Serial Port
*@param  databits Type:  int Databits 7 or 8
*@param  stopbits Type:  int Stopbits 1 or 2
*@param  parity  Type:  int  Parity Type: n,N,e,E,o,O,s,S
*/
int benewake::set_Parity(int lidar_fd_, int databits, int parity, int stopbits)
{
    struct termios options;
    if ( tcgetattr( lidar_fd_,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(-1);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits)               /*Set Datebits*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return(-1);
    }

    switch (parity) /*Set Parity*/
    {
    case 'n':
    case 'N':
          options.c_cflag &= ~PARENB;   /* Clear parity enable */
          options.c_iflag &= ~INPCK;     /* Enable parity checking */
          break;
    case 'o':
    case 'O':
          options.c_cflag |= (PARODD | PARENB); /* Odd Checking*/
          options.c_iflag |= INPCK;             /* Disnable parity checking */
          break;
    case 'e':
    case 'E':
          options.c_cflag |= PARENB;     /* Enable parity */
          options.c_cflag &= ~PARODD;   /* Even Checking*/
          options.c_iflag |= INPCK;       /* Disnable parity checking */
          break;
    case 'S':
    case 's':  /*as no parity*/
          options.c_cflag &= ~PARENB;
          options.c_cflag &= ~CSTOPB;
          break;
    default:
          fprintf(stderr,"Unsupported parity\n");
          return(-1);
    }

    switch (stopbits)               /*Set Stobits*/
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return(-1);
    }
    /* Set input parity option */
    if (parity != 'n')
      options.c_iflag |= INPCK;

    /*以下两句添加后发送方可以不加回车换行，但是read读取不完整*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);     /*Input*/
    options.c_oflag &= ~OPOST;  /*Output*/

    //屏蔽功能： NL(换行)->CR(回车)、CR->NL、忽略输入回车
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL);

    tcflush(lidar_fd_,TCIFLUSH);
    //未设置O_NONBLOCK或O_NDELAY
    options.c_cc[VTIME] = 150; /* Timeout in 15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(lidar_fd_,TCSANOW,&options) != 0)
    {
      perror("SetupSerial 3");
      return(-1);
    }
    return(0);
}

bool benewake::init(const std::string &dev_serial_name){
 
    lidar_fd_ = open(dev_serial_name.c_str(), O_RDWR|O_NOCTTY);
    if (lidar_fd_ > 0)
    {
        printf("Open %s Success!\n", dev_serial_name.c_str());
    }
    else
    {
        AERROR << "Can't Open "<< dev_serial_name << " lidar_fd_ = " << lidar_fd_;
        // printf("Can't Open %s\n", dev_serial_name.c_str());
        return false;
        exit(0);
    }

    set_speed(lidar_fd_, 115200);
    if(set_Parity(lidar_fd_, 8, 'N', 1) == -1)
    {
        close(lidar_fd_);
        // exit(-1);
        return false;
    }
  return  true;
}

bool benewake::data_parse(const std::array<int,BENEWAKE_DATA_LEN> &byte_msg,lidar::BenewakeLidar &benewake_msg){
/**************************北醒激光雷达数据报文说明*****************************************************************
Byte0 0x59,帧头,每一帧都相同
Byte1 0x59,帧头,每一帧都相同
Byte2 Dist_L 距离值低八位
Byte3 Dist_H 距离值高八位
Byte4 Strength_L 低八位
Byte5 Strength_H 高八位
Byte6 Temp_L 低八位
Byte7 Temp_H 高八位
Byte8 Checksum 为前 8 字节数据的累加和,取累加和的低 8 位

输出数据说明
Dist(Distance):代表 TF02-Pro 测量输出的距离值,默认单位为 cm,解析为十进制的值范围为
0-4500。实际使用过程中,当信号强度值 Strength≤60 时,Dist 的测量值被认为不可信,默认输出
4500。当信号强度大于 60 且实际距离在 45~60m 时,Dist 输出值为 4500。当信号强度大于 60 且实
际距离超过 60m 时,会有过周期数据出现,此时数据为 0 或其他异常值。
Strength:指信号强度,默认输出值会在 0-65535 之间。当测距档位一定时,测距越远,信号
强度越低;目标物反射率越低,信号强度越低。
Temp(Temperature):表征芯片内部温度值。摄氏度 = Temp / 8 - 256
*********************************************************************************************/
 
    if(byte_msg.at(0) != 0x59 || byte_msg.at(1) != 0x59){
        return false;//前两个字节固定都是0x59
    } 
    double distance = byte_msg.at(2) + (byte_msg.at(3) << 8) ;
    double strength = byte_msg.at(4) + (byte_msg.at(5) << 8);
    double temperature = (byte_msg.at(6) + (byte_msg.at(7) << 8))/8 - 256;
    // std::cout<<"distance="<<distance<<";strength="<<strength<<";temperature="<<temperature<<std::endl;
   
    if(strength <= 60){
        return false;
    } 

    benewake_msg.set_distance(distance);
    benewake_msg.set_strength(strength);
    benewake_msg.set_temperature(temperature);

    return true;
}
bool benewake::GetLiarData(lidar::BenewakeLidar &benewake_msg){
 

    unsigned char buf[512] = {0};
    int nread = 0;
    if((nread = read(lidar_fd_,buf,512)) <= 0)  
    {  
        AERROR << "read num=" << std::to_string(nread) ; 
        return false; 
    }  
    if(nread < BENEWAKE_DATA_LEN){
        AERROR << "read num=" << std::to_string(nread) ; 
        return false;
    }

    std::array<int,BENEWAKE_DATA_LEN> byte_msg;
    for(int i = 0; i< BENEWAKE_DATA_LEN ;i++){
        byte_msg.at(i) = buf[i]; 
    } 
    data_parse(byte_msg, benewake_msg); 

  return  true;
} 
 
}  // namespace cyber
}  // namespace apollo
