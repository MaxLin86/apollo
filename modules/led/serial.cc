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
 
// #include "cyber/proto/unit_test.pb.h" 

#include <string>
#include <thread> 
 


#include     <stdio.h>      /*标准输入输出定义*/  
#include     <stdlib.h>     /*标准函数库定义*/  
#include     <unistd.h>     /*Unix 标准函数定义*/  
#include     <sys/types.h>    
#include     <sys/stat.h>     
#include     <fcntl.h>      /*文件控制定义*/  
#include     <termios.h>    /*PPSIX 终端控制定义*/  
#include     <errno.h>      /*错误号定义*/
#include <array>
 
#include "serial.h"
#include "cyber/common/log.h" 

std::array<int,9> speed_arr= { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
std::array<int,9> name_arr= { 115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};
// uint32_t speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
// uint32_t name_arr[] =  { 115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};


//修改波特率的代码如下：
void serial::set_speed(int led_fd_, int speed){ 
    int   status;
    struct termios   Opt;
    tcgetattr(led_fd_, &Opt);
    for( uint32_t i = 0; i < speed_arr.size(); i++ )
    // for( i=0; i < (sizeof(speed_arr) / sizeof(uint32_t)); i++ )
    {
        if (speed == name_arr[i])
        {
            tcflush(led_fd_, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(led_fd_, TCSANOW, &Opt);
            if (status != 0)
            {
                perror("tcsetattr led_fd_");
                return;
            }
            tcflush(led_fd_,TCIOFLUSH);
        }
    }
}

int serial::set_Parity(int led_fd_, int databits, int parity, int stopbits)
{
    struct termios options;
    if ( tcgetattr( led_fd_,&options)  !=  0)
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

    tcflush(led_fd_,TCIFLUSH);
    //未设置O_NONBLOCK或O_NDELAY
    options.c_cc[VTIME] = 150; /* Timeout in 15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(led_fd_,TCSANOW,&options) != 0)
    {
      perror("SetupSerial 3");
      return(-1);
    }
    return(0);
}

bool serial::init(const std::string &dev_serial_name){
  char word_a = 'a', word_A = 'A', word_0 = '0';
  uint16_t gb2312_a = 97, gb2312_A = 65, gb2312_0 = 48;
//   uint16_t gb2312_a = 0xA3E1, gb2312_A = 0xA3C1, gb2312_0 = 0xA3B0;

  for (int i = 0; i < 26; i++) {
    word_gb2312_.emplace(word_a++, gb2312_a++);
    word_gb2312_.emplace(word_A++, gb2312_A++);
  }
  for (int i = 0; i <= 9;i++){
    word_gb2312_.emplace(word_0++, gb2312_0++);
  }
    //   AERROR<<"inti size="<<word_gb2312_.size();
  led_fd_ = open(dev_serial_name.c_str(), O_RDWR | O_NOCTTY);
  if (led_fd_ > 0) {
    printf("Open %s Success!\n", dev_serial_name.c_str());
  } else {
    AERROR << "Can't Open " << dev_serial_name << " led_fd_ = " << led_fd_;
    // printf("Can't Open %s\n", dev_serial_name.c_str());
    return false;
  }

    set_speed(led_fd_, 9600);
    if(set_Parity(led_fd_, 8, 'N', 1) == -1)
    {
        close(led_fd_);
        // exit(-1);
        return false;
    }
  return  true;
}

void serial::SetParam(uint8_t id ,LEDCmd &cmd){ 
     cmd.head = 0x7E;
     cmd.id = id;
    //  cmd.id = 0x03;
     cmd.color = 0x01;
    //  cmd.display = 0x01;
     cmd.display = 0x07;
     cmd.velocity = 0x02;
     cmd.tail = 0x7E;
}

bool serial::word2gb2312(const std::string& word,
                    std::vector<uint8_t> &gb2312_value){
  size_t len = word.length();


/////////////////////////////////////////
      AERROR<<"size="<<word_gb2312_.size();
//     std::map<char, uint16_t>::iterator iter;
    
//   for(iter=word_gb2312_.begin(); iter!=word_gb2312_.end(); iter++){
//       AERROR<<"data="<<std::to_string(iter->first);
//   }
/////////////////////////////////////////
  for (size_t i = 0; i < len;i++){
    // char a = word.at(0);
    // return a = '-';
    
    auto it = word_gb2312_.find(word.at(i));
    if(it == word_gb2312_.end()){
        AERROR<<"word.at(i)="<<std::to_string(i)<<word.at(i);
      return false;
    }
    // uint8_t h_value = it->second >> 8;
    uint8_t l_value = (it->second << 8) >> 8;
    // gb2312_value.emplace_back(h_value);
    gb2312_value.emplace_back(l_value);
  }
  return true;
}
// uint8_t serial::word2gb2312(const std::string &word){

//   auto it = word_gb2312_.find(word);
// }

bool serial::SetLEDCmd(LEDCmd cmd){
#if 1
    uint16_t sum = 0;
    auto pushBuf = [&](unsigned char buf[], uint8_t data, int &len) {
        if(data == 0x7E)
        {
            buf[len++] = 0x7d;
            buf[len++] = 0x01;
            sum += 0x7E;
        }
        else if (data == 0x7d)
        {
            buf[len++] = 0x7d;
            buf[len++] = 0x00;
            sum += 0x7d;
        }
        else
        {
            buf[len++] = data;
            sum += data;
        }
    };
    unsigned char buf[512] = {0};
    int len = 0;
    buf[len++] = cmd.head;
    pushBuf(buf, cmd.id, len);
    pushBuf(buf, cmd.color, len);
    pushBuf(buf, cmd.display, len);
    pushBuf(buf, cmd.velocity, len);
    // buf[len++] = cmd.id;
    // buf[len++] = cmd.color;
    // buf[len++] = cmd.display;
    // buf[len++] = cmd.velocity;
    // buf[len++] = cmd.check_code;
    std::vector<uint8_t> gb2312_value;
    if(!word2gb2312(cmd.word,gb2312_value)){
        // AERROR<<"!word2gb2312";
      return false;
    }
    for (const auto value : gb2312_value) {
      pushBuf(buf, value, len);
    }
    // for (const auto value : cmd.word) {
    //   pushBuf(buf, value, len);
    // }
    cmd.check_code=(sum << 8) >> 8;
    pushBuf(buf, cmd.check_code, len); 
    buf[len++] = cmd.tail;
#else
    unsigned char buf[26] = {0};
#define ZHONG
    int len = 0;
    buf[len++] = 0x7e;
    buf[len++] = 0x03;
    buf[len++] = 0x01;
    buf[len++] = 0x01;
    buf[len++] = 0x02;
#ifdef ZHONG
    buf[len++] = 0xD6;
    buf[len++] = 0xD0;
#endif
    buf[len++] = 0xBF;
    buf[len++] = 0xC6;
    buf[len++] = 0xD4;
    buf[len++] = 0xc6;
    buf[len++] = 0xc9;
    buf[len++] = 0xbc;
    buf[len++] = 0xd0;
    buf[len++] = 0xc5;
    buf[len++] = 0xcf;
    buf[len++] = 0xa2;
    buf[len++] = 0xbc;
    buf[len++] = 0xac;
    buf[len++] = 0xca;
    buf[len++] = 0xf5;

#ifdef ZHONG
    buf[len++] = 0x7d;
    buf[len++] = 0x01;
#else
    buf[len++] = 0xD8;
#endif
    buf[len++] = 0x7e;
#endif
    int nCount = write(led_fd_, buf, len);
    if(nCount<0){
        AERROR << "led write fail"  ;
        return false;
    }
    curr_id_ = cmd.id;
    return true;
}
 
bool serial::IsSuccess(uint8_t id){
    // char buf[512] = {0};
    unsigned char buf[512] = {0};
    int nread = 0;
    std::vector<uint8_t> fb_data;
    // do{
    //     nread = read(led_fd_,buf,512);
    //     if(nread>0){
    //         for (int i = 0; i < nread; i++)
    //         {
    //             fb_data.push_back(buf[i]);
    //         }
    //     }
    // } while (nread <= 0);
    // nread = read(led_fd_,buf,512);
    if ((nread = read(led_fd_, buf, 512)) <= 0)
    {  
        AERROR << "read num=" << std::to_string(nread) ; 
        return false; 
    }
    // if ((nread = read(led_fd_, buf, 512)) <= 0)
    // {  
    //     AERROR << "read num=" << std::to_string(nread) ; 
    //     return false; 
    // }
    int feedback = 5;//反馈报文长度
    if (nread < feedback){
        AERROR << "feedback num=" << std::to_string(nread) ;
        for (int i = 0; i < nread;i++){
            AERROR << "num=" << i << ";data=" << std::to_string(buf[i]);
        }
        return false;
    }
    if(buf[0] != 0x7E || buf[4] != 0x7E){
        AERROR << "error head:" << std::to_string(buf[0]) << ";tai;:"<<std::to_string(buf[4]);
        return false;
    }
    uint16_t temp = buf[1] + buf[2];
    uint8_t check_code = (temp << 8) >> 8;
    if (check_code != buf[3])
    {
        AERROR << "check code id:" << std::to_string(buf[3]);
        return false;
    }
    if (id != buf[1])
    {
        AERROR << "error id:" << std::to_string(buf[1]);
        return false;
    }
    if(buf[2] != 0x00){
        AERROR << "feedback fail";
        return false;
    }
    return true;
}
