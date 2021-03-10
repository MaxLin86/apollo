

#ifndef _SERIAL_BASE_H_
#define _SERIAL_BASE_H_
 

#include <string>
#include<vector>
#include <map>
#include "modules/led/proto/led.pb.h"
#define serial_DATA_LEN 9    

struct LEDCmd
{
    uint8_t head;
    uint8_t id;
    uint8_t color;
    uint8_t display;
    uint8_t velocity;
    uint8_t check_code;
    uint8_t tail;
    std::string word;//英文字母或数字
    // std::vector<uint8_t> word;//每个文字转化为GB2312编码对应的16进制值
    /* data */
};

class serial{
public:
    bool init(const std::string &dev_serial_name);
    bool SetLEDCmd(LEDCmd cmd);
    bool IsSuccess(uint8_t id);
    void SetParam(uint8_t id, LEDCmd &cmd);
    // bool GetLiarData(lidar::serialLidar &serial_msg);
private:
    int led_fd_;
    uint8_t curr_id_;
    std::map<char, uint16_t> word_gb2312_;
    void set_speed(int fd, int speed);
    int set_Parity(int fd, int databits, int parity, int stopbits);
    // uint8_t word2gb2312(const std::string &word);
    bool word2gb2312(const std::string& word,
                     std::vector<uint8_t> &gb2312_value);
    // bool data_parse(const std::array<int,serial_DATA_LEN>
    // &byte_msg,lidar::serialLidar &serial_msg);
}; 
#endif