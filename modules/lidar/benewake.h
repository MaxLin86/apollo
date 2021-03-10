

#ifndef _LIDAR_LIDAR_BASE_H_
#define _LIDAR_LIDAR_BASE_H_

#include "modules/lidar/proto/lidar.pb.h"

#define BENEWAKE_DATA_LEN 9   //北醒报文长度
namespace apollo {
namespace lidar {

//北醒激光雷达TF02-Pro
class benewake{
public:
    bool init(const std::string &dev_serial_name);
    bool GetLiarData(lidar::BenewakeLidar &benewake_msg);
private:

    int lidar_fd_;
    void set_speed(int fd, int speed);
    int set_Parity(int fd, int databits, int parity, int stopbits);
    bool data_parse(const std::array<int,BENEWAKE_DATA_LEN> &byte_msg,lidar::BenewakeLidar &benewake_msg);
};
}
}
#endif