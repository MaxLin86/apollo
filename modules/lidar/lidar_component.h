 

#pragma once

#include <memory>

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/lidar/benewake.h"

#include "modules/led/proto/led.pb.h"
/**
 * @namespace apollo::lidar 
 * @brief apollo::lidar
 */
namespace apollo {
namespace lidar {

class LidarComponent : public apollo::cyber::TimerComponent {
public:
  bool Init() override;
  bool Proc() override;

private: 
  LidarConf lidar_conf_;
  lidar::BenewakeLidar info_lidar_; 
  std::shared_ptr<benewake> benewake_ = nullptr;
  std::shared_ptr<cyber::Writer<lidar::BenewakeLidar>> BenewakeLidar_writer_ =nullptr;

  std::shared_ptr<cyber::Writer<led::ledmsg>> led_writer_ = nullptr;
  bool                  bWithInfo_ = false;
  int32_t            nWithInfoCount_ = 0;
};

CYBER_REGISTER_COMPONENT(LidarComponent)

}  // namespace lidar
}  // namespace apollo
