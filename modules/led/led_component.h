 

#pragma once

#include "cyber/common/macros.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/led/serial.h"

#include <memory>
#include <unordered_map>
#include <mutex>
/**
 * @namespace apollo::led 
 * @brief apollo::led
 */
namespace apollo {
namespace led {

class ledComponent : public apollo::cyber::TimerComponent {
public:
  bool Init() override;
  bool Proc() override;
  void OnLedmsg(const std::shared_ptr<ledmsg> &led_msg);

 private:
  void WordInit();
  std::unordered_map<uint8_t, std::vector<uint8_t>> word_list_;
  // std::unordered_map<ledmsg::word, int> word_list_;
  ledConf led_conf_;
  bool has_change_;
  std::string word_;
  std::mutex mutex_;
  // ledmsg::word wordid_;
  std::shared_ptr<serial> serial_ = nullptr;
  // std::shared_ptr<cyber::Writer<led::ledmsg>> ledmsg_read_ =nullptr;
};

CYBER_REGISTER_COMPONENT(ledComponent)

}  // namespace led
}  // namespace apollo
