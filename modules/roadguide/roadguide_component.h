#include <thread>
#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "modules/roadguide/proto/roadguide.pb.h"
#include "modules/roadguide/proto/roadguide_conf.pb.h"
#include "modules/roadguide/transfer.h"
#include "modules/roadguide/rxmodule.h"

using apollo::cyber::Component;

namespace apollo{
namespace roadguide{

class RoadguideComponent : public apollo::cyber::TimerComponent {
 public:
  ~RoadguideComponent() {}
  bool Init() override;
  bool Proc() override;

 private:
  RoadguideConf roadguide_conf_;
  std::shared_ptr<apollo::cyber::Writer<RoadguideMsg>> roadguide_writer_;
  std::unique_ptr<std::thread> data_thread_ptr_;
  std::unique_ptr<Rxmodule> rxmodule_ptr_;
  std::shared_ptr<Transfer> transfer_ptr_;
  std::mutex roadguide_mutex_;
  std::shared_ptr<RoadguideMsg> roadguide_msg_;

  bool CreateTransfer(const RoadguideConf &conf);
  void PollingRadar();
  void StartPollingRadar();
  void PublishMsg();
};

CYBER_REGISTER_COMPONENT(RoadguideComponent)

}  // namespace roadguide
}  // namespace apollo
