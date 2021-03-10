#pragma once
#include "cyber/cyber.h"
#include "modules/roadguide/transfer.h"
#include "modules/roadguide/rxboard_def.h"

namespace apollo{
namespace roadguide{

class Rxboard {
 public:
  Rxboard(std::shared_ptr<Transfer>& transfer_ptr, const rxmodule_board_t& board);
  ~Rxboard() = default;

  rxmodule_board_t board;
  bool FetchMeas(rxmodule_board_t& board);

 private:
  std::shared_ptr<Transfer> transfer_ptr_;
  std::unique_ptr<Rxboard> rxboard_ptr_;
};

}  // namespace roadguide
}  // namespace apollo
