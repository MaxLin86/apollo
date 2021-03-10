#include "modules/roadguide/rxboard.h"
#include "modules/roadguide/rxboard_def.h"

namespace apollo{
namespace roadguide{

Rxboard::Rxboard(std::shared_ptr<Transfer>& transfer_ptr, 
    const rxmodule_board_t& board)
    : board(board), transfer_ptr_(transfer_ptr)
{
  
}

bool Rxboard::FetchMeas(rxmodule_board_t& board)
{
  // transfer_meas_t meas;

  // memset(&meas, 0, sizeof(meas));

  // if (!transfer_ptr_->FetchMeas(board.addr, meas)) {
  //   AERROR << "Fetch meas failed";
  //   return false;
  // }

  return true;
}

}  // namespace roadguide
}  // namespace apollo
