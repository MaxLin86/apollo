#include "modules/roadguide/rxmodule.h"
#include "modules/roadguide/transfer.h"
#include "modules/roadguide/proto/roadguide.pb.h"

namespace apollo{
namespace roadguide{

bool Rxmodule::CreateMonitorMsg()
{
  for (uint32_t i = 0; i < RXMODULE_BOARDS; i++) {
    for (uint32_t j = 0; j < RXBOARD_ANTS; j++) {
      if (rxboards_[i].ant[j].col >= RXMODULE_ANT_COLS) {
        AERROR << "Ant row num exceed max allowed value";
        return false;
      }
      if (rxboards_[i].ant[j].row >= RXMODULE_ANT_ROWS) {
        AERROR << "Ant col num exceed max allowed value";
        return false;
      }
    }
  }

  for (uint32_t i = 0; i < RXMODULE_ANT_ROWS; i++) {
    meas_row_[i] = roadguide_msg_->add_meas_row();
    meas_row_[i]->set_row_num(i);
    for (uint32_t j = 0; j < RXMODULE_ANT_COLS; j++) {
      meas_ant_[i][j] = meas_row_[i]->add_meas_ant();
    }
  }

  meas_row_merge_ = roadguide_msg_->add_meas_row();
  for (uint32_t ant = 0; ant < RXMODULE_ANT_ROWS * RXMODULE_ANT_COLS; ant++) {
    meas_ant_merge_[ant] = meas_row_merge_->add_meas_ant();
  }
  
  return true;
}

Rxmodule::Rxmodule(std::shared_ptr<RoadguideMsg>& roadguide_msg, 
    std::shared_ptr<Transfer>& transfer_ptr)
    : roadguide_msg_(roadguide_msg), transfer_ptr_(transfer_ptr)
{
  CreateMonitorMsg();
  memset(meas_mag_, 0, sizeof(meas_mag_));
  memset(meas_mag_tmp_, 0, sizeof(meas_mag_tmp_));
  memset(meas_mag_avg_, 0, sizeof(meas_mag_tmp_));
  for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
      meas_mag_idx_[row] = 0;
      meas_mag_buf_full_[row] = false;
      meas_mag_break_[row] = false;
      meas_mags_invalid_cnt_[row] = 0;
      meas_mag_len_[row] = 0;
  }
}

void Rxmodule::UpdateMeasMsg()
{
  char mag_bar[RXMODULE_BAR_LEN + 1];
  uint32_t bar_len = 0;
  uint32_t mag;

  if (pos_valid_) {
    roadguide_msg_->set_id(rcvd_id_);
    roadguide_msg_->set_pos_x(pos_x_);
    pos_valid_ = false;
  }

  for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
    for (uint32_t col = 0; col < RXMODULE_ANT_COLS; col++) {
      if (!meas_updated_[row][col]) {
        continue;
      }
      meas_updated_[row][col] = false;
      // Currently not thread safe
      mag = meas_mag_tmp_[row][col];
      // meas_ant_[row][col]->set_id(meas_id_[row][col]);
      // meas_ant_[row][col]->set_mag(mag);
      if (mag > max_mag_) {
        bar_len = RXMODULE_BAR_LEN;
      } else {
        bar_len = mag * RXMODULE_BAR_LEN / max_mag_;
      }
      memset(mag_bar, 0, sizeof(mag_bar));
      memset(mag_bar, '*', bar_len);
      meas_ant_[row][col]->set_mag_bar(mag_bar);
    }
  }

  for (uint32_t col = 0; col < RXMODULE_ANT_COLS; col++) {
    for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
      mag = meas_mag_avg_[row][col];
      if (mag > max_mag_) {
        bar_len = RXMODULE_BAR_LEN;
      } else {
        bar_len = mag * RXMODULE_BAR_LEN / max_mag_;
      }
      memset(mag_bar, 0, sizeof(mag_bar));
      memset(mag_bar, '*', bar_len);
      meas_ant_merge_[row + col * RXMODULE_ANT_ROWS]->set_mag(mag);
      meas_ant_merge_[row + col * RXMODULE_ANT_ROWS]->set_mag_bar(mag_bar);
    }
  }
}

bool Rxmodule::OnGetMeas(rxmodule_board_t& board, transfer_meas_t& meas)
{
  uint32_t row, col, mag, id;
  bool mag_valid = false;
  
  for (uint32_t i = 0; i < RXBOARD_ANTS; i++) {
    row = board.ant[i].row;
    col = board.ant[i].col;
    id = meas.ant[i].id;
    mag = meas.ant[i].mag;

    meas_id_[row][col] = id;
    meas_mag_tmp_[row][col] = mag * board.ant[i].cal_coe;
    meas_updated_[row][col] = true;
    
    if (mag > max_mag_tmp_) {
      max_mag_tmp_ = mag;
    }

    if (mag > 0) {
      mag_valid = true;
      // Test
      AERROR << "ID: " << id << ", row: " << row << ", col: " << col;
    }
  }
  return mag_valid;
}

void Rxmodule::CalAvgMag()
{
  uint64_t mag_avg;

  for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
    for (uint32_t col = 0; col < RXMODULE_ANT_COLS; col++) {
      mag_avg = 0;
      for (uint32_t idx = 0; idx < meas_mag_len_[row]; idx++) {
        mag_avg += meas_mag_[idx][row][col];
      }
      if (meas_mag_len_[row] > 0) {
        meas_mag_avg_[row][col] = mag_avg / meas_mag_len_[row];
      } else {
        meas_mag_avg_[row][col] = 0;
      }
    }
  }
}

void Rxmodule::CalPosition()
{
  uint32_t max_mag;
  uint32_t start_idx, end_idx;
  uint32_t mag_avg[RXMODULE_ANT_ROWS * RXMODULE_ANT_COLS];

  CalAvgMag();

  for (uint32_t col = 0; col < RXMODULE_ANT_COLS; col++) {
    for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
      mag_avg[col * RXMODULE_ANT_ROWS + row] = meas_mag_avg_[row][col];
    }
  }

  max_mag = 0;
  for (uint32_t idx = 0; idx < RXMODULE_ANT_ROWS * RXMODULE_ANT_COLS; idx++) {
    if (mag_avg[idx] > max_mag) {
      max_mag = mag_avg[idx];
      start_idx = idx;
      end_idx = idx;
    }
  }
      
  if (max_mag == 0) {
    return;
  }

  for (uint32_t loop = 0; loop < 2; loop++) {
    uint32_t mag_left, mag_right;
    mag_left = (start_idx == 0) ? 0 : mag_avg[start_idx - 1];
    mag_right = (end_idx == RXMODULE_ANT_ROWS * RXMODULE_ANT_COLS) ? 
      0 : mag_avg[end_idx + 1];

    if (!mag_left && !mag_right) {
      break;
    }
    if (mag_left > mag_right) {
      start_idx--;
    } else {
      end_idx++;
    }
  }

  pos_x_ = ant0_offset_ + ant_interval_ * (start_idx + end_idx) / 2.0;
  pos_valid_ = true;
}

void Rxmodule::UpdateMeasBuf(bool mag_valid, uint32_t row)
{
  if (mag_valid) {
    meas_mag_break_[row] = false;
    memcpy(&meas_mag_[meas_mag_idx_[row]][row], &meas_mag_tmp_[row], sizeof(meas_mag_[0][0]));
    meas_mag_idx_[row]++;
    if (meas_mag_idx_[row] == RXMODULE_MEAS_MAG_BUF_LEN) {
      meas_mag_idx_[row] = 0;
      meas_mag_buf_full_[row] = true;
    }
    if (meas_mag_buf_full_[row]) {
      meas_mag_len_[row] = RXMODULE_MEAS_MAG_BUF_LEN;
    } else {
      meas_mag_len_[row] = meas_mag_idx_[row];
    }
    meas_mags_invalid_cnt_[row] = 0;
  } else {
    if (meas_mag_idx_[row] > 0 || meas_mag_buf_full_[row]) {
      meas_mags_invalid_cnt_[row]++;
    } else {
      meas_mags_invalid_cnt_[row] = 0;
    }
    if (meas_mags_invalid_cnt_[row] == RXMODULE_MEAS_MAG_INVALID_THRESHOLD) {
      meas_mag_break_[row] = true;
      meas_mag_idx_[row] = 0;
      meas_mags_invalid_cnt_[row] = 0;
      meas_mag_buf_full_[row] = false;
    }
  }
}

bool Rxmodule::Time2CalPosition()
{
  bool all_rows_have_mag = true;
  bool anyone_row_is_full = false;
  bool anyone_row_is_break = false;
  bool time_2_cal_postion = false;
  
  for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
    if (meas_mag_len_[row] == 0) {
      all_rows_have_mag = false;
    }
    if (meas_mag_buf_full_[row]) {
      anyone_row_is_full = true;
    }
    if (meas_mag_break_[row]) {
      anyone_row_is_break = true;
    }
  }

  if ((anyone_row_is_full || anyone_row_is_break) && (all_rows_have_mag || true)) {
    // AERROR << "meas_mag_len_[0]: " << meas_mag_len_[0]
    //        << ", meas_mag_len_[1]: " << meas_mag_len_[1];
    time_2_cal_postion = true;
  }
  // AERROR << "all_rows_have_mag: " << all_rows_have_mag;

  return time_2_cal_postion;
}

void Rxmodule::Proc()
{
  transfer_meas_t meas;
  bool mags_valid;

  for (uint32_t i = 0; i < RXMODULE_BOARDS; i++) {
    memset(&meas, 0, sizeof(meas));
    if (!transfer_ptr_->FetchMeas(rxboards_[i].addr, meas)) {
      AERROR << "Board (" << static_cast<uint32_t>(rxboards_[i].addr) << ")Get mag failed";
    }
    // Test
    // AERROR << "Addr: " << static_cast<uint32_t>(rxboards_[i].addr)
    //        << ", Mag[0]: " << meas.ant[0].mag
    //        << ", Mag[1]: " << meas.ant[1].mag;
    OnGetMeas(rxboards_[i], meas);
    // usleep(10 * 1000);
  }
  max_mag_ = max_mag_tmp_;

  for (uint32_t row = 0; row < RXMODULE_ANT_ROWS; row++) {
    mags_valid = false;
    for (uint32_t col = 0; col < RXMODULE_ANT_COLS; col++) {
      if (meas_mag_tmp_[row][col] > 0) {
        mags_valid = true;
        rcvd_id_ = meas_id_[row][col];
        break;
      }
    }
    UpdateMeasBuf(mags_valid, row);
  }

  if (Time2CalPosition()) {
    CalPosition();
  }
}

}  // namespace roadguide
}  // namespace apollo
