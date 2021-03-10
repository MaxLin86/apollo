#pragma once
#include "cyber/cyber.h"
#include "modules/roadguide/transfer.h"
#include "modules/roadguide/rxboard_def.h"
#include "modules/roadguide/proto/roadguide.pb.h"

#define RXMODULE_BOARDS (14)
#define RXMODULE_ANT_ROWS (2)
#define RXMODULE_ANT_COLS (20)
#define RXMODULE_BAR_LEN (50)
#define RXMODULE_MEAS_MAG_BUF_LEN (10)
#define RXMODULE_MEAS_MAG_INVALID_THRESHOLD (3)

typedef struct {
  uint8_t  row;
  uint8_t  col;
  float    cal_coe;
} rxmodule_ant_t;

typedef struct {
  uint8_t addr;
  rxmodule_ant_t ant[RXBOARD_ANTS];
} rxmodule_board_t;

namespace apollo{
namespace roadguide{

class Rxmodule {
 public:
  Rxmodule(std::shared_ptr<RoadguideMsg>& roadguide_msg, std::shared_ptr<Transfer>& transfer_ptr);
  ~Rxmodule() {}
  void Proc();
  void UpdateMeasMsg();

 private:
  std::shared_ptr<RoadguideMsg> roadguide_msg_;
  std::shared_ptr<Transfer> transfer_ptr_;
  MeasRow* meas_row_[RXMODULE_ANT_ROWS];
  MeasAnt* meas_ant_[RXMODULE_ANT_ROWS][RXMODULE_ANT_COLS];
  MeasRow* meas_row_merge_;
  MeasAnt* meas_ant_merge_[RXMODULE_ANT_ROWS * RXMODULE_ANT_COLS];
  uint32_t meas_id_[RXMODULE_ANT_ROWS][RXMODULE_ANT_COLS];
  uint32_t meas_mag_[RXMODULE_MEAS_MAG_BUF_LEN][RXMODULE_ANT_ROWS][RXMODULE_ANT_COLS];
  uint32_t meas_mag_avg_[RXMODULE_ANT_ROWS][RXMODULE_ANT_COLS];
  uint32_t meas_mag_tmp_[RXMODULE_ANT_ROWS][RXMODULE_ANT_COLS];
  bool meas_updated_[RXMODULE_ANT_ROWS][RXMODULE_ANT_COLS];
  uint32_t meas_mag_idx_[RXMODULE_ANT_ROWS];
  uint32_t meas_mag_len_[RXMODULE_ANT_ROWS];
  bool meas_mag_buf_full_[RXMODULE_ANT_ROWS];
  bool meas_mag_break_[RXMODULE_ANT_ROWS];
  uint32_t meas_mags_invalid_cnt_[RXMODULE_ANT_ROWS];
  uint32_t max_mag_ = 1;
  uint32_t max_mag_tmp_ = 1;
  rxmodule_board_t rxboards_[RXMODULE_BOARDS] = {
    {10, {{0, 0, 0.781}, {0, 1, 0.671}}},
    { 3, {{0, 2, 0.809}, {0, 3, 1.028}}},
    { 7, {{0, 4, 0.939}, {0, 5, 1.211}}},
    { 5, {{0, 6, 0.819}, {0, 7, 0.986}}},
    {12, {{0, 8, 0.977}, {0, 9, 1.234}}},
    {13, {{0,10, 0.977}, {0,11, 1.234}}},
    { 2, {{0,12, 0.977}, {0,13, 1.234}}},

    { 9, {{1, 1, 0.972}, {1, 0, 0.773}}},
    { 8, {{1, 3, 1.024}, {1, 2, 1.325}}},
    {11, {{1, 5, 1.070}, {1, 4, 0.676}}},
    { 1, {{1, 7, 1.240}, {1, 6, 0.907}}}, // Abnormal
    { 6, {{1, 9, 1.104}, {1, 8, 1.181}}},
    {14, {{1,11, 1.104}, {1,10, 1.181}}},
    {15, {{1,13, 1.104}, {1,12, 1.181}}},
  };
  // int32_t ant_offset_[RXMODULE_ANT_ROWS] = {-520, -560}; // Millimeters
  int32_t ant0_offset_ = -560; // Millimeters
  int32_t ant_interval_ = 40; // Millimeters

  bool pos_valid_ = false;
  int32_t rcvd_id_;
  int32_t pos_x_;
  
  bool CreateMonitorMsg();
  bool OnGetMeas(rxmodule_board_t& board, transfer_meas_t& meas);
  void CalAvgMag();
  bool Time2CalPosition();
  void CalPosition();
  void UpdateMeasBuf(bool mag_valid, uint32_t row);
};

}  // namespace roadguide
}  // namespace apollo
