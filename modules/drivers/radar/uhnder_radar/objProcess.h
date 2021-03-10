#pragma once

#include <memory>
#include <string>
#include "Eigen/Eigen"
#include <vector>

namespace apollo {
namespace drivers{
namespace uhnder_radar {

enum TRACKSTATUS{
    DETECTED = 0,
    LOCKED   = 1,
    LOST     = 2,
};

typedef struct {
    float  range;
    float  azimuth;
    float  speed;
    float  elevation;
    float  mag;
    float  snr;
    float  pos_x;
    float  pos_y;
    float  pos_z;
    float  rcs;
    bool   flag;
    bool   match_status;
    bool   clutter;
}measure;


typedef struct{
    int16_t   id;
    float   pos_x;
    float   pos_y;
    float   pos_z;
    float   vel_x;
    float   vel_y;
    float   acc_x;
    float   acc_y;
    float   vcs_x;
    float   vcs_y;
    float   match_error;
    float   match_dx;
    float   match_dy;
    float   match_dv;
    float   speed;
    float   rcs;
    double  lifetime;
    float   snr;
    uint32_t count;
    uint32_t lifecycle; 
    int16_t     match_index;
    bool    match_status;
    TRACKSTATUS track_status;
    Eigen::Matrix3d R_matrix;
    Eigen::Matrix<double, 6, 6> Q_matrix;
    Eigen::Matrix<double, 6, 6> P_matrix; 
}track;

class RadarTrackProcess {
 public:
  double timestamp_;
  double delta_time_;
  std::vector<measure> MeasLists_;
  std::vector<track> TrackLists_;
  float install_x_;
  float install_y_;
  float install_ang_;
  void radarTrackMainProcess(std::vector<track> &tracks);
  void initRadarTrack(float pos_x, float pos_y,float angle);
  bool Flag_2D_;
 private:

  Eigen::MatrixXd H_matrix_;
  int32_t id_;
  void trackCluster();
  void trackMatch();
  void trackUpdate();
  void trackManage();
  void trackAddNew();
  void trackPredict();
  void trackOutput(std::vector<track> &tracks);
  void calculateJacobianMatrix(Eigen::Matrix<double, 6, 1>  x_matrix);
  int  countframes(uint32_t count, int frames);

};

}
}
}
