
#include "modules/drivers/radar/uhnder_radar/objProcess.h"
#include <math.h>
#include <algorithm>


namespace apollo {
namespace drivers{
namespace uhnder_radar {

bool cmp(track a, track b) {
  if (fabs(a.lifetime - b.lifetime) < 0.001) {
    return (a.count > b.count);
  } else {
    return (a.lifetime > b.lifetime);
  }
}

void RadarTrackProcess::initRadarTrack(float pos_x, float pos_y,float angle){
    delta_time_ = 0.0;
    timestamp_  = 0.0;
    id_ = 0;
    install_x_ = pos_x;
    install_y_ = pos_y;
    install_ang_ = angle;
    MeasLists_.clear();
    TrackLists_.clear();
    H_matrix_.resize(3,6);
}

void RadarTrackProcess::radarTrackMainProcess(std::vector<track> &tracks) {
  trackCluster();
  trackMatch();
  trackUpdate();
  trackManage();
  trackAddNew();
  trackOutput(tracks);
  trackPredict();
}

void RadarTrackProcess::trackCluster() {
  for (auto it = MeasLists_.begin(); it != MeasLists_.end();) {
    float vcs_px = it->pos_x * cos(install_ang_) -
                   it->pos_y * sin(install_ang_) + install_x_;
    float vcs_py = it->pos_x * sin(install_ang_) +
                   it->pos_y * cos(install_ang_) + install_y_;
    if (vcs_px > 60.0 || fabs(vcs_py) > 6.0 ) {
      it = MeasLists_.erase(it);
    } else {
      ++it;
    }
  }

  int size = MeasLists_.size();
  float eps = 0.75;
  for (int i = 0; i < size; i++) {
    for (int j = i + 1; j < size; j++) {
      float dx = MeasLists_[i].pos_x - MeasLists_[j].pos_x;
      float dy = MeasLists_[i].pos_y - MeasLists_[j].pos_y;
      float dv = fabs(MeasLists_[i].speed - MeasLists_[j].speed);
      float dis = sqrt(dx * dx + dy * dy);
      float d_clu = 0.75 * dis + 0.25 * dv;
      if (d_clu < eps) {
        if (MeasLists_[i].snr > MeasLists_[j].snr) {
          MeasLists_[j].clutter = true;
        } else {
          MeasLists_[i].clutter = true;
        }
      }
    }
  }

  for (auto it = MeasLists_.begin(); it != MeasLists_.end();) {
    if (it->clutter == true) {
      it = MeasLists_.erase(it);
    } else {
      ++it;
    }
  }

}

void RadarTrackProcess::trackMatch() {
 // std::sort(TrackLists_.begin(),TrackLists_.end(),cmp);
  int track_size = TrackLists_.size();
  int meas_size = MeasLists_.size();
  for (int i = 0; i < track_size; i++) {
    float dx_match = 0.75;
    float dy_match = 0.5;
    if (TrackLists_[i].id > -1) {
      TrackLists_[i].match_status = false;
      TrackLists_[i].match_index = -1;
      for (int j = 0; j < meas_size; j++) {
        if (MeasLists_[j].match_status == false) {
          float dx = fabs(TrackLists_[i].pos_x - MeasLists_[j].pos_x);
          float dy = fabs(TrackLists_[i].pos_y - MeasLists_[j].pos_y);
          float dr = sqrt(dx * dx + dy * dy);
          float dv = fabs(TrackLists_[i].speed - MeasLists_[j].speed);
          if (dx < dx_match && dy < dy_match) {
            TrackLists_[i].match_status = true;
            dx_match = dx;
            dy_match = dy;
            MeasLists_[j].match_status = true;
            int idx = TrackLists_[i].match_index;
            if (idx > -1) {
              MeasLists_[idx].match_status = false;
            }
            TrackLists_[i].match_index = j;
            TrackLists_[i].match_error = dr;
            TrackLists_[i].match_dx = dx;
            TrackLists_[i].match_dy = dy;
            TrackLists_[i].match_dv = dv;
          }
        }
      }
    }
  }

  for (int i = 0; i < track_size; i++) {
    float dx_match = 0.75;
    float dy_match = 0.5;
    if (TrackLists_[i].id == -1) {
      TrackLists_[i].match_status = false;
      TrackLists_[i].match_index = -1;
      for (int j = 0; j < meas_size; j++) {
        if (MeasLists_[j].match_status == false) {
          float dx = fabs(TrackLists_[i].pos_x - MeasLists_[j].pos_x);
          float dy = fabs(TrackLists_[i].pos_y - MeasLists_[j].pos_y);
          float dr = sqrt(dx * dx + dy * dy);
          float dv = fabs(TrackLists_[i].speed - MeasLists_[j].speed);
          if (dx < dx_match && dy < dy_match) {
            TrackLists_[i].match_status = true;
            dx_match = dx;
            dy_match = dy;
            MeasLists_[j].match_status = true;
            int idx = TrackLists_[i].match_index;
            if (idx > -1) {
              MeasLists_[idx].match_status = false;
            }
            TrackLists_[i].match_index = j;
            TrackLists_[i].match_error = dr;
            TrackLists_[i].match_dx = dx;
            TrackLists_[i].match_dy = dy;
            TrackLists_[i].match_dv = dv;
          }
        }
      }
    }
  }


}

void RadarTrackProcess::trackUpdate() {
  int track_size = TrackLists_.size();

  for (int i = 0; i < track_size; i++) {
    Eigen::Matrix<double, 6, 1> X_matrix;
    Eigen::Matrix<double, 6, 6> P_matrix;
    Eigen::Matrix3d R_matrix;
    Eigen::Vector3d Z_matrix;
    Eigen::Vector3d tmp_matrix;
    Eigen::Matrix3d S_matrix;
    Eigen::Vector3d y_matrix;
    Eigen::MatrixXd K_matrix(6, 3);
    if (TrackLists_[i].match_status == true) {
      P_matrix = TrackLists_[i].P_matrix;
      X_matrix << TrackLists_[i].pos_x, TrackLists_[i].pos_y,
          TrackLists_[i].vel_x, TrackLists_[i].vel_y, TrackLists_[i].acc_x,
          TrackLists_[i].acc_y;
      calculateJacobianMatrix(X_matrix);
      R_matrix = TrackLists_[i].R_matrix;

      int idx = TrackLists_[i].match_index;

      Z_matrix << MeasLists_[idx].range, MeasLists_[idx].azimuth,
          MeasLists_[idx].speed;

      double rho = sqrt(X_matrix(0) * X_matrix(0) + X_matrix(1) * X_matrix(1));
      double theta = atan2(X_matrix(1), X_matrix(0));
      double rate =
          (X_matrix(0) * X_matrix(2) + X_matrix(1) * X_matrix(3)) / rho;

      tmp_matrix << rho, theta, rate;
      y_matrix = Z_matrix - tmp_matrix;

      S_matrix = H_matrix_ * P_matrix * H_matrix_.transpose() + R_matrix;

      K_matrix = P_matrix * H_matrix_.transpose() * S_matrix.inverse();

      X_matrix = X_matrix + K_matrix * y_matrix;
      P_matrix = P_matrix - K_matrix * H_matrix_ * P_matrix;

      TrackLists_[i].speed = MeasLists_[idx].speed;

      if (fabs(MeasLists_[idx].pos_z) > 0.0001) {
        TrackLists_[i].pos_z = MeasLists_[idx].pos_z;
      }
      TrackLists_[i].rcs = MeasLists_[idx].rcs;

      TrackLists_[i].P_matrix = P_matrix;
      TrackLists_[i].pos_x = X_matrix(0);
      TrackLists_[i].pos_y = X_matrix(1);
      TrackLists_[i].vel_x = X_matrix(2);
      TrackLists_[i].vel_y = X_matrix(3);
      TrackLists_[i].acc_x = X_matrix(4);
      TrackLists_[i].acc_y = X_matrix(5);
    }
  }
}

void RadarTrackProcess::calculateJacobianMatrix(Eigen::Matrix<double, 6, 1> x_matrix) {
  float px = x_matrix(0);
  float py = x_matrix(1);
  float vx = x_matrix(2);
  float vy = x_matrix(3);

  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  Eigen::MatrixXd Hj(3, 6);
  if (fabs(c1) < 0.001f) {
    H_matrix_ = Hj;
    return;
  }

  Hj << px / c2, py / c2, 0.0, 0.0, 0.0, 0.0,
       -py / c1, px / c1, 0.0, 0.0, 0.0, 0.0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2,
      py / c2, 0.0, 0.0;
  H_matrix_ = Hj;
}


void RadarTrackProcess::trackManage() {
  int lock_thre = 4;
  int lost_thre = 3;
  int lock_bit = 6;

  uint32_t countThre = (1 << 15) -1;
  
  int lockCount = 0;
  int lostCount = 0;

  int track_size = TrackLists_.size();
  for(int i = 0; i < track_size; i++){
    if(TrackLists_[i].match_status == true){
      TrackLists_[i].count = (TrackLists_[i].count << 1) + 1;
      TrackLists_[i].count = (TrackLists_[i].count & countThre);
    } else{
      TrackLists_[i].count = (TrackLists_[i].count << 1) + 0;
      TrackLists_[i].count = (TrackLists_[i].count & countThre);
    }
    
    TrackLists_[i].lifetime += delta_time_;
  
    lostCount = countframes(TrackLists_[i].count,lost_thre);
    lockCount = countframes(TrackLists_[i].count,lock_bit);

    if(TrackLists_[i].track_status == DETECTED){
      if(lostCount == 0){
        TrackLists_[i].track_status = LOST;
      }
      if(lockCount >= lock_thre){
        TrackLists_[i].track_status = LOCKED;
        TrackLists_[i].id = id_;
        id_++;
        if(id_ >= 65535){
          id_ = 0;
        }
      }
    } else if(TrackLists_[i].track_status == LOCKED){
      if(lostCount == 0){
        TrackLists_[i].track_status = LOST;
      }
    }
  }

  for (auto it = TrackLists_.begin(); it != TrackLists_.end();) {
    if (it->track_status == LOST) {
      it = TrackLists_.erase(it);
    } else {
      ++it;
    }
    
  }

  for (auto it = TrackLists_.begin(); it != TrackLists_.end();) {
    float vcs_px = it->pos_x * cos(install_ang_) -
                   it->pos_y * sin(install_ang_) + install_x_;
    float vcs_py = it->pos_x * sin(install_ang_) +
                   it->pos_y * cos(install_ang_) + install_y_;
    if (vcs_px > 60.0 || fabs(vcs_py) > 6.0) {
      it = TrackLists_.erase(it);
    } else {
      ++it;
    }
  }
  
  if(TrackLists_.size() > 1000){
     TrackLists_.clear();
 }
}

int RadarTrackProcess::countframes(uint32_t count, int frames){
   int step = 0;
   for(int i = 0; i < frames; i++){
     if((count & 1) == 1){
       step++;
     }
     count = count >> 1;
   }
   return step;
}

void RadarTrackProcess::trackAddNew() {
  Eigen::Matrix3d r;
  Eigen::Matrix<double, 6, 6> p;
  Eigen::Matrix<double, 6, 6> q;
  r << 0.25, 0.0, 0.0,
       0.0 , 0.0016, 0.0,
       0.0 , 0.0, 0.25;
  p << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
  q << 0.16, 0, 0, 0, 0, 0,
       0, 0.16, 0, 0, 0, 0,
       0, 0, 0.16, 0, 0, 0,
       0, 0, 0, 0.16, 0, 0,
       0, 0, 0, 0, 0.16, 0,
       0, 0, 0, 0, 0,  0.16;
  
  int meas_num = MeasLists_.size();
  for(int i = 0; i < meas_num; i++){
    if (MeasLists_[i].match_status == false) {
      track trackObj;

      trackObj.id = -1;
      trackObj.pos_x = MeasLists_[i].pos_x;
      trackObj.pos_y = MeasLists_[i].pos_y;
      trackObj.pos_z = MeasLists_[i].pos_z;
      trackObj.vel_x = MeasLists_[i].speed * cos(MeasLists_[i].azimuth);
      trackObj.vel_y = MeasLists_[i].speed * sin(MeasLists_[i].azimuth);
      trackObj.match_index = -1;
      trackObj.match_status = false;
      trackObj.count = 1;
      trackObj.track_status = DETECTED;
      trackObj.match_error = 0;
      trackObj.match_dx = 0;
      trackObj.match_dy = 0;
      trackObj.match_dv = 0;
      trackObj.speed = MeasLists_[i].speed;
      trackObj.P_matrix = p;
      trackObj.Q_matrix = q;
      trackObj.R_matrix = r;
      trackObj.lifetime = 0;
      trackObj.rcs = MeasLists_[i].rcs;
      TrackLists_.push_back(trackObj);
    }
  }
}
void RadarTrackProcess::trackOutput(std::vector<track> &tracks) {
  int track_size = TrackLists_.size();
  tracks.clear();
  for(int i = 0; i < track_size; i++){
    if(TrackLists_[i].track_status == LOCKED){
      tracks.push_back(TrackLists_[i]);
    }
  }
}

void RadarTrackProcess::trackPredict() {
  float delta_t = delta_time_;
  int track_size = TrackLists_.size();
  Eigen::Matrix<double, 6, 6> F_matrix;
  F_matrix << 1.0, 0.0, delta_t, 0.0, 0.5 * delta_t * delta_t, 0.0, 0.0, 1.0,
      0.0, delta_t, 0.0, 0.5 * delta_t * delta_t, 0.0, 0.0, 1.0, 0.0, delta_t,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0, delta_t, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0;

  for (int i = 0; i < track_size; i++) {
    Eigen::Matrix<double, 6, 1> X_matrix;
    Eigen::Matrix<double, 6, 6> P_matrix;
    Eigen::Matrix<double, 6, 6> Q_matrix;

    X_matrix << TrackLists_[i].pos_x, TrackLists_[i].pos_y,
        TrackLists_[i].vel_x, TrackLists_[i].vel_y, TrackLists_[i].acc_x,
        TrackLists_[i].acc_y;

    P_matrix = TrackLists_[i].P_matrix;
    Q_matrix = TrackLists_[i].Q_matrix;

    X_matrix = F_matrix * X_matrix;
    P_matrix = F_matrix * P_matrix * F_matrix.transpose() + Q_matrix;

    TrackLists_[i].P_matrix = P_matrix;
    TrackLists_[i].pos_x = X_matrix(0);
    TrackLists_[i].pos_y = X_matrix(1);
    TrackLists_[i].vel_x = X_matrix(2);
    TrackLists_[i].vel_y = X_matrix(3);
    TrackLists_[i].acc_x = X_matrix(4);
    TrackLists_[i].acc_y = X_matrix(5);
  }
}
}
}
}
