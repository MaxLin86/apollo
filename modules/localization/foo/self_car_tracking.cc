/******************************************************************************

 *****************************************************************************/


#include "modules/localization/foo/self_car_tracking.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "cyber/common/file.h"
#include <vector>
#include <list>
#include <memory>
#include <utility>
#include <algorithm>
#include "modules/common/time/time.h"

#define imu_px_offset 1.705//2.393
#define imu_py_offset 0.8715//0.835

// vehicle coor
#define ant_to_imu_px_offset -0.985//-0.15
#define ant_to_imu_py_offset 0.057//1.60

#define steering_percentage_offset 0.0//1.8


#define use_rear_wheel_center true

#define junction_to_vehicle 0.4
#define junction_to_trailer 12.0

#define enable_guidepost_kf true

namespace apollo {
namespace localization {

//using apollo::common::Status;
using apollo::common::SelfCarLocalizationEKF;
using apollo::common::GPSKF;
using apollo::common::TrailerKF;
using apollo::common::GuidepostEKF;

apollo::common::KalmanFilterInterface * SelfCarLocalizationTracking::CreateKalmanFilter(int kf_coor_sys){
  kalman_filter_ = new SelfCarLocalizationEKF(kf_coor_sys);
  return kalman_filter_;
}
/**/
int SelfCarLocalizationTracking::kf_coor_sys_ = 1;
bool SelfCarLocalizationTracking::InitKalmanFilter(LocalView &local_view, LocalizationEstimate &start_localization, LocalizationEstimate &init_localization) {
  // init state of ekf
  SelfCarLocalizationEKF *ekf = dynamic_cast<SelfCarLocalizationEKF *>(kalman_filter_);
  cv::Point2d init_pose;
  double init_theta = 0.0;
  init_pose.x = local_view.localization_estimate->pose().position().x();
  init_pose.y = local_view.localization_estimate->pose().position().y();

  init_theta = 0.0;

  if (init_localization.has_header() && start_localization.has_header()) {
    const double start_enux = start_localization.pose().position().x();
    const double start_enuy = start_localization.pose().position().y();
    const double cur_enux = init_localization.pose().position().x();
    const double cur_enuy =init_localization.pose().position().y();
   
    if (std::fabs(cur_enux - start_enux) > 1e-6) {
      init_theta = atan((cur_enuy-start_enuy)/(cur_enux-start_enux));
      if(cur_enux < start_enux) {
        if (cur_enuy >= start_enuy) {
          init_theta = init_theta + M_PI;
        } else {
          init_theta = init_theta - M_PI;
        }
      }        
    } else {
      if (std::fabs(cur_enuy - start_enuy) < 1e-6) {
        init_theta = 0.0;
      }else if (cur_enuy > start_enuy) {
        init_theta = M_PI_2;
      } else {
        init_theta = -M_PI_2;
      }
    }
   // init_theta = atan2((cur_enuy-start_enuy),(cur_enux-start_enux));
  } else {
    return false;
  } 

  ekf->Init(init_pose, init_theta);
  return true;
}

void SelfCarLocalizationTracking::KalmanFilterPredict(LocalView &local_view) {
  SelfCarLocalizationEKF *ekf = dynamic_cast<SelfCarLocalizationEKF *>(kalman_filter_);

  //if (local_view.chassis.has_header() && local_view.chassis.header().has_timestamp_sec()) {    
   // static double pre_timestamp = local_view.chassis->header().timestamp_sec();
    //const double cur_timestamp = local_view.chassis->header().timestamp_sec();
        static double pre_timestamp = local_view.chassis->header().timestamp_sec();
    const double cur_timestamp = local_view.chassis->header().timestamp_sec();
  //} 

  double dt = static_cast<double>(cur_timestamp - pre_timestamp); //
  const double speed_ms = static_cast<double>(local_view.chassis->speed_mps());//
  const double steering_percentage = static_cast<double>(local_view.chassis->steering_percentage());//
  double ds = dt * speed_ms;//
  common::VehicleParam vehicle_param_ =
  common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double steer_ratio = vehicle_param_.steer_ratio();
  const double steer_single_direction_max_degree =
  vehicle_param_.max_steer_angle() ;/// M_PI * 180;
  double steer_theta = steering_percentage / 100.0 *steer_single_direction_max_degree / steer_ratio;
  const double wheel_base = vehicle_param_.wheel_base(); //add 20200806

  double steer_radius = wheel_base / sin(steer_theta);//
  double dtheta = ds / steer_radius;//

  cv::Mat ctrl(4, 1, CV_64FC1);

  ctrl.at<double>(0, 0) = dt;//
  ctrl.at<double>(1, 0) = ds;//
  ctrl.at<double>(2, 0) = dtheta;//
  ctrl.at<double>(3, 0) = steer_radius;//

  const cv::Mat &predict = ekf->Predict(ctrl);
  pre_timestamp  = cur_timestamp;
  if (predict.at<double>(0,0)) {}//
}

void SelfCarLocalizationTracking::KalmanFilterCorrect(LocalView &local_view) {
  SelfCarLocalizationEKF *ekf = dynamic_cast<SelfCarLocalizationEKF *>(kalman_filter_);
  
  cv::Mat mesurement = cv::Mat(2, 1, CV_64FC1);
  mesurement.at<double>(0, 0) = static_cast<double>(local_view.localization_estimate->pose().position().x());
  mesurement.at<double>(1, 0) = static_cast<double>(local_view.localization_estimate->pose().position().y());

  // if offset distance is too larget, virtual guidepsot, skill correct
  #if 0
  if (offset_min > 3.0) {
    return;
  }
  #endif

  cv::Mat extra_measure = cv::Mat(2, 1, CV_64FC1);
  extra_measure.at<double>(0,0) = local_view.localization_estimate->pose().linear_velocity().x();
  extra_measure.at<double>(1,0) = local_view.localization_estimate->pose().linear_velocity().y();

  const double std_dev_px = local_view.localization_estimate->uncertainty().position_std_dev().x();
  const double std_dev_py = local_view.localization_estimate->uncertainty().position_std_dev().y();
  double speed = local_view.chassis->speed_mps();
  if (std::fabs(speed) < 0.005) {
    ekf->SetMeasureNoise(10*std_dev_px, 10*std_dev_py);
  } else {
    ekf->SetMeasureNoise(2*std_dev_px, 2*std_dev_py);
  }
  const cv::Mat &correct = ekf->Correct(mesurement, extra_measure);

  // AERROR << "correct:pre state: " << cur_px << ", " << cur_py << ", " << cur_theta << 
  //   "extra_measure: " << cur_guidepostGroupMark << ", " << extra_measure.at<double>(0,0) << ", " <<extra_measure.at<double>(1,0) <<
  //     "correct state: "<<correct.at<double>(0,0) << ", " << correct.at<double>(1,0) << ", " << correct.at<double>(2,0) << "\t";

  if (correct.at<double>(0,0)) {}
}

//----------------------------------------------------------------------------------
apollo::common::KalmanFilterInterface * GPSLocalizationFilter::CreateKalmanFilter(int kf_coor_sys){
  kalman_filter_ = new GPSKF(kf_coor_sys);
  return kalman_filter_;
}
/**/
int GPSLocalizationFilter::kf_coor_sys_ = 1;

bool GPSLocalizationFilter::InitKalmanFilter0(LocalView &local_view, cv::Point2d &start_utm_xy, cv::Point2d &init_utm_xy,
        double gnss_heading) {
  // init state of ekf
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);
  cv::Point2d init_pose;
  double init_theta = 0.0;
  init_pose.x = init_utm_xy.x;
  init_pose.y = init_utm_xy.y;

  double heading = gnss_heading/180.0*M_PI;
  if (gnss_heading <= 180.0) {
      heading = (-gnss_heading)/180.0*M_PI;
  } else if (gnss_heading > 180.0 && gnss_heading <= 360.0) {
    heading = (360.0-gnss_heading)/180.0*M_PI;
  }
  init_theta = common::math::NormalizeAngle(heading) ;

  ekf->Init(init_pose, init_theta);
  return true;
}

bool GPSLocalizationFilter::InitKalmanFilter(LocalView &local_view, cv::Point2d &start_utm_xy, cv::Point2d &init_utm_xy,
        double gnss_heading, double gnss_heading_std_dev) {
  // init state of ekf
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);
  cv::Point2d init_pose;
  double init_theta = 0.0;
  init_pose.x = init_utm_xy.x;
  init_pose.y = init_utm_xy.y;

  const double start_enux = start_utm_xy.x;
  const double start_enuy = start_utm_xy.y;
  const double cur_enux = init_utm_xy.x;
  const double cur_enuy = init_utm_xy.y;    

  if ((start_enux-cur_enux)*(start_enux-cur_enux) +
        (start_enuy-cur_enuy)*(start_enuy-cur_enuy) > 0.5 ) {  
   
    if (std::fabs(cur_enux - start_enux) > 1e-6) {
      init_theta = atan((cur_enuy-start_enuy)/(cur_enux-start_enux));
      if(cur_enux < start_enux) {
        if (cur_enuy >= start_enuy) {
          init_theta = init_theta + M_PI;
        } else {
          init_theta = init_theta - M_PI;
        }
      }        
    } else {
      if (std::fabs(cur_enuy - start_enuy) < 1e-6) {
        init_theta = 0.0;
      }else if (cur_enuy > start_enuy) {
        init_theta = M_PI_2;
      } else {
        init_theta = -M_PI_2;
      }
    }
   // init_theta = atan2((cur_enuy-start_enuy),(cur_enux-start_enux));

    double heading = gnss_heading/180.0*M_PI;
    if (gnss_heading <= 180.0) {
        heading = (-gnss_heading)/180.0*M_PI;
    } else if (gnss_heading > 180.0 && gnss_heading <= 360.0) {
      heading = (360.0-gnss_heading)/180.0*M_PI;
    }

    if (gnss_heading_std_dev < 5.0 && gnss_heading_std_dev > 1e-6) {
     init_theta = common::math::NormalizeAngle(heading);
    } else if (gnss_heading_std_dev < 10.0 && gnss_heading_std_dev >= 5.0) {
      init_theta =0.5*(init_theta + common::math::NormalizeAngle(heading)) ;
    }

  } else {
    return false;
  } 

  ekf->Init(init_pose, init_theta);
  return true;
}

void GPSLocalizationFilter::KalmanFilterPredict(LocalView &local_view, double gnss_heading_std_dev) {
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);

  //if (local_view.chassis.has_header() && local_view.chassis.header().has_timestamp_sec()) {    
   // static double pre_timestamp = local_view.chassis->header().timestamp_sec();
    //const double cur_timestamp = local_view.chassis->header().timestamp_sec();
        static double pre_timestamp = local_view.chassis->header().timestamp_sec();
    const double cur_timestamp = local_view.chassis->header().timestamp_sec();
  //} 

  double dt = static_cast<double>(cur_timestamp - pre_timestamp); //
  double speed_ms = static_cast<double>(local_view.chassis->speed_mps());//
  if (local_view.chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) {
    speed_ms = -1.0*speed_ms;
  }
  const double steering_percentage = static_cast<double>(local_view.chassis->steering_percentage());//
  double ds = dt * speed_ms;//
  common::VehicleParam vehicle_param_ =
  common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double steer_ratio = vehicle_param_.steer_ratio();
  const double steer_single_direction_max_degree =
  vehicle_param_.max_steer_angle() ;/// M_PI * 180;
  double steer_theta = (steering_percentage+2.0) / 100.0 *steer_single_direction_max_degree / steer_ratio;//addb 20201019
  const double wheel_base = vehicle_param_.wheel_base(); //add 20200806

  double steer_radius = wheel_base / tan(steer_theta);//
  double dtheta = ds / steer_radius;//

  // test,20200907
  /*
  if (std::fabs(steering_percentage) > 30.0) {
    dtheta *=2.4;
  }
  */
  //--------------------------
  //+++++++++++++++++++++++++++
  double heading_noise_std = gnss_heading_std_dev;
  if (gnss_heading_std_dev > 10.0) {
    heading_noise_std = 2.0*M_PI;
  }  else {
    if (std::fabs(gnss_heading_std_dev) < 1e-9) {
      heading_noise_std = 2.0*M_PI;
    }
  }

 // if (gnss_heading_std_dev < 1.0) {
  //   heading_noise_std = heading_noise_std / 10.0;
 // }

    ekf->SetMeasureNoise(0.03, 0.03, heading_noise_std);

  //++++++++++++++++++++++++++++


  cv::Mat ctrl(4, 1, CV_64FC1);

  ctrl.at<double>(0, 0) = dt;//
  ctrl.at<double>(1, 0) = ds;//
  ctrl.at<double>(2, 0) = dtheta;//
  ctrl.at<double>(3, 0) = steer_radius;//

  //ekf->state_post_.at<double>(2,0) = common::math::NormalizeAngle(
   // ekf->state_post_.at<double>(2,0));

  const cv::Mat &predict = ekf->Predict(ctrl);
  pre_timestamp  = cur_timestamp;
  if (predict.at<double>(0,0)) {
      //ekf->state_pre_.at<double>(2,0) = common::math::NormalizeAngle(
    //ekf->state_pre_.at<double>(2,0));
  }//
}

void GPSLocalizationFilter::KalmanFilterCorrect(LocalView &local_view, cv::Point2d utm_xy, cv::Point2d utm_xy_std_dev,
        double gnss_heading, double gnss_heading_std_dev, bool is_msf_heading) {
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);
  
  cv::Mat mesurement = cv::Mat(3, 1, CV_64FC1); //1===========================
  mesurement.at<double>(0, 0) = utm_xy.x;
  mesurement.at<double>(1, 0) = utm_xy.y;
  /*
  if (std::fabs(utm_xy.x-794767.514647) < 0.001 && std::fabs(utm_xy.y - 2490534.623796)<0.001	)
  {
    AERROR << "STATE POST:" << ekf->state_post_.at<double>(0,0) << "\t" << ekf->state_post_.at<double>(1,0) << "\t" <<
         ekf->state_post_.at<double>(2,0) << "\t";
        AERROR << "STATE PRE:" << ekf->state_pre_.at<double>(0,0) << "\t" << ekf->state_pre_.at<double>(1,0) << "\t" <<
         ekf->state_pre_.at<double>(2,0) << "\t";

  }
  */

  double heading = gnss_heading/180.0*M_PI;
  if (gnss_heading <= 180.0) {
      heading = (-gnss_heading)/180.0*M_PI;
  } else if (gnss_heading > 180.0 && gnss_heading <= 360.0) {
    heading = (360.0-gnss_heading)/180.0*M_PI;
  }
  /**/
  //mesurement.at<double>(2,0) = common::math::NormalizeAngle(heading);//TODO==============
/**/
  if (std::fabs(std::fabs(mesurement.at<double>(2,0))-M_PI) < 0.1 &&
    mesurement.at<double>(2,0) *ekf->state_pre_.at<double>(2,0) < 0.0) {      
      if (mesurement.at<double>(2,0) < 0.0) {
        mesurement.at<double>(2,0) += 2.0*M_PI;
      } else {
        mesurement.at<double>(2,0) -=2.0*M_PI; 
      }
      
  }
  
   mesurement.at<double>(2,0) = common::math::NormalizeAngle(heading);//TODO==============
   if (is_msf_heading) {
      mesurement.at<double>(2,0) = gnss_heading;
   }
  //std::ofstream measure;
  //measure.open("measure.txt",std::ios::app);
  //measure << std::fixed <<  gnss_heading << "\t" <<  heading << "\t" <<  mesurement.at<double>(2,0) << "\t" 
  //<< ekf->state_pre_.at<double>(2,0) << "\n";
 
  // if offset distance is too larget, virtual guidepsot, skill correct
  #if 0
  if (offset_min > 3.0) {
    return;
  }
  #endif

  cv::Mat extra_measure = cv::Mat(2, 1, CV_64FC1);
  const double speed = local_view.chassis->speed_mps();


  double heading_noise_std = gnss_heading_std_dev/180.0/5.0*M_PI;
  if (gnss_heading_std_dev > 10.0) {
    heading_noise_std = 2.0*M_PI;//\\\\\\\\\\\\\\\\\\\\\\//

         if (utm_xy_std_dev.x < 0.08 && utm_xy_std_dev.y<0.08) {
               utm_xy_std_dev.x *=0.8;
        utm_xy_std_dev.y *=0.8;
     } 

  } else {
    if (std::fabs(heading) <1e-9 && std::fabs(gnss_heading_std_dev) < 1e-9) {
      heading_noise_std = 2.0*M_PI;//\\\\\\\\\\\\\\\\\\\\\\//

           if (utm_xy_std_dev.x < 0.08 && utm_xy_std_dev.y<0.08) {
               utm_xy_std_dev.x *=0.8;
        utm_xy_std_dev.y *=0.8;
     } 

    } else {
      if (std::fabs(local_view.chassis->steering_percentage()) > 10.0 && std::fabs(speed) > 0.2) {
        //heading_noise_std = gnss_heading_std_dev/180.0*M_PI/std::fabs(local_view.chassis->steering_percentage()*10.0);
      }
      /*
      if (gnss_heading_std_dev < 10.0) {

      } else {
        heading_noise_std = 2.0*M_PI;
        utm_xy_std_dev.x *=0.8;
        utm_xy_std_dev.y *=0.8;
      }
      */
          if (utm_xy_std_dev.x < 0.08 && utm_xy_std_dev.y<0.08) {
        //       utm_xy_std_dev.x *=10.08;
       // utm_xy_std_dev.y *=10.08;
     } 

    }
  }
  

  //test
  static double pre_gnss_heading = gnss_heading;
  if (gnss_heading_std_dev > 1e6 && gnss_heading_std_dev < 1.0) {
    
  } else {
    if (std::fabs(gnss_heading - pre_gnss_heading)  > 10.0 && std::fabs(gnss_heading - pre_gnss_heading)  < 350.0) {
       heading_noise_std = 2.0*M_PI;//\\\\\\\\\\\\\\\\\\\\\\//
    }
  }
  pre_gnss_heading = gnss_heading;
  //-----------------------




  double pose_x_noise_x = utm_xy_std_dev.x;
  double pose_x_noise_y = utm_xy_std_dev.y;
  if (std::fabs(local_view.chassis->steering_percentage()) > 10.0 && std::fabs(speed) > 0.2) {
    if (pose_x_noise_x < 0.1 && pose_x_noise_y < 0.1) {
      //pose_x_noise_x *=0.1;//delete 20201019
      //pose_x_noise_y *=0.1;//delete 20201019
    }
  }

  //add 20201019
  if (pose_x_noise_x > 0.6 || pose_x_noise_y > 0.6){
    pose_x_noise_x *=12.50;
    pose_x_noise_y *=12.50;
  }
  //-------------------

  ekf->SetMeasureNoise(pose_x_noise_x, pose_x_noise_y, heading_noise_std);





    //ekf->state_pre_.at<double>(2,0) = common::math::NormalizeAngle(
    //ekf->state_pre_.at<double>(2,0));

  static double state_pre_heading = ekf->state_post_.at<double>(2,0);//++++++++++++++++++++

  const cv::Mat &correct = ekf->Correct(mesurement, extra_measure);
/*
    const double heading_theta = ekf->state_post_.at<double>(2,0);
    double rear_wheel_center_px =  mesurement.at<double>(0,0) - (imu_px_offset*cos(-heading_theta) + imu_py_offset*sin(-heading_theta));
    double rear_wheel_center_py =  mesurement.at<double>(1,0) -(- imu_px_offset*sin(-heading_theta) + imu_py_offset*cos(-heading_theta));
    if (utm_xy_std_dev.x < 0.1 && utm_xy_std_dev.y<0.1) {
        if (std::fabs(std::fabs(ekf->state_post_.at<double>(2,0)) - M_PI) < 0.09) {
          ekf->state_post_.at<double>(1,0) = (ekf->state_pre_.at<double>(1,0) +3.0*rear_wheel_center_py )/4.0;
         ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +3.0*rear_wheel_center_px )/4.0;
        }
    }
    */
  #if 1
  if (std::fabs(std::fabs(state_pre_heading) -std::fabs(ekf->state_post_.at<double>(2,0))) > 4.0/180.0*M_PI ) {

    ekf->state_post_.at<double>(2,0) = ekf->state_pre_.at<double>(2,0);
    //ekf->state_post_.at<double>(1,0) = ekf->state_pre_.at<double>(1,0);
    //ekf->state_post_.at<double>(0,0) = ekf->state_pre_.at<double>(0,0);
    ekf->error_cov_post_ = ekf->error_cov_pre_.clone();
    if (std::fabs(std::fabs(state_pre_heading) -std::fabs(ekf->state_post_.at<double>(2,0))) < 6.0/180.0*M_PI ) {
      ekf->state_post_.at<double>(2,0) = 0.5*(ekf->state_pre_.at<double>(2,0) +ekf->state_post_.at<double>(2,0)) ;
    }
    
  }

  if (std::fabs(gnss_heading_std_dev) > 1e-6 && std::fabs(gnss_heading_std_dev) < 1.0) {
    if (std::fabs(std::fabs(mesurement.at<double>(2,0))- std::fabs(ekf->state_post_.at<double>(2,0))) > 3.50/180*M_PI) {
       ekf->state_post_.at<double>(2,0) = mesurement.at<double>(2,0);
    }
  }/* else  if (std::fabs(gnss_heading_std_dev) > 1e-6 && std::fabs(gnss_heading_std_dev) < 5.0) {
    if (std::fabs(std::fabs(mesurement.at<double>(2,0))- std::fabs(ekf->state_post_.at<double>(2,0))) > 8.0/180*M_PI) {
       ekf->state_post_.at<double>(2,0) = 0.5*(mesurement.at<double>(2,0)+ekf->state_post_.at<double>(2,0));
    }
  }*/ //test

  if (!use_rear_wheel_center) {
    if (utm_xy_std_dev.x < 0.08 && utm_xy_std_dev.y<0.08) {
          ekf->state_post_.at<double>(1,0) =(ekf->state_pre_.at<double>(1,0) +3.0*mesurement.at<double>(1,0) )/4.0;
      ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +3.0*mesurement.at<double>(0,0) )/4.0;
    } else if(utm_xy_std_dev.x < 0.18 && utm_xy_std_dev.y<0.18) {
              ekf->state_post_.at<double>(1,0) =(ekf->state_pre_.at<double>(1,0) +1.0*mesurement.at<double>(1,0) )/2.0;
      ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +1.0*mesurement.at<double>(0,0) )/2.0;
    }
  } else {
    const double heading_theta = ekf->state_post_.at<double>(2,0);
    double rear_wheel_center_px =  mesurement.at<double>(0,0) - (imu_px_offset*cos(-heading_theta) + imu_py_offset*sin(-heading_theta));
    double rear_wheel_center_py =  mesurement.at<double>(1,0) -(- imu_px_offset*sin(-heading_theta) + imu_py_offset*cos(-heading_theta));
    
    double state_post_diff = std::fabs(ekf->state_post_.at<double>(0,0) - rear_wheel_center_px) + 
                    std::fabs(ekf->state_post_.at<double>(1,0) - rear_wheel_center_py);
    double state_pre_diff = std::fabs(ekf->state_pre_.at<double>(0,0) - rear_wheel_center_px) + 
                    std::fabs(ekf->state_pre_.at<double>(1,0) - rear_wheel_center_py);
    if (utm_xy_std_dev.x < 0.1 && utm_xy_std_dev.y<0.1) {
      if (state_post_diff > state_pre_diff && state_post_diff > 1.2) {
        ekf->state_post_.at<double>(1,0) = (ekf->state_pre_.at<double>(1,0) +3.0*rear_wheel_center_py )/4.0;
        ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +3.0*rear_wheel_center_px )/4.0;
       // if (std::fabs(std::fabs(ekf->state_post_.at<double>(2,0)) - M_PI) < 0.09) {
        //  ekf->state_post_.at<double>(1,0) = (ekf->state_pre_.at<double>(1,0) +6.0*rear_wheel_center_py )/7.0;
        // ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +6.0*rear_wheel_center_px )/7.0;
        //}
      }
    }/* else if(utm_xy_std_dev.x < 0.18 && utm_xy_std_dev.y<0.18) {
      if (state_post_diff > state_pre_diff) {
        ekf->state_post_.at<double>(1,0) =(ekf->state_pre_.at<double>(1,0) +1.0*rear_wheel_center_py )/2.0;
        ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +1.0*rear_wheel_center_px )/2.0;
      }
    }*/
  }
  

  state_pre_heading =  ekf->state_post_.at<double>(2,0);//++++++++++++++++++++
  #endif

  ekf->state_post_.at<double>(2,0) = common::math::NormalizeAngle(
    ekf->state_post_.at<double>(2,0));


  if (correct.at<double>(0,0)) {

  }
}
//----------------------------------------------------------------


/******************************************************
 * Trailer CYRA
 ******************************************************/
apollo::common::KalmanFilterInterface * TrailerLocalizationFilter::CreateKalmanFilter(int kf_coor_sys){
  kalman_filter_ = new TrailerKF(kf_coor_sys);
  return kalman_filter_;
}
int TrailerLocalizationFilter::kf_coor_sys_ = 1;

bool TrailerLocalizationFilter::InitKalmanFilter(cv::Point2d &init_vehicle_xy, 
          double init_vehicle_heading, double init_trailer_heading) {
    // init state of ekf
  TrailerKF *ekf = dynamic_cast<TrailerKF *>(kalman_filter_);

  double init_junction_px = cos(-init_vehicle_heading)*junction_to_vehicle + init_vehicle_xy.x;
  double init_junction_py = -sin(-init_vehicle_heading)*junction_to_vehicle + init_vehicle_xy.y;

  cv::Point2d init_trailer_pose;
  init_trailer_pose.x = cos(-init_trailer_heading)*(-junction_to_trailer) + init_junction_px;
  init_trailer_pose.y = -sin(-init_trailer_heading)*(-junction_to_trailer) + init_junction_py;

  double init_trailer_vel = 0.0;

  ekf->Init(init_trailer_pose, init_trailer_vel, init_trailer_heading);
  return true;
}

void TrailerLocalizationFilter::KalmanFilterPredict(LocalView &local_view) {
  TrailerKF *ekf = dynamic_cast<TrailerKF *>(kalman_filter_);

  //static double trailer_local_pre_timestamp = apollo::common::time::Clock::NowInSeconds();
  //double trailer_local_cur_timestamp = apollo::common::time::Clock::NowInSeconds();
  static double trailer_local_pre_timestamp = local_view.chassis->header().timestamp_sec();
  double trailer_local_cur_timestamp = local_view.chassis->header().timestamp_sec();
  double dt = trailer_local_cur_timestamp - trailer_local_pre_timestamp; //
  cv::Mat ctrl(1, 1, CV_64FC1);
  ctrl.at<double>(0, 0) = dt;
  const cv::Mat &predict = ekf->Predict(ctrl);
  trailer_local_pre_timestamp = trailer_local_cur_timestamp;
  if (predict.at<double>(0,0)){
  }
}

void TrailerLocalizationFilter::KalmanFilterCorrect(cv::Point2d vehicle_xy, cv::Point2d vehicle_xy_std_dev, 
        double vehicle_heading, double vehicle_heading_std_dev) {
  TrailerKF *ekf = dynamic_cast<TrailerKF *>(kalman_filter_);

  cv::Mat mesurement = cv::Mat(2, 1, CV_64FC1);
  mesurement.at<double>(0, 0) = vehicle_xy.x;
  mesurement.at<double>(1, 0) = vehicle_xy.y;

  cv::Mat extra_measure = cv::Mat(1, 1, CV_64FC1);
  extra_measure.at<double>(0,0) = vehicle_heading;

  //ekf->SetMeasureNoise(pose_x_noise_x, pose_x_noise_y, heading_noise_std);

  const cv::Mat &correct = ekf->Correct(mesurement, extra_measure);

  if (correct.at<double>(0,0)) {
    ekf->state_post_.at<double>(2,0) = common::math::NormalizeAngle(
      ekf->state_post_.at<double>(2,0));
  }
}


//MSF localization DR ekf 

int MSFLocalizationDRCorrectFilter::kf_coor_sys_ = 1;

apollo::common::KalmanFilterInterface * MSFLocalizationDRCorrectFilter::CreateKalmanFilter(int kf_coor_sys){
  kalman_filter_ = new GPSKF(kf_coor_sys);
  return kalman_filter_;
}

bool MSFLocalizationDRCorrectFilter::InitKalmanFilter(LocalView &local_view) {
  // init state of ekf
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);
  //TODO
  cv::Point2d init_utm_xy;
    init_utm_xy.x = local_view.localization_estimate->pose().position().x();
    init_utm_xy.y = local_view.localization_estimate->pose().position().y();
  double gnss_heading = local_view.localization_estimate->pose().heading();

  ekf->Init(init_utm_xy, gnss_heading);

  #if enable_guidepost_kf
  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  guidepost_ekf->Init(init_utm_xy, gnss_heading);
  #endif

  return true;
}

void MSFLocalizationDRCorrectFilter::KalmanFilterPredict(LocalView &local_view) {
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);

  
  static double pre_timestamp = local_view.chassis->header().timestamp_sec();
  const double cur_timestamp = local_view.chassis->header().timestamp_sec();  

  double dt = static_cast<double>(cur_timestamp - pre_timestamp); //
  double speed_ms = static_cast<double>(local_view.chassis->speed_mps());//
  if (local_view.chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) {
    speed_ms = -1.0*speed_ms;
  }
  const double steering_percentage = static_cast<double>(local_view.chassis->steering_percentage());//
  
  if (dt > 0.1) {
    dt =1.0/100.0;
  }
 /*
  static double pre_timestamp = loca_chassis.header().timestamp_sec();
  const double cur_timestamp = loca_chassis.header().timestamp_sec();  
    double dt = static_cast<double>(cur_timestamp - pre_timestamp); //
  double speed_ms = static_cast<double>(loca_chassis.speed_mps());//
  if (loca_chassis.gear_location() == canbus::Chassis::GEAR_REVERSE) {
    speed_ms = -1.0*speed_ms;
  }
  const double steering_percentage = static_cast<double>(loca_chassis.steering_percentage());//
  */
  //---------------------------------------------------

  double ds = dt * speed_ms;//
  common::VehicleParam vehicle_param_ =
  common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double steer_ratio = vehicle_param_.steer_ratio();
  const double steer_single_direction_max_degree =
  vehicle_param_.max_steer_angle() ;/// M_PI * 180;
  double steer_theta = (steering_percentage+steering_percentage_offset) / 100.0 *steer_single_direction_max_degree / steer_ratio;//addb 20201019
  const double wheel_base = vehicle_param_.wheel_base(); //add 20200806

  double steer_radius = wheel_base / tan(steer_theta);//
  double dtheta = ds / steer_radius;//

  cv::Mat ctrl(4, 1, CV_64FC1);
  ctrl.at<double>(0, 0) = dt;//
  ctrl.at<double>(1, 0) = ds;//
  ctrl.at<double>(2, 0) = dtheta;//
  ctrl.at<double>(3, 0) = steer_radius;//

  const cv::Mat &predict = ekf->Predict(ctrl);
  pre_timestamp  = cur_timestamp;
  if (predict.at<double>(0,0)) {
      //ekf->state_pre_.at<double>(2,0) = common::math::NormalizeAngle(
    //ekf->state_pre_.at<double>(2,0));
  }

  //add 20201118
  //ekf->state_post_ = ekf->state_pre_.clone();
  //ekf->error_cov_post_ = ekf->error_cov_pre_.clone();
  //-------------------

  #if enable_guidepost_kf
  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  guidepost_ekf->state_pre_ = ekf->state_pre_.clone();
  guidepost_ekf->state_post_ = ekf->state_post_.clone();
  guidepost_ekf->error_cov_pre_ = ekf->error_cov_pre_.clone();
  guidepost_ekf->error_cov_post_ = ekf->error_cov_post_.clone();
  #endif
  

   #if 0
      std::ofstream test_predict_data;
  test_predict_data.open("test_predict_data.txt",std::ios::app);  

  test_predict_data << std::fixed << cur_timestamp << "\t"  << dt << "\t"  << ds << "\t" << dtheta << "\t"
  << ekf->state_pre_.at<double>(0,0) << "\t" <<
  ekf->state_pre_.at<double>(1,0) << "\t" << ekf->state_pre_.at<double>(2,0) << std::endl;
  #endif

  if (std::fabs(local_view.chassis->header().timestamp_sec()-1607321842.01221) < 0.0001) {
    AERROR <<std::fixed<< "pre_post:" << ekf->state_post_.at<double>(0,0) << "\t" <<
    ekf->state_post_.at<double>(1,0) << "\t" <<
    ekf->state_post_.at<double>(2,0);

    AERROR <<std::fixed<< "cur_pre:" << ekf->state_pre_.at<double>(0,0) << "\t" <<
     ekf->state_pre_.at<double>(1,0) << "\t" <<
     ekf->state_pre_.at<double>(2,0);

  }

}

void MSFLocalizationDRCorrectFilter::KalmanFilterCorrect(LocalView &local_view, bool is_update_gps) {
  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);

  // stop, correct only best_gnss_pos is valid
  if (local_view_.chassis->speed_mps() < 1e-4) {
    if (local_view_.best_gnss_pos->latitude_std_dev() > 0.01 || local_view_.best_gnss_pos->longitude_std_dev() > 0.01) {
      return;
    }
  }
  
   if (std::fabs(local_view.chassis->header().timestamp_sec()-1607242712.73495) < 0.001) {
    AERROR <<std::fixed<< "cur_pre:" << ekf->state_pre_.at<double>(0,0) << "\t" <<
     ekf->state_pre_.at<double>(1,0) << "\t" <<
     ekf->state_pre_.at<double>(2,0);
   }
  cv::Mat mesurement = cv::Mat(3, 1, CV_64FC1); 
  mesurement.at<double>(0, 0) = local_view.localization_estimate->pose().position().x();
  mesurement.at<double>(1, 0) = local_view.localization_estimate->pose().position().y();
  mesurement.at<double>(2,0) = local_view.localization_estimate->pose().heading();
 
 /*
  if (std::fabs(std::fabs(mesurement.at<double>(2,0))-M_PI) < 0.1 &&
    mesurement.at<double>(2,0) *ekf->state_pre_.at<double>(2,0) < 0.0) {      
      if (mesurement.at<double>(2,0) < 0.0) {
        mesurement.at<double>(2,0) += 2.0*M_PI;
      } else {
        mesurement.at<double>(2,0) -=2.0*M_PI; 
      }
      
  }
  */
  // if offset distance is too larget, virtual guidepsot, skill correct
  #if 0
  if (offset_min > 3.0) {
    return;
  }
  #endif

  cv::Mat extra_measure = cv::Mat(2, 1, CV_64FC1);
  //const double speed = local_view.chassis->speed_mps();

  cv::Point2d utm_xy_std_dev;
  utm_xy_std_dev.x = local_view.localization_estimate->uncertainty().position_std_dev().x();
  utm_xy_std_dev.y = local_view.localization_estimate->uncertainty().position_std_dev().y();
  double gnss_heading_std_dev = 0.08;//0.02;
  double heading_noise_std = gnss_heading_std_dev; 

  heading_noise_std = 0.5*(local_view.localization_estimate->uncertainty().orientation_std_dev().x() + 
    local_view.localization_estimate->uncertainty().orientation_std_dev().y());

  double pose_x_noise_x = utm_xy_std_dev.x;
  double pose_x_noise_y = utm_xy_std_dev.y;

/*
  double parm = std::max(1.0,std::min(pose_x_noise_x/0.1,10.0));

if (pose_x_noise_x > 0.15) {
      pose_x_noise_x *=parm;
    }
    parm = std::max(1.0, std::min(pose_x_noise_y/0.1,10.0));
     if (pose_x_noise_y > 0.15) {
      pose_x_noise_y *=parm;//10.250; 
    }

    
    if (local_view.best_gnss_pos->latitude_std_dev() > 0.04 || local_view.best_gnss_pos->longitude_std_dev() > 0.04 ) {
      pose_x_noise_x *=2.0;
            pose_x_noise_y *=2.0;
      if (local_view.gnss_heading->heading_std_dev() > 5.0 || local_view.gnss_heading->heading_std_dev() < 1e-6) {
          pose_x_noise_x *=10.0;
            pose_x_noise_y *=10.0;
            heading_noise_std *= 10.0;
        }
    } else { // add 20201204
      if (local_view.gnss_heading->heading_std_dev() > 5.0 || local_view.gnss_heading->heading_std_dev() < 1e-6) {            
            heading_noise_std *= 3.0;
        } else if (local_view.gnss_heading->extended_solution_status() <=0 && local_view.gnss_heading->heading_std_dev() > 0.30) {
          //heading_noise_std *= 5.0;
        }
    }
*/
    

   if (local_view.localization_status->state_message()!=
        "Warning: Lidar Localization Is Abnormal") {
     double p2 = std::min(utm_xy_std_dev.x/0.02*3.0,30.0);
     pose_x_noise_x *=p2;//20.0;
     p2 = std::min(utm_xy_std_dev.y/0.02*3.0,30.0);
        pose_x_noise_y *=p2;//20.0;      
  }


  // gnss best pose is SINGLE or PSRDIFF to NARROW_INT, msf is unvalid (about 1.5s)
  static int invalid_cnt = 0;
  static int inval_to_val_cnt = 0;
  int sol_type = static_cast<int>(local_view.best_gnss_pos->sol_type());
  //if (sol_type ==16 ||
  //      sol_type ==17) {
  if (sol_type <47 ||
       sol_type >51 || utm_xy_std_dev.x > 0.2) {
         invalid_cnt++;
          inval_to_val_cnt = 0;
 }

 if ((sol_type == 50 || sol_type == 48) /*&& utm_xy_std_dev.x < 0.05*/) {
   if (invalid_cnt > 200) {     
     if ( utm_xy_std_dev.x < 0.05) {
     inval_to_val_cnt++;   
     }
     if (inval_to_val_cnt < 2.5*100/* || utm_xy_std_dev.x > 0.1*/) {
       pose_x_noise_x = 40.0;
       pose_x_noise_y = 40.0;

        const double best_pos_lat_std_dev = local_view_.best_gnss_pos->latitude_std_dev();
        const double best_pos_lon_std_dev = local_view_.best_gnss_pos->longitude_std_dev();
        //static int msf_invalid_cnt = 0;
        static bool msf_valid = false;
        if ((sol_type == 50 || sol_type == 48) && best_pos_lat_std_dev < 0.01 && best_pos_lon_std_dev < 0.01) {
        if (is_update_gps) {
            msf::UTMCoor utm_xy;
            msf::FrameTransform::LatlonToUtmXY(local_view_.best_gnss_pos->longitude()/180.0*M_PI,
                                          local_view_.best_gnss_pos->latitude()/180.0*M_PI, &utm_xy);
            const double heading_theta = ekf->state_pre_.at<double>(2,0);
            double imu_px = ant_to_imu_px_offset*cos(-heading_theta) + ant_to_imu_py_offset*sin(-heading_theta) + utm_xy.x;
            double imu_py = -ant_to_imu_px_offset*sin(-heading_theta) + ant_to_imu_py_offset*cos(-heading_theta) + utm_xy.y;
          if (std::fabs(imu_px -  mesurement.at<double>(0,0)) + std::fabs(imu_py -  mesurement.at<double>(1,0)) > 0.07) {
              mesurement.at<double>(0,0) = imu_px;
              mesurement.at<double>(1,0) = imu_py;
              //pose_x_noise_x = 0.005;
              //pose_x_noise_y = 0.005;
              msf_valid = false;
              AERROR<<"pppppppppppppppp "<<std::fixed << local_view_.chassis->header().timestamp_sec() << "\t" << imu_px << "\t" << imu_py << best_pos_lat_std_dev << "\t" << best_pos_lon_std_dev;
                
              double rear_wheel_center_px =  mesurement.at<double>(0,0) - (imu_px_offset*cos(-heading_theta) + imu_py_offset*sin(-heading_theta));
              double rear_wheel_center_py =  mesurement.at<double>(1,0) -(- imu_px_offset*sin(-heading_theta) + imu_py_offset*cos(-heading_theta));
              
              ekf->state_post_.at<double>(1,0) = (ekf->state_pre_.at<double>(1,0) +3.0*rear_wheel_center_py )/4.0;
              ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +3.0*rear_wheel_center_px )/4.0;
              return;
          } else {
            msf_valid = true;
          }
        }
        }

        if (!msf_valid && !is_update_gps) {
          pose_x_noise_x = 50.0;
          pose_x_noise_y = 50.0;
          AERROR << "TTTTTTTTTTTTT " << std::fixed << local_view_.chassis->header().timestamp_sec() << "\t" << mesurement.at<double>(0,0) << "\t" << mesurement.at<double>(1,0);
          //return;
        }

     } else {
       if (local_view_.best_gnss_pos->latitude_std_dev() <= 0.008 && local_view_.best_gnss_pos->longitude_std_dev() <= 0.008) {
       invalid_cnt = 0;
       }
     }
   }
 } else if (utm_xy_std_dev.x > 0.1) {
   if (sol_type == 50 || sol_type == 48) {
     pose_x_noise_x *= 50.0;
     pose_x_noise_y *= 50.0;
   } else {
       pose_x_noise_x *= 20.0;
       pose_x_noise_y *= 20.0;
   }
 }

if (pose_x_noise_x < 30.0) {
  //pose_x_noise_x = std::min(5.0,pose_x_noise_x);
  //pose_x_noise_y = std::min(5.0,pose_x_noise_y);
} else {
//pose_x_noise_x = std::min(100.0,pose_x_noise_x);
//pose_x_noise_y = std::min(100.0,pose_x_noise_y);
}
  ekf->SetMeasureNoise(pose_x_noise_x, pose_x_noise_y, heading_noise_std);

  const cv::Mat &correct = ekf->Correct(mesurement, extra_measure);

 

  #if 0

  if (std::fabs(gnss_heading_std_dev) > 1e-6 && std::fabs(gnss_heading_std_dev) < 1.0) {
    if (std::fabs(std::fabs(mesurement.at<double>(2,0))- std::fabs(ekf->state_post_.at<double>(2,0))) > 3.50/180*M_PI) {
       ekf->state_post_.at<double>(2,0) = mesurement.at<double>(2,0);
    }
  }

  if (!use_rear_wheel_center) {
    if (utm_xy_std_dev.x < 0.08 && utm_xy_std_dev.y<0.08) {
          ekf->state_post_.at<double>(1,0) =(ekf->state_pre_.at<double>(1,0) +3.0*mesurement.at<double>(1,0) )/4.0;
      ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +3.0*mesurement.at<double>(0,0) )/4.0;
    } else if(utm_xy_std_dev.x < 0.18 && utm_xy_std_dev.y<0.18) {
              ekf->state_post_.at<double>(1,0) =(ekf->state_pre_.at<double>(1,0) +1.0*mesurement.at<double>(1,0) )/2.0;
      ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +1.0*mesurement.at<double>(0,0) )/2.0;
    }
  } else {
    const double heading_theta = ekf->state_post_.at<double>(2,0);
    double rear_wheel_center_px =  mesurement.at<double>(0,0) - (imu_px_offset*cos(-heading_theta) + imu_py_offset*sin(-heading_theta));
    double rear_wheel_center_py =  mesurement.at<double>(1,0) -(- imu_px_offset*sin(-heading_theta) + imu_py_offset*cos(-heading_theta));
    
    double state_post_diff = std::fabs(ekf->state_post_.at<double>(0,0) - rear_wheel_center_px) + 
                    std::fabs(ekf->state_post_.at<double>(1,0) - rear_wheel_center_py);
    double state_pre_diff = std::fabs(ekf->state_pre_.at<double>(0,0) - rear_wheel_center_px) + 
                    std::fabs(ekf->state_pre_.at<double>(1,0) - rear_wheel_center_py);
    
    static double pre_state_x = ekf->state_post_.at<double>(0,0);
    static double pre_state_y = ekf->state_post_.at<double>(1,0);

    double state_diff = std::fabs(ekf->state_post_.at<double>(0,0)-pre_state_x) + 
                    std::fabs(ekf->state_post_.at<double>(1,0)-pre_state_y);
    if (state_diff > 0.2 && (utm_xy_std_dev.x > 0.5 || utm_xy_std_dev.y>0.5)) {
     // ekf->state_post_.at<double>(1,0) = (1*ekf->state_pre_.at<double>(1,0) +ekf->state_post_.at<double>(1,0) )/2.0;
      //ekf->state_post_.at<double>(0,0) = (1*ekf->state_pre_.at<double>(0,0) +ekf->state_post_.at<double>(0,0))/2.0;
    }
    
    if (utm_xy_std_dev.x < 0.1 && utm_xy_std_dev.y<0.1) {
      if (state_post_diff > state_pre_diff && state_post_diff > 1.2) {
        ekf->state_post_.at<double>(1,0) = (ekf->state_pre_.at<double>(1,0) +3.0*rear_wheel_center_py )/4.0;
        ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +3.0*rear_wheel_center_px )/4.0;
       // if (std::fabs(std::fabs(ekf->state_post_.at<double>(2,0)) - M_PI) < 0.09) {
        //  ekf->state_post_.at<double>(1,0) = (ekf->state_pre_.at<double>(1,0) +6.0*rear_wheel_center_py )/7.0;
        // ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +6.0*rear_wheel_center_px )/7.0;
        //}
      }
    }/* else if(utm_xy_std_dev.x < 0.18 && utm_xy_std_dev.y<0.18) {
      if (state_post_diff > state_pre_diff) {
        ekf->state_post_.at<double>(1,0) =(ekf->state_pre_.at<double>(1,0) +1.0*rear_wheel_center_py )/2.0;
        ekf->state_post_.at<double>(0,0) = (ekf->state_pre_.at<double>(0,0) +1.0*rear_wheel_center_px )/2.0;
      }
    }*/
    pre_state_x = ekf->state_post_.at<double>(0,0);
pre_state_y = ekf->state_post_.at<double>(1,0);
  }
  


  #endif



  if (correct.at<double>(0,0)) {
    ekf->state_post_.at<double>(2,0) = mesurement.at<double>(2,0);
   ekf->error_cov_pre_ = ekf->error_cov_post_.clone();
   ekf->state_pre_ = ekf->state_post_.clone();
   
  }
   //}
   //ekf->state_post_.at<double>(2,0) = mesurement.at<double>(2,0);//222222222222222
  ekf->state_post_.at<double>(2,0) = common::math::NormalizeAngle(
    ekf->state_post_.at<double>(2,0));

  #if enable_guidepost_kf
  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  guidepost_ekf->state_pre_ = ekf->state_pre_.clone();
  guidepost_ekf->state_post_ = ekf->state_post_.clone();
  guidepost_ekf->error_cov_pre_ = ekf->error_cov_pre_.clone();
  guidepost_ekf->error_cov_post_ = ekf->error_cov_post_.clone();
  #endif

  if (std::fabs(local_view.chassis->header().timestamp_sec()-1607321842.01221) < 0.0001) {
    AERROR <<std::fixed<< "cur_pre888:" << ekf->state_pre_.at<double>(0,0) << "\t" <<
     ekf->state_pre_.at<double>(1,0) << "\t" <<
     ekf->state_pre_.at<double>(2,0);

    AERROR <<std::fixed<< "cur_post888:" << ekf->state_post_.at<double>(0,0) << "\t" <<
    ekf->state_post_.at<double>(1,0) << "\t" <<
    ekf->state_post_.at<double>(2,0);


    AERROR <<std::fixed << "mearsure888: " << mesurement.at<double>(0, 0) << "\t" << mesurement.at<double>(1, 0) << "\t" 
    << mesurement.at<double>(2, 0);
    const double heading_theta = ekf->state_post_.at<double>(2,0);
    double rear_wheel_center_px =  mesurement.at<double>(0,0) - (imu_px_offset*cos(-heading_theta) + imu_py_offset*sin(-heading_theta));
    double rear_wheel_center_py =  mesurement.at<double>(1,0) -(- imu_px_offset*sin(-heading_theta) + imu_py_offset*cos(-heading_theta));
    AERROR <<std::fixed << "trans measure888: " << rear_wheel_center_px << "\t" << rear_wheel_center_py << "\t" << heading_theta;


  }

}


apollo::common::KalmanFilterInterface * MSFLocalizationDRCorrectFilter::CreateGuidepostKalmanFilter(int kf_coor_sys){
  guidepost_kalman_filter_ = new GuidepostEKF(kf_coor_sys);
  return guidepost_kalman_filter_;
}

void MSFLocalizationDRCorrectFilter::KalmanFilterGuidepostCorrect(LocalView &local_view,
         const std::vector<GuidepostGroup> &guideposts) {

  GuidepostEKF *guidepost_ekf = dynamic_cast<GuidepostEKF *>(guidepost_kalman_filter_);
  
  cv::Mat mesurement = cv::Mat(2, 1, CV_64FC1);
  mesurement.at<double>(0, 0) = local_view.routing->routing_request().waypoint().rbegin()->pose().x();
  mesurement.at<double>(1, 0) = local_view.routing->routing_request().waypoint().rbegin()->pose().y();

  // to mathced routing map
  //const double cur_px =  guidepost_ekf->state_pre_.at<double>(0,0);
  //const double cur_py =  guidepost_ekf->state_pre_.at<double>(1,0); // change  20201214

  const double cur_px =  guidepost_ekf->state_post_.at<double>(0,0);
  const double cur_py =  guidepost_ekf->state_post_.at<double>(1,0);

  const double cur_theta = local_view.localization_estimate->pose().heading();//guidepost_ekf->state_pre_.at<double>(2,0);
  double m_x_1 = mesurement.at<double>(0, 0)*(cos(-cur_theta)) + mesurement.at<double>(1, 0)*(sin(-cur_theta));
  double m_y_1 = mesurement.at<double>(0, 0)*(-sin(-cur_theta)) + mesurement.at<double>(1, 0)*(cos(-cur_theta));
  uint cur_guidepostGroupMark = 0;
  double offset_min = 99999.0;
  for(uint i = 0; i < guideposts.size() - 1; i++)
  {
    double offset = (guideposts[i].x - m_x_1 - cur_px) * (guideposts[i].x - m_x_1 - cur_px) + 
                (guideposts[i].y - m_y_1 - cur_py) * (guideposts[i].y - m_y_1 - cur_py);
    if (offset <= offset_min) {
      offset_min = offset;
      cur_guidepostGroupMark = i;
    }
  }
AERROR << "offset: " << offset_min;
AERROR << "mark: " << cur_guidepostGroupMark;
  // if offset distance is too larget, virtual guidepsot, skill correct
  #if 1
  if (offset_min >1.5 || cur_guidepostGroupMark >= 122) { //TODO/4444444444
     AERROR << "no matched the approprite guidepost mark!";    
     return;
  }
  #endif
  //guidepost_ekf->SetGuidepostIndex(cur_guidepostGroupMark);
  cur_perception_guidepost_index_ = cur_guidepostGroupMark;
  cv::Mat extra_measure = cv::Mat(2, 1, CV_64FC1);
  extra_measure.at<double>(0,0) = guideposts[cur_guidepostGroupMark].x;
  extra_measure.at<double>(1,0) = guideposts[cur_guidepostGroupMark].y;

  const cv::Mat &correct = guidepost_ekf->Correct(mesurement, extra_measure);

  // AERROR << "correct:pre state: " << cur_px << ", " << cur_py << ", " << cur_theta << 
  //   "extra_measure: " << cur_guidepostGroupMark << ", " << extra_measure.at<double>(0,0) << ", " <<extra_measure.at<double>(1,0) <<
  //     "correct state: "<<correct.at<double>(0,0) << ", " << correct.at<double>(1,0) << ", " << correct.at<double>(2,0) << "\t";

  if (correct.at<double>(0,0)) {
    guidepost_ekf->state_pre_ = guidepost_ekf->state_post_.clone();
    guidepost_ekf->error_cov_pre_ = guidepost_ekf->error_cov_post_.clone();
  } //20201214

  GPSKF *ekf = dynamic_cast<GPSKF *>(kalman_filter_);
  ekf->state_pre_ = guidepost_ekf->state_pre_.clone();
  ekf->state_post_ = guidepost_ekf->state_post_.clone();
  ekf->error_cov_pre_ = guidepost_ekf->error_cov_pre_.clone();
  ekf->error_cov_post_ = guidepost_ekf->error_cov_post_.clone();

  


  /*
  if (!StateToRect(cameras, correct, tf->rect_im_sta_, tf->rect_gnd_sta_)) {
    delete kalman_filter_;
    kalman_filter_ = NULL;
  }
  */
}

//----------------------------------------------------------------

//----------------------------------------------

}  // namespace planning
}  // namespace apollo
