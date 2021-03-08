#include <glog/logging.h>
#include <memory>
#include <iostream>

#include <fstream>

#include <utility>

#include "modules/localization/foo/imu_gps_localizer/utils.h"
#include "modules/localization/foo/imu_gps_localizer/imu_gps_localizer.h"

#include "modules/localization/msf/common/util/frame_transform.h"
namespace apollo {
namespace localization {

State ImuGpsLocalizer::GetState() {
    return state_;
}

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps) 
    : initialized_(false){
    /*
    initializer_ = std::make_unique<Initializer>(I_p_Gps);    
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007));
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
    */
    initializer_ = std::unique_ptr<Initializer>(new Initializer(I_p_Gps));
        imu_processor_ = std::unique_ptr<ImuProcessor>(new ImuProcessor(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007)));
    gps_processor_ = std::unique_ptr<GpsProcessor>(new GpsProcessor(I_p_Gps));
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    std::ofstream iii;
     // std::ofstream imudata0;
  iii.open("imu_data.txt",std::ios::app);
  iii<< "last imu: " << state_.imu_data_ptr->timestamp << ", "<< state_.imu_data_ptr->acc.x() << ", "<< state_.imu_data_ptr->acc.y() << ", "<< state_.imu_data_ptr->acc.z() << std::endl;
    iii<< "cur imu: " << imu_data_ptr->timestamp << ", "<< imu_data_ptr->acc.x() << ", "<< imu_data_ptr->acc.y() << ", "<< imu_data_ptr->acc.z() << std::endl;
    iii.close();
    // Predict.
    imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

    // Convert ENU state to lla.
    ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));

    std::ofstream jjj;
  jjj.open("gps_data.txt",std::ios::app);
  jjj << "init lla: " << init_lla_ << std::endl;
  jjj << "ENU  : " << state_.G_p_I<< std::endl;
  jjj << "lla  : " << state_.lla<< std::endl;
        jjj.close();


    *fused_state = state_;
    return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr) {
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;
        
        initialized_ = true;

    
            msf::UTMCoor utm_xy;
  msf::FrameTransform::LatlonToUtmXY(init_lla_.y()/180.0*M_PI,
                                init_lla_.x()/180.0*M_PI, &utm_xy);
                                state_.G_p_I.x() = utm_xy.x;
                                state_.G_p_I.y() = utm_xy.y;
                                state_.G_p_I.z() = init_lla_.z();
    /**/
  
        //LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }
    // Update.
    gps_processor_->UpdateStateByGpsPosition(init_lla_, gps_data_ptr, &state_);

    return true;
}

}  // namespace ImuGpsLocalization
}