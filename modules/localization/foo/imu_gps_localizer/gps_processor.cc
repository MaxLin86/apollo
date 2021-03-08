#include "modules/localization/foo/imu_gps_localizer/gps_processor.h"

#include "modules/localization/foo/imu_gps_localizer/utils.h"

#include <fstream>
#include <iostream>
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {

void wgs84ToEcef(double lat, double lon, double h, double *x, double *y,double*z) {
		double a = 6378137;
		double b = 6356752.3142;
		double f = (a - b) / a;
		double e_sq = f * (2 - f);
        
		double lamb = lat/180.0*M_PI;
		double phi = lon/180.0*M_PI;
		double s = sin(lamb);
		double N = a / sqrt(1 - e_sq * s * s);

		double sin_lambda =sin(lamb);
		double cos_lambda = cos(lamb);
		double sin_phi = sin(phi);
		double cos_phi = cos(phi);

		*x = (h + N) * cos_lambda * cos_phi;
		*y = (h + N) * cos_lambda * sin_phi;
		*z = (h + (1 - e_sq) * N) * sin_lambda;

}
void ecefToEnu(double x, double y, double z, double lat, double lng, double height,
    double *xEast, double *yNorth, double *zUp) {
		double a = 6378137;
		double b = 6356752.3142;
		double f = (a - b) / a;
		double e_sq = f * (2 - f);
		double lamb = lat/180.0*M_PI;
		double phi = lng/180.0*M_PI;
		double s = sin(lamb);
		double N = a / sqrt(1 - e_sq * s * s);
		double sin_lambda = sin(lamb);
		double cos_lambda =cos(lamb);
		double sin_phi = sin(phi);
		double cos_phi = cos(phi);

		double x0 = (height + N) * cos_lambda * cos_phi;
		double y0 = (height + N) * cos_lambda * sin_phi;
		double z0 = (height + (1 - e_sq) * N) * sin_lambda;

		double xd = x - x0;
		double yd = y - y0;
		double zd = z - z0;

		double t = -cos_phi * xd - sin_phi * yd;

		*xEast = -sin_phi * xd + cos_phi * yd;
		*yNorth = t * sin_lambda + cos_lambda * zd;
		*zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
}


GpsProcessor::GpsProcessor(const Eigen::Vector3d& I_p_Gps) : I_p_Gps_(I_p_Gps) { }

bool GpsProcessor::UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state) {
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residual;
    ComputeJacobianAndResidual(init_lla, gps_data_ptr, *state, &H, &residual);
    const Eigen::Matrix3d& V = gps_data_ptr->cov;

    // EKF.
    const Eigen::MatrixXd& P = state->cov;
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;

    // Add delta_x to state.
    AddDeltaToState(delta_x, state);

    // Covarance.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
    return true;
}

void GpsProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                              const GpsPositionDataPtr gps_data, 
                                              const State& state,
                                              Eigen::Matrix<double, 3, 15>* jacobian,
                                              Eigen::Vector3d* residual) {
    const Eigen::Vector3d& G_p_I   = state.G_p_I;
    const Eigen::Matrix3d& G_R_I   = state.G_R_I;

    // Convert wgs84 to ENU frame.
    Eigen::Vector3d G_p_Gps;
    ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);

  //------------------------------------

  //measure->gnss_pos.longitude = G_p_Gps.x() * DEG_TO_RAD;
  //measure->gnss_pos.latitude = G_p_Gps.y() * DEG_TO_RAD;
 // measure->gnss_pos.height =
   //   bestgnsspos_msg.height_msl() + bestgnsspos_msg.undulation();
   
    msf::UTMCoor utm_xy;
    msf::FrameTransform::LatlonToUtmXY(gps_data->lla.y()/180.0*M_PI,
                                gps_data->lla.x()/180.0*M_PI, &utm_xy);


    G_p_Gps.x() = utm_xy. x;
    G_p_Gps.y() = utm_xy. y;
    G_p_Gps.z() = gps_data->lla.z();

    /**/
    std::cout << "-------------------------"<< std::endl;
      std::ofstream obs_gnss_out;
  obs_gnss_out.open("obs_gnss_out.txt",std::ios::app);
  obs_gnss_out <<std::fixed << state.timestamp << "\t" << G_p_Gps.x() << "\t" <<G_p_Gps.y() << "\t" << G_p_Gps.z() << std::endl;  
    

    // Compute residual.
    *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);

    // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(I_p_Gps_);
}

void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
    state->G_p_I     += delta_x.block<3, 1>(0, 0);
    state->G_v_I     += delta_x.block<3, 1>(3, 0);
    state->acc_bias  += delta_x.block<3, 1>(9, 0);
    state->gyro_bias += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
        state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
}

}  // namespace ImuGpsLocalization
}