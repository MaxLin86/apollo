// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RDC_THRESHCTRL_H
#define SRS_HDR_RDC_THRESHCTRL_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

extern const CHAR* threshold_preset_descriptions[];

//
//! Enumeration of threshold presets
//
enum ThresholdPresetEnum
{
    LOW_SENSITIVITY,            //!< indoors
    HIGH_SENSITIVITY,           //!< driving

    NUM_THRESHOLD_PRESETS
};

//
//! Descriptor containing thresholds to be used when scanning
//
struct RDC_ThresholdControl /* UHDP_TYPE_THRESHOLD */
{
    //! the threshold at which a target signal is determined to be a detection
    FLOAT detection_thresh_snr_dB;

    //! the threshold at which a target signal is displayed in the clutter image,
    //! must be less than or equal to detection_thresh_snr_dB
    FLOAT clutter_image_thresh_snr_dB;

    //! the threshold at which a target signal is selected for output in the
    //! point cloud, must be less than or equal to detection_thresh_snr_dB
    FLOAT point_cloud_thresh_snr_dB;

    //! the distance in X meters from the radar at which the detection threshold
    //! scaling begins. Beyond this range, the detetection threshold is flat. Typical value is 25m
    FLOAT scale_det_thresh_max_range;

    //! the maximum adjustment in detection threshold (in SNR dB) directly in
    //! front of the radar. This detection threshold adjustment starts as
    //! scale_det_thresh_snr_adj dB at the radar and reduces linearly to 0 at
    //! scale_det_thresh_max_range meters. Typical value is 9 dB
    FLOAT scale_det_thresh_snr_adj;

    //! The threshold in m/s at which a detection is determined to be moving, no
    //! longer stationary, when the radar itself is not moving and target
    //! velocity has optimal accuracy. If 0.0, the detection is considered
    //! stationary if it was captured within the RDC3 static slice. Stationary
    //! detections are marked with flag RDC_DET_FLAG_STATIC
    FLOAT ego_zero_stationary_threshold_mps;

    //! The threshold in m/s at which a detection is determined to be moving, no
    //! longer stationary, when the radar itself is moving and thus ego velocity
    //! estimation errors can contribute to errors in target velocity
    //! estimation. if 0.0, the detection is considered stationary if it was
    //! captured within the RDC3 static slice. Stationary detections are marked
    //! with flag RDC_DET_FLAG_STATIC
    FLOAT ego_nonz_stationary_threshold_mps;

    // Side-lobe removal (SLR) - we define a baseline threshold level (as SNR
    // dB) below which all samples are considered noise.  Large targets will
    // raise this noise threshold for all the other samples in the same
    // activation, based on a simple notch curve.
    FLOAT notch_width_radians;      //!< the azimuth delta angle at which larger side-lobes appear (radians)
    FLOAT notch_depth_dB;           //!< the amount of clean SNR from target peak at near-angles (as dB)
    FLOAT outer_depth_dB;           //!< the amount of clean SNR from target peak at off-angles (as dB)
    FLOAT ridge_extra_thresh_dB;    //!< increase to per-angle thresholds when ridge is detected
    // TODO: Deprecate SLR configuration in favor of antenna-config calibrations

    void defaults()
    {
        apply_preset(LOW_SENSITIVITY);
    }

    void apply_preset(ThresholdPresetEnum p)
    {
        apply_slr_preset(p);
        apply_detection_threshold_preset(p);
    }

    void disable_slr()
    {
        notch_depth_dB     = 0.0F;
        outer_depth_dB     = 0.0F;
        ridge_extra_thresh_dB = 0.0F;
    }

    bool is_slr_enabled()
    {
        return (notch_depth_dB     != 0.0F) ||
               (outer_depth_dB     != 0.0F) ||
               (ridge_extra_thresh_dB != 0.0F);
    }

    void apply_detection_threshold_preset(ThresholdPresetEnum p)
    {
        ego_zero_stationary_threshold_mps = 0.05F;
        ego_nonz_stationary_threshold_mps = 0.5F;

        switch (p)
        {
        case LOW_SENSITIVITY:
            // thresholds
            detection_thresh_snr_dB     = 17.0F;
            clutter_image_thresh_snr_dB = 10.0F;
            point_cloud_thresh_snr_dB   = 13.0F;
            scale_det_thresh_max_range  = 20.0F;
            scale_det_thresh_snr_adj    = 6.0F;
            break;

        case HIGH_SENSITIVITY:
            // thresholds
            detection_thresh_snr_dB     = 14.0F;
            clutter_image_thresh_snr_dB = 6.0F;
            point_cloud_thresh_snr_dB   = 10.0F;
            scale_det_thresh_max_range  = 8.0F;
            scale_det_thresh_snr_adj    = 3.0F;
            break;

        default:
            break;
        }
    }

    void apply_slr_preset(ThresholdPresetEnum p)
    {
        notch_width_radians = 0.47123889803846897f; // 27 degrees

        switch (p)
        {
        case LOW_SENSITIVITY:
            notch_depth_dB              = 10.0F;
            outer_depth_dB              = 7.0F;
            ridge_extra_thresh_dB       = 6.0F;
            break;

        case HIGH_SENSITIVITY:
            // SLR
            notch_depth_dB              = 18.0F;
            outer_depth_dB              = 15.0F;
            ridge_extra_thresh_dB       = 1.0F;
            break;

        default:
            break;
        }
    }
};

SRS_CLOSE_NAMESPACE()

#endif // ifndef SRS_HDR_RDC_THRESHCTRL_H
