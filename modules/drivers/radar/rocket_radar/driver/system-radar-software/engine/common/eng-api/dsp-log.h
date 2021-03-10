// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_DSP_LOG_H
#define SRS_HDR_DSP_LOG_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

enum DspLogSubunits
{
    DSP_ZERO_DOPPLER_RPC,
    DSP_HISTOGRAM_RPC,
    DSP_DETECTION_RPC,
    DSP_POINTCLOUD_RPC,
    DSP_CLUTTER_IMAGE_RPC,
    DSP_EGO_VELOCITY_RPC,
    DSP_MUSIC_RPC,
    NUM_DSP_SUBUNITS
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DSP_LOG_H
