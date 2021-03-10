// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_CONTROL_MESSAGE_DEF_H
#define SRS_HDR_CONTROL_MESSAGE_DEF_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! Environment defined message type enumerations for UhdpControl
enum UhdpControlMessageType
{
    ENV_UHNDER_PING_EXAMPLE, // replace me
    ENV_UHNDER_EGO_VEL_DATA,
    ENV_UHNDER_EXTRA_ACTIVATION_DATA,

    NUM_CONTROL_MESSAGE_TYPES
};

//! message struct sent to radar with type ENV_UHNDER_EXTRA_ACTIVATION_DATA
struct ExtraActivationRequestMessage
{
    bool     track_target;
    float    range;                 // if track_target = false
    float    doppler;               // if track_target = false
    float    azimuth;               // if track_target = false
    float    elevation;             // if track_target = false
    uint16_t scan_count_duration;
    uint16_t range_bin_half_width;
    uint16_t doppler_bin_half_width;
    uint16_t azimuth_bin_half_width;
    uint16_t elevation_bin_half_width;
    uint16_t future_use;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_CONTROL_MESSAGE_DEF_H
