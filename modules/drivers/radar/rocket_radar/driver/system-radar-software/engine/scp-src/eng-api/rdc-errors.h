// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RDC_ERRORS_H
#define SRS_HDR_RDC_ERRORS_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

enum RDC_LogSubunit
{
    LOG_RDC_LAYER,
    LOG_RDC_FRAME_CONFIG,
    LOG_RDC_SCAN_CONFIG,
    LOG_RDC_CALIB,
    LOG_RDC_POST_PROC,
    // Insert more RDC classes here..
    LOG_RDC_SUBUNIT_MAX
};

enum RDC_error
{
    RDC_ERR_OK = 0,
    RDC_ERR_invalid_param,
    RDC_ERR_bad_state,
    RDC_ERR_no_memory,
    RDC_ERR_scanning_enabled,
    RDC_ERR_not_allocated,
    RDC_ERR_rhal_error,
    RDC_ERR_no_more,
    RDC_ERR_not_found,
    RDC_ERR_internal,

    // Insert additional RDC API error codes here...

    RDC_ERR_unknown
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_ERRORS_H
