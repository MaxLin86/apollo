// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_WINDOW_H
#define SRS_HDR_WINDOW_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"

SRS_DECLARE_NAMESPACE()

extern const CHAR* rdc_window_names[];

struct ChanRemezData
{
    uint32_t     window_size;
    const FLOAT *window_buf;
};

extern const ChanRemezData *chan_remez_info;

void calc_window(FLOAT*win, uint32_t n, RDC_WindowType wtype = WINDOW_BOXCAR);
bool is_remez_size_available(uint32_t size);

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_WINDOW_H
