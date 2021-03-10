// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"

class ADCCaptureData
{
public:

    virtual ~ADCCaptureData() {}

    virtual void     get_adc_capture_info(uint16_t& rx_channel_mask, uint16_t& tx_channel_mask, uint32_t& samples_per_channel) const = 0;

    virtual void     get_adc_initial_rx_lanes(uint8_t initial_rx_lane[NUM_RX_PER_BANK]) const = 0;

    //! User must provide output storage and select the channel to be returned. Returns
    //! the count of samples written into the user buffer
    virtual uint32_t get_adc_samples(cint8*  user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map=0x1FF) const = 0;
    virtual uint32_t get_adc_samples(cint16* user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map=0x1FF) const = 0;

    virtual void     release() = 0;

    // Create an ADCCaptureData instance that is unassociated with any ScanObject
    static ADCCaptureData* load_raw_capture_file(const char* filename);
};

