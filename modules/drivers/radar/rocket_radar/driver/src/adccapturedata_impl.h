// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/adccapturedata.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"

class ScanObject_Impl;

class ADCCaptureData_Impl: public ADCCaptureData
{
public:

    ADCCaptureData_Impl(ScanObject_Impl* so)
        : myscan(so)
        , adc_raw(NULL)
        , total_raw_bytes(0)
        , raw_bytes_received(0)
        , aborted(false)
    {
    }

    virtual ~ADCCaptureData_Impl()
    {
        free(adc_raw);
    }

    virtual void     get_adc_capture_info(uint16_t& rx_channel_mask, uint16_t& tx_channel_mask, uint32_t& samples_per_channel) const;

    virtual void     get_adc_initial_rx_lanes(uint8_t initial_rx_lane[NUM_RX_PER_BANK]) const;

    //! User must provide output storage and select the channel to be returned. Returns
    //! the count of samples written into the user buffer
    virtual uint32_t get_adc_samples(cint8*  user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map=0x1FF) const;
    virtual uint32_t get_adc_samples(cint16* user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map=0x1FF) const;

    virtual void     release();

            bool     deserialize(ScanSerializer& s);

            bool     serialize(ScanSerializer& s);

            void     handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size);

            void     finish_uhdp(uint8_t last_message_type);

    ScanObject_Impl*    myscan;

    UhdpADCDescriptor   desc;

    void*               adc_raw;

    uint32_t            total_raw_bytes;

    uint32_t            raw_bytes_received;

    bool                aborted;
};

