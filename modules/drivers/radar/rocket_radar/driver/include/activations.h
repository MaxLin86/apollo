// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

class Activations
{
public:

    virtual ~Activations() {}

    //! The Sabine radar emits activations into three buckets:
    //! THRESH_LO - activations below 25dB SNR
    //! THRESH_HI - activations above 25dB SNR
    //! CUSTOM_RD - activations requested by software
    enum BucketEnum { THRESH_LO, THRESH_HI, CUSTOM_RD };

    //! returns uint16_t num_beamforming_angles
    virtual uint32_t        get_num_angle_bins() const = 0;

    //! returns true if complex activations were captured
    virtual bool            complex_data_available() const = 0;

    //! returns the number of activations (Range/Doppler Skewers above PFA) in
    //! the specified bucket
    virtual uint32_t        get_count(BucketEnum bucket) const = 0;

    //! returns uint16_t magnitude[num_beamforming_angles]
    //! @param[in]  bucket        select an activation bucket
    //! @param[in]  act_idx       0..count - 1
    //! @param[out] exponent      exponent of magnitude values in this skewer
    //! @param[out] range_bin     range bin of this skewer
    //! @param[out] doppler_bin   doppler bin of this skewer
    //! @param[out] max_val       maximum magnitude vaue within this skewer
    virtual const uint16_t* get_raw_samples(BucketEnum bucket, uint32_t act_idx, int16_t& exponent,
                                            uint16_t& range_bin, uint16_t& doppler_bin, uint16_t& max_val) const = 0;

    virtual const cint16*   get_raw_samples_complex(BucketEnum bucket, uint32_t act_idx, int16_t& exponent,
                                            uint16_t& range_bin, uint16_t& doppler_bin, uint16_t& max_val) const = 0;

    //! release all storage of the activations
    virtual void     release() = 0;

    //! an enumeration of the errors potentially returned by this class
    enum Err
    {
        ACT_NO_ERROR,
        ACT_INDEX_OUT_OF_RANGE,
        ACT_FILE_WRITE_FAILURE,
    };

    //! returns the last error encountered by this instance
    virtual Err      get_last_error() const = 0;
};
