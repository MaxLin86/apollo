// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/activations.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

class ScanObject_Impl;
class ScanSerializer;
struct Skewer;

class Activations_Impl : public Activations
{
public:

    struct BucketData
    {
        Skewer*   skewers;

        uint32_t  total_skewers;

        uint32_t  num_skewers;

        uint32_t  num_sum;

        uint32_t  rb_start;

        BucketData() : skewers(NULL), total_skewers(0), num_skewers(0), num_sum(0), rb_start(0) {}

        ~BucketData();
    };

    Activations_Impl(ScanObject_Impl& scan)
        : myscan(scan)
        , aborted(false)
        , last_err(ACT_NO_ERROR)
    {
    }

    virtual ~Activations_Impl() {}

    virtual uint32_t        get_count(BucketEnum bucket) const;

    virtual uint32_t        get_num_angle_bins() const;

    virtual bool            complex_data_available() const;

    virtual const uint16_t* get_raw_samples(BucketEnum bucket,
                                            uint32_t  act_idx,
                                            int16_t&  exponent,
                                            uint16_t& range_bin,
                                            uint16_t& doppler_bin,
                                            uint16_t& max_val) const;

    virtual const cint16*   get_raw_samples_complex(BucketEnum bucket,
                                            uint32_t  act_idx,
                                            int16_t&  exponent,
                                            uint16_t& range_bin,
                                            uint16_t& doppler_bin,
                                            uint16_t& max_val) const;

    virtual void     release();

    virtual Err      get_last_error() const { return last_err; }

            bool     deserialize(ScanSerializer&);

            bool     serialize(ScanSerializer&) const;

            void     handle_uhdp(UhdpRDC3Header* rdc3hdr, uint32_t total_size);

            void     setup();

    ScanObject_Impl& myscan;

    BucketData       buckets[NUM_RD_BUF];

    bool             aborted;

    mutable Err      last_err;
};
