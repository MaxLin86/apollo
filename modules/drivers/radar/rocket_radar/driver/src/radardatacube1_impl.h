// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/include/radardatacube1.h"

class ScanObject_Impl;
class ScanSerializer;

class RadarDataCube1_Impl: public RadarDataCube1
{
public:

    struct RangeExponentData
    {
        uint16_t base_exp;
        uint16_t max_growth;
        uint16_t incr_pulse[8];
    };

    RadarDataCube1_Impl(ScanObject_Impl& s)
        : myscan(s)
        , last_err(RDC1_NO_ERROR)
        , rdc1(NULL)
        , rdc1exp(NULL)
        , range_exponents(NULL)
        , rdc1_playback_data(NULL)
        , rdc1_playback_received(0)
        , rdc1_playback_total_bytes(0)
        , aborted(false)
    {
    }

    virtual ~RadarDataCube1_Impl()
    {
        if (rdc1_playback_data)
        {
            delete [] rdc1_playback_data;
        }
        else
        {
            delete [] rdc1;
            delete [] rdc1exp;
        }
        rdc1 = NULL;
        rdc1exp = NULL;
        rdc1_playback_data = NULL;

        range_exponents = NULL;
        delete [] range_exponents;
    }

    virtual uint32_t             get_range_dimension() const;

    virtual uint32_t             get_vrx_dimension() const;

    virtual uint32_t             get_pulses_dimension() const;

    virtual const cint16*        get_samples() const;

    virtual const char*          get_exponent_data() const;

    virtual const char*          get_rdc1_info() const;

    virtual uint32_t             get_exponent_at_pulse_range(uint32_t pulse, uint32_t range_bin) const;

    virtual size_t               get_rdc1_size_bytes() const;

    virtual size_t               get_rdc1_exp_size_bytes() const;

    virtual size_t               get_rdc1_info_size_bytes() const;

    virtual bool                 is_replayable() const { return rdc1_playback_data != NULL; }

    virtual void                 release();

    virtual Err                  get_last_error() const { return last_err; }

            void                 allocate();

            void                 handle_uhdp(const char *payload, uint32_t total_size);

            void                 handle_replay_uhdp(const char* payload, uint32_t total_size);

            void                 setup();

            bool                 serialize(ScanSerializer& s) const;

            bool                 deserialize(ScanSerializer& s);

    ScanObject_Impl& myscan;

    mutable Err      last_err;

    cint16*          rdc1;

    char*            rdc1exp;

    RangeExponentData* range_exponents;

    char*            rdc1_playback_data;

    uint32_t         rdc1_playback_received;

    uint32_t         rdc1_playback_total_bytes;

    bool             aborted;
};
