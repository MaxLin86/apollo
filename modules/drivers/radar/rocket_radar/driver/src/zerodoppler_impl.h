// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/zerodoppler.h"

class ScanObject_Impl;
class ScanSerializer;

class ZeroDoppler_Impl : public ZeroDoppler
{
public:

    ZeroDoppler_Impl(ScanObject_Impl& scan)
        : myscan(scan)
        , plane(NULL)
        , cur_byte_offset(0)
        , num_zd_range_bins(0)
        , aborted(false)
    {
    }

    virtual ~ZeroDoppler_Impl()
    {
        delete [] plane;
    }

    virtual uint32_t get_range_dimension() const;

    virtual uint32_t get_vrx_dimension() const;

    virtual const cint64* get_range_bin(uint32_t r) const;

    virtual void     release();

            bool     deserialize(ScanSerializer&);

            bool     serialize(ScanSerializer&) const;

            void     handle_uhdp(uint32_t* payload, uint32_t total_size);

            void     allocate();

            void     setup();

    ScanObject_Impl& myscan;

    cint64*          plane;

    uint32_t         cur_byte_offset;

    uint32_t         num_zd_range_bins;

    bool             aborted;
};
