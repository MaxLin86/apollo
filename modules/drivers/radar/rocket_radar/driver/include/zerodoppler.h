// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

class ZeroDoppler
{
public:

    virtual ~ZeroDoppler() {}

    //! returns the number of range bins in the zero-doppler plane. Note that
    //! the captured zero-doppler plane may be a subset of the full RDC2
    //! zero-doppler. Two fields in UhdpScanInformation indicate which range
    //! bins were captured.
    //!     uint16_t   rdc2_zd_rb_center;
    //!     uint16_t   rdc2_zd_rb_halfwidth;
    //! This function returns min(2 * rdc2_zd_rb_halfwidth + 1, num_range_bins)
    //! which is the actual number of range bins captured.
    virtual uint32_t get_range_dimension() const = 0;

    //! returns the number of virtual receivers in the zero-doppler plane
    virtual uint32_t get_vrx_dimension() const = 0;

    //! can return NULL if range bin is out of range. returns a complex 64bit integer
    //! per virtual receiver, in +Y major order. range_bin must be between 0 and
    //! the value returned by get_range_dimension();
    virtual const cint64* get_range_bin(uint32_t range_bin) const = 0;

    //! release all of the memory held by this ZeroDoppler instance
    virtual void     release() = 0;
};
