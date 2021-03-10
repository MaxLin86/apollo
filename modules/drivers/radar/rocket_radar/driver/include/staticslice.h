// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

class StaticSlice
{
public:

    virtual ~StaticSlice() {}

    //! number of slices captured (can be less than the number of slices in the scan)
    virtual uint32_t             get_num_slices() const = 0;

    //! number of angle bins in each static slice skewer
    virtual uint32_t             get_num_angle_bins() const = 0;

    //! returns true if complex static slice data was captured
    virtual bool                 complex_data_available() const = 0;

    //! returns the absolute (world) doppler velocity indicated by a particular
    //! doppler slice of the static slice
    virtual float                get_slice_velocity(uint32_t slice) const = 0;

    //! returns uint16_t magnitude[scan_info.SS_size_A]. If the static slice was
    //! captured as complex data, the magnitude of each sample is calculated and
    //! returned
    virtual const uint16_t*      get_skewer(uint32_t range_bin, uint32_t slice) const = 0;

    //! returns cint16 magnitude[scan_info.SS_size_A]. If the static slice was
    //! captured as magnitude only, the magnitude is returned as the I component
    //! and the Q is returned as 0.
    virtual const cint16*        get_skewer_complex(uint32_t range_bin, uint32_t slice) const = 0;

    //! returns the exponent which must be applied to the samples at the given
    //! range bin in order to measure the full gain.  Samples between range bins
    //! should not be compared without accounting for range exponent differences.
    virtual int16_t              get_exponent_at_range(uint32_t rbin) const = 0;

    //! release all of the memory held by the static slice
    virtual void                 release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        SS_NO_ERROR,
        SS_INDEX_OUT_OF_RANGE,
        SS_FILE_WRITE_ERROR,
    };

    //! returns the last error encountered by this instance
    virtual Err                  get_last_error() const = 0;
};
