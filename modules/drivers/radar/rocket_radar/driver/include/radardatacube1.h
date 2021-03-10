// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

class RadarDataCube1
{
public:

    virtual ~RadarDataCube1() {}

    //! returns the number of virtual receivers in RDC1
    virtual uint32_t      get_vrx_dimension() const = 0;

    //! returns the number of range bins in RDC1
    virtual uint32_t      get_range_dimension() const = 0;

    //! returns the number of pulses in RDC1
    virtual uint32_t      get_pulses_dimension() const = 0;

    //! returns samples as cint16 rdc1[PULSE][RANGE][VRX]
    //! if RDC1 is replayable, the stride is unknown and only the radar
    //! can properly interpret the returned buffer
    virtual const cint16* get_samples() const = 0;

    virtual uint32_t      get_exponent_at_pulse_range(uint32_t pulse, uint32_t range_bin) const = 0;

    //! returns raw exponent data
    virtual const char*   get_exponent_data() const = 0;

    //! returns RDC1 replay info (gains, thresholds)
    //! returns NULL if this is not a replayable RDC1 capture
    virtual const char*   get_rdc1_info() const = 0;

    //! returns size of RDC1 buffer in bytes
    virtual size_t        get_rdc1_size_bytes() const = 0;

    //! returns size of RDC1 exponents in bytes
    virtual size_t        get_rdc1_exp_size_bytes() const = 0;

    //! returns size of RDC1 info in bytes
    virtual size_t        get_rdc1_info_size_bytes() const = 0;

    //! returns true if the captured RDC1 buffer contains enough data to be
    //! replayable on a radar
    virtual bool          is_replayable() const = 0;

    //! release all of the memory held by the RDC1
    virtual void          release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        RDC1_NO_ERROR,
        RDC1_INDEX_OUT_OF_VRX,
        RDC1_FILE_WRITE_ERROR,
    };

    //! returns the last error encountered by this instance
    virtual Err get_last_error() const = 0;
};
