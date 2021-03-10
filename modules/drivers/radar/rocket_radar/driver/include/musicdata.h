// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

SRS_DECLARE_NAMESPACE()
struct RDC_MusicSampleData;
SRS_CLOSE_NAMESPACE()

class MUSICData
{
public:

    virtual ~MUSICData() {}

    //! requires DL_RDC2, a scan with a valid channelizer ratio, and active MUSIC region requests
    //! This is the raw input to the covariance and smoothing stages. For debugging purposes, only
    virtual uint32_t        get_rdc2_bin_count() const = 0;

    //! requires DL_COV, a scan with a valid channelizer ratio, and active MUSIC region requests
    //! This is the raw input to the SVD stage. For debugging purposes, only
    virtual uint32_t        get_covariance_count() const = 0;

    //! requires DL_SVD, a scan with a valid channelizer ratio, and active MUSIC region requests
    //! This is the raw input to the MUSIC spectrum sampler. For debugging purposes, only
    virtual uint32_t        get_svd_count() const = 0;

    //! requires a scan with a valid channelizer ratio, and active MUSIC region requests
    //! This is the output of the MUSIC spectrum sampler, corresponding to the
    //! active MUSIC requests
    virtual uint32_t        get_output_count() const = 0;

    //! returns cint16[num_chan_iters][num_vrx] the VRX are in +Y major order
    virtual const cint16*   get_rdc2_bin(uint32_t  skidx,
                                         uint32_t& range_bin,
                                         uint32_t& doppler_bin,
                                         uint32_t& num_vrx,
                                         uint32_t& num_chan_iters) const = 0;

    //! returns cint64[size * size]
    virtual const cint64*   get_covariance(uint32_t covidx,
                                           uint32_t& range_bin,
                                           uint32_t& doppler_bin,
                                           uint32_t& size) const = 0;

    //! returns Unitary=cint32[size*size] followed by S=cint32[size]
    virtual const cint32*   get_svd(uint32_t svdidx,
                                    uint32_t& range_bin,
                                    uint32_t& doppler_bin,
                                    uint32_t& size) const = 0;

    //! returns sampled MUSIC spectrum data
    virtual const RDC_MusicSampleData* get_output(uint32_t reqidx) const = 0;

    //! release all storage of this scan's MUSIC related outputs
    virtual void            release() = 0;

    //! an enumeration of the errors potentially returned by this class
    enum Err
    {
        MUSIC_NO_ERROR,
        MUSIC_INDEX_OUT_OF_RANGE,
    };

    //! returns the last error encountered by this instance
    virtual Err             get_last_error() const = 0;
};
