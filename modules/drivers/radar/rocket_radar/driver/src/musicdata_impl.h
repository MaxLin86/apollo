// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/musicdata.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"

class ScanObject_Impl;
class ScanSerializer;
struct CovarianceData;
struct SVDData;

class MUSICData_Impl : public MUSICData
{
public:

    MUSICData_Impl(ScanObject_Impl& scan)
        : last_err(MUSIC_NO_ERROR)
        , myscan(scan)
        , rdc2_summaries(NULL)
        , rdc2_skewers(NULL)
        , covariances(NULL)
        , svds(NULL)
        , samples(NULL)
        , total_rdc2_rdbin(0)
        , cur_rdc2_summary(0)
        , cur_rdc2_skewer(0)
        , cur_covariance(0)
        , cur_svd(0)
        , cur_sample(0)
    {
    }

    virtual ~MUSICData_Impl();

    virtual uint32_t        get_rdc2_bin_count() const { return total_rdc2_rdbin; }

    virtual uint32_t        get_covariance_count() const;

    virtual uint32_t        get_svd_count() const;

    virtual uint32_t        get_output_count() const;

    virtual const cint16*   get_rdc2_bin(uint32_t  skidx,
                                         uint32_t& range_bin,
                                         uint32_t& doppler_bin,
                                         uint32_t& num_vrx,
                                         uint32_t& num_chan_iters) const;

    virtual const cint64*   get_covariance(uint32_t covidx,
                                           uint32_t& range_bin,
                                           uint32_t& doppler_bin,
                                           uint32_t& size) const;

    virtual const cint32*   get_svd(uint32_t svdidx,
                                    uint32_t& range_bin,
                                    uint32_t& doppler_bin,
                                    uint32_t& size) const;

    virtual const RDC_MusicSampleData* get_output(uint32_t reqidx) const;

    virtual void            release();

    virtual Err             get_last_error() const { return last_err; }

            void            handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size);

            void            abort_uhdp(uint8_t message_type);

            void            finish_uhdp(uint8_t message_type);

            bool            deserialize(ScanSerializer& s);

            bool            serialize(ScanSerializer& s) const;

    mutable Err             last_err;

    ScanObject_Impl&        myscan;

    RDC_RDsummary*          rdc2_summaries;

    cint16*                 rdc2_skewers;

    CovarianceData*         covariances;

    SVDData*                svds;

    RDC_MusicSampleData*    samples;

    uint32_t                total_rdc2_rdbin;

    uint32_t                cur_rdc2_summary;

    uint32_t                cur_rdc2_skewer;

    uint32_t                cur_covariance;

    uint32_t                cur_svd;

    uint32_t                cur_sample;
};
