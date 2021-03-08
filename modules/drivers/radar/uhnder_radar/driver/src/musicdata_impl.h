// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE
#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/musicdata.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"

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

    RHAL_RDsummary*         rdc2_summaries;

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
