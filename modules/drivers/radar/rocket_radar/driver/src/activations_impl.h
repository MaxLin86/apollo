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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
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

        BucketData() : skewers(NULL), total_skewers(0), num_skewers(0), num_sum(0) {}

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
