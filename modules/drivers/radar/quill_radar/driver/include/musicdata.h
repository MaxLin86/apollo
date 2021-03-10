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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

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
