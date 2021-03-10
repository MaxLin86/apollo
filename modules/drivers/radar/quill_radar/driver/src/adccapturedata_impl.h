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
#include "modules/drivers/radar/quill_radar/driver/include/adccapturedata.h"
#include "modules/drivers/radar/quill_radar/driver/include/serializer.h"

class ScanObject_Impl;

class ADCCaptureData_Impl: public ADCCaptureData
{
public:

    ADCCaptureData_Impl(ScanObject_Impl* so)
        : myscan(so)
        , adc_raw(NULL)
        , total_raw_bytes(0)
        , raw_bytes_received(0)
        , aborted(false)
    {
    }

    virtual ~ADCCaptureData_Impl()
    {
        free(adc_raw);
    }

    virtual void     get_adc_capture_info(uint16_t& rx_channel_mask, uint16_t& tx_channel_mask, uint32_t& samples_per_channel) const;

    virtual void     get_adc_initial_rx_lanes(uint8_t initial_rx_lane[NUM_RX_PER_BANK]) const;

    //! User must provide output storage and select the channel to be returned. Returns
    //! the count of samples written into the user buffer
    virtual uint32_t get_adc_samples(cint8*  user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map=0x1FF) const;
    virtual uint32_t get_adc_samples(cint16* user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map=0x1FF) const;

    virtual void     release();

            bool     deserialize(ScanSerializer& s);

            bool     serialize(ScanSerializer& s);

            void     handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size);

            void     finish_uhdp(uint8_t last_message_type);

    ScanObject_Impl*    myscan;

    UhdpADCDescriptor   desc;

    void*               adc_raw;

    uint32_t            total_raw_bytes;

    uint32_t            raw_bytes_received;

    bool                aborted;
};

