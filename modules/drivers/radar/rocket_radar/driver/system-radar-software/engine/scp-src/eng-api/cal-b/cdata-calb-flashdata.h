#ifndef SRS_HDR_CDATA_CALB_FLASHDATA_H
#define SRS_HDR_CDATA_CALB_FLASHDATA_H 1
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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct CDataCalBLane
{
    struct CdataCalBChan
    {
        uint64_t offset : 8;
        uint64_t c3: 3;
        uint64_t c4: 4;
        uint64_t c5: 5;
        uint64_t c6: 6;
        uint64_t c7: 7;
        uint64_t : 31;
    }
    iq[2];
};

struct CDataCalBDataRaw
{
    CDataCalBLane       cdata[NUM_RX_PER_BANK][NUM_ADC_LANE];

    static void u32_to_bin(uint32_t val, int bits, char *output)
    {
        int ch = 0;
        for (int i = bits-1; i >= 0; i--)
        {
            output[ch++] = (val & (1U << i)) ? '1' : '0';
        }
        output[ch] = 0;
    }

    void uhprint(bool all=true) const
    {
        char i_of[9], q_of[9];
        char i_c3[4], q_c3[4];
        char i_c4[5], q_c4[5];
        char i_c5[6], q_c5[6];
        char i_c6[7], q_c6[7];
        char i_c7[8], q_c7[8];

        for (uint32_t rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            for (uint32_t ln = 0; ln < NUM_ADC_LANE; ln++)
            {
                u32_to_bin(cdata[rx][ln].iq[0].offset, 8, i_of);
                u32_to_bin(cdata[rx][ln].iq[0].c3    , 3, i_c3);
                u32_to_bin(cdata[rx][ln].iq[0].c4    , 4, i_c4);
                u32_to_bin(cdata[rx][ln].iq[0].c5    , 5, i_c5);
                u32_to_bin(cdata[rx][ln].iq[0].c6    , 6, i_c6);
                u32_to_bin(cdata[rx][ln].iq[0].c7    , 7, i_c7);

                u32_to_bin(cdata[rx][ln].iq[1].offset, 8, q_of);
                u32_to_bin(cdata[rx][ln].iq[1].c3    , 3, q_c3);
                u32_to_bin(cdata[rx][ln].iq[1].c4    , 4, q_c4);
                u32_to_bin(cdata[rx][ln].iq[1].c5    , 5, q_c5);
                u32_to_bin(cdata[rx][ln].iq[1].c6    , 6, q_c6);
                u32_to_bin(cdata[rx][ln].iq[1].c7    , 7, q_c7);

                UHPRINTF("RX%1d Lane%1d: I: offset=%s c3=%s c4=%s c5=%s c6=%s c7=%s    Q: offset=%s c3=%s c4=%s c5=%s c6=%s c7=%s\n",
                        rx, ln, i_of, i_c3, i_c4, i_c5, i_c6, i_c7, q_of, q_c3, q_c4, q_c5, q_c6, q_c7);
            }

            if (!all)
                break;
        }
    }

    void set_reg_defaults()
    {
        memset(this, 0U, sizeof(*this));
    }
};

struct CDataCalBData : public CDataCalBDataRaw
{
    uint8_t padding[1460 - sizeof(CDataCalBDataRaw)];

    void set_defaults()
    {
        memset(this, 0xFFU, sizeof(*this));
        set_reg_defaults();
    }
};

struct CDataCalBKey
{
    FLOAT                     temperature;     // Chip temperature C at which this cal was performed
    uint32_t                  adc_sample_rate;

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;

    enum { KEY_VERSION = 1 };

    CDataCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(CDATA) Temperature: %f\n", temperature);
        UHPRINTF("ADC sample rate: %d\n", adc_sample_rate);
    }

#if __SCP__
    void upgrade_from_version(uint32_t, CDataCalBData&) { }
#endif

    //!! This is the all-important key-search algorithm, find an 'exact-match' !!
    bool compare(const CDataCalBKey& other) const
    {
        return adc_sample_rate == other.adc_sample_rate;
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const CDataCalBKey& other) const
    {
        return uh_fabsf(temperature - other.temperature) +
               UH_ABS(INT(adc_sample_rate) - INT(other.adc_sample_rate));
    }
};

SRS_CLOSE_NAMESPACE()

#endif
