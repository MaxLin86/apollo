#ifndef SRS_HDR_QILO_CAL_FLASHDATA_H
#define SRS_HDR_QILO_CAL_FLASHDATA_H 1
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
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct QiloCalData
{
    int8_t     rx_dcap_value[NUM_RX_PER_BANK];
    int8_t     tx_dcap_value[NUM_TX];

    void uhprint() const
    {
        if (NUM_RX_PER_BANK >= 8)
        {
            UHPRINTF("RX: %d, %d, %d, %d, %d, %d, %d, %d%s\n",
                     rx_dcap_value[0], rx_dcap_value[1], rx_dcap_value[2],
                     rx_dcap_value[3], rx_dcap_value[4], rx_dcap_value[5],
                     rx_dcap_value[6], rx_dcap_value[7],
                     (NUM_RX_PER_BANK > 8) ? " ..." : "");
        }
        if (NUM_TX >= 12)
        {
            UHPRINTF("TX: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d%s\n",
                     tx_dcap_value[0], tx_dcap_value[1], tx_dcap_value[2],
                     tx_dcap_value[3], tx_dcap_value[4], tx_dcap_value[5],
                     tx_dcap_value[6], tx_dcap_value[7], tx_dcap_value[8],
                     tx_dcap_value[9], tx_dcap_value[10], tx_dcap_value[11],
                     (NUM_TX > 12) ? " ..." : "");
        }
    }

    QiloCalData()
    {
        set_defaults();
    }

    void set_defaults()
    {
        // negative values imply allowing the software to use factory default
        // QILO dcap values
        for (uint32_t i = 0; i < NUM_RX_PER_BANK; i++)
        {
            rx_dcap_value[i] = -1;
        }
        for (uint32_t i = 0; i < NUM_TX; i++)
        {
            tx_dcap_value[i] = -1;
        }
    }
};

struct QiloCalKey
{
    FLOAT                     temperature;  // Chip temperature C at which this cal was performed
    FLOAT                     carrier_freq;
    uint32_t                  reserved[6];

    enum { KEY_VERSION = 2 };

    QiloCalKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(QILO) Temperature: %.1f\n", temperature);
        UHPRINTF("      carrier freq: %.2f\n", carrier_freq);
    }

    void upgrade_from_version(uint32_t old_version, QiloCalData& data)
    {
        if (old_version == 1)
        {
            uint32_t freq = uint32_t(carrier_freq * 10.0F);

            if (freq < 725)
            {
                freq = 725;
            }
            else if (freq > 780)
            {
                freq = 780;
            }

            // baseline defaults that were used for Version1 calibrations
            uint16_t generic_dcap_hv;
            if (freq > 730)
            {
                generic_dcap_hv = 12 - ((freq - 725) / 4); //mod318: 2 at 77.5
            }
            else
            {
                generic_dcap_hv = 14 - ((freq - 725) / 4);
            }

            for (uint32_t tx = 0; tx < NUM_TX; tx++)
            {
                // in Version1, these were offsets. now they are absolute
                // values. the hardware register is 5 unsigned bits, so it fits
                // easily in 8 signed bits
                data.tx_dcap_value[tx] += generic_dcap_hv;
            }
            for (uint32_t i = 0; i < NUM_RX_PER_BANK; i++)
            {
                data.rx_dcap_value[i] += generic_dcap_hv;
            }
        }
    }

    void upgrade_from_version(uint32_t old_version, struct QiloCalBData& data) {}

    //!! This is the all-important key-search algorithm, find an 'exact-match' !!
    bool compare(const QiloCalKey& other) const
    {
        return uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F;
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const QiloCalKey& other) const
    {
        FLOAT dist = uh_fabsf(temperature - other.temperature) + uh_fabsf(carrier_freq - other.carrier_freq) * 100.0F;
        return dist;
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_QILO_CAL_FLASHDATA_H
