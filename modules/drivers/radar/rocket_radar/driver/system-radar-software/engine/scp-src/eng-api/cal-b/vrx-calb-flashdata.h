#ifndef SRS_HDR_VRX_CALB_FLASHDATA_H
#define SRS_HDR_VRX_CALB_FLASHDATA_H 1
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
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"

SRS_DECLARE_NAMESPACE()

struct VrxAlignCalBData
{
    //! relative TX delay in units of picoseconds, +ve moves target away
    //! all delay_tx values must be greater than or equal to zero
    FLOAT     delay_tx[NUM_TX];

    //! relative RX delay in units of picoseconds, +ve moves target closer
    //! can be negative or positive
    FLOAT     delay_rx[NUM_RX_BANKS][NUM_RX_PER_BANK];

    //! measured ADI delay_computed as seen with the above correction applied
    //! this data is needed to apply the correction reliably
    uint8_t   delay_computed[NUM_RX_BANKS][NUM_RX_PER_BANK];

    // The net effect of applying both the RX and TX delays should be that
    // targets appear at the correct range, and all virtual receivers are
    // aligned in range (and thus in time)

    uint32_t  padding[2 * (NUM_TX + (NUM_RX_BANKS * NUM_RX_PER_BANK)) - 4];

    void uhprint() const
    {
        for (uint32_t i = 0; i < NUM_TX; i++)
        {
            if (i < NUM_RX_PER_BANK)
            {
                UHPRINTF("TX[%d] delay: %5.1f  RX[%d] delay: Bank0 %5.1f (%d) Bank1 %5.1f (%d)\n",
                        i, delay_tx[i],
                        i, delay_rx[0][i], delay_computed[0][i],
                           delay_rx[1][i], delay_computed[1][i]);
            }
            else
            {
                UHPRINTF("TX[%d] delay: %5.1f\n", i, delay_tx[i]);
            }
        }
    }

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));
    }
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_VRX_CALB_FLASHDATA_H
