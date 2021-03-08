#ifndef SRS_HDR_QILO_CALB_FLASHDATA_H
#define SRS_HDR_QILO_CALB_FLASHDATA_H 1
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

SRS_DECLARE_NAMESPACE()

struct QiloCalBDataRaw
{
    /* temperature and carrier dependent */
    uint8_t dcap_hv[NUM_RX_PER_BANK];
    uint8_t qilo_dcaps[NUM_TX];

    /* temperature dependent, maybe carrier */
    uint8_t vga2_config__bn_i[NUM_RX_PER_BANK];
    uint8_t vga2_config__bn_q[NUM_RX_PER_BANK];
    uint8_t vga2_config__bp_i[NUM_RX_PER_BANK];
    uint8_t vga2_config__bp_q[NUM_RX_PER_BANK];
    uint8_t rxfe3_config__rx_mx_i_bc[NUM_RX_PER_BANK];
    uint8_t rxfe3_generic__rx_mx_q_bc[NUM_RX_PER_BANK];

    /* should be set only once at boot, initializes track-and-hold circuit */
    uint8_t vga0_generic__bias4_i[NUM_RX_PER_BANK];
    uint8_t vga2_generic__bias4_q[NUM_RX_PER_BANK];

    void set_reg_defaults();

    void uhprint() const
    {
        for (uint32_t rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            UHPRINTF("RX[%d] dcap_hv: %u, vga2 bn: [%u,%u], bp: [%u,%u], mx_bc: [%u,%u], bias4: [%u,%u]\n",
                    rx,
                    dcap_hv[rx],
                    vga2_config__bn_i[rx],
                    vga2_config__bn_q[rx],
                    vga2_config__bp_i[rx],
                    vga2_config__bp_q[rx],
                    rxfe3_config__rx_mx_i_bc[rx],
                    rxfe3_generic__rx_mx_q_bc[rx],
                    vga0_generic__bias4_i[rx],
                    vga2_generic__bias4_q[rx]);
        }
        UHPRINTF("TX dcaps: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
                qilo_dcaps[0], qilo_dcaps[1], qilo_dcaps[2], qilo_dcaps[3],
                qilo_dcaps[4], qilo_dcaps[5], qilo_dcaps[6], qilo_dcaps[7],
                qilo_dcaps[8], qilo_dcaps[9], qilo_dcaps[10], qilo_dcaps[11]);

    }
};


struct QiloCalBData : public QiloCalBDataRaw
{
    uint8_t padding[256 - sizeof(QiloCalBDataRaw)];

    void set_defaults()
    {
        set_reg_defaults();
        memset(&padding[0], 0xFF, sizeof(padding));
    }
};

SRS_CLOSE_NAMESPACE()

#endif
