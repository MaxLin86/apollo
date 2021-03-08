#ifndef SRS_HDR_DC_CALB_FLASHDATA_H
#define SRS_HDR_DC_CALB_FLASHDATA_H 1
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

struct DCCalBData
{
    struct VgaDcData
    {
        uint8_t vga0_generic__bf_i, vga1_config__offset1_i, vga1_config__offset2_i, vga1_config__offset3_i;
        uint8_t vga2_generic__bf_q, vga1_generic__offset1_q, vga1_generic__offset2_q, vga1_generic__offset3_q;
        uint8_t vga0_generic__bias4_i, vga2_generic__bias4_q;
    };

    VgaDcData   vga[NUM_RX_PER_BANK];
    cint16      color_gain[NUM_RX_PER_BANK][NUM_ADC_LANE]; // note - could be cuint8
    cint16      color_dc[NUM_RX_PER_BANK][NUM_ADC_LANE];
    uint32_t    algorithm_version;

    void uhprint(bool all=false) const
    {
        for (uint32_t rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            UHPRINTF("RX[%d] VGA IOFF: %u, %u, %u QOFF %u %u %u BQF %ui, %uq BIAS %ui %uq\n", rx,
                    vga[rx].vga1_config__offset1_i,  vga[rx].vga1_config__offset2_i, vga[rx].vga1_config__offset3_i,
                    vga[rx].vga1_generic__offset1_q, vga[rx].vga1_generic__offset2_q, vga[rx].vga1_generic__offset3_q,
                    vga[rx].vga0_generic__bf_i,      vga[rx].vga2_generic__bf_q,
                    vga[rx].vga0_generic__bias4_i,   vga[rx].vga2_generic__bias4_q);
            UHPRINTF("RX[%d] Color Gain IOFF: %u, %u, %u, %u, %u, %u, %u, %u, %u\n", rx,
                    color_gain[rx][0].i, color_gain[rx][1].i, color_gain[rx][2].i,
                    color_gain[rx][3].i, color_gain[rx][4].i, color_gain[rx][5].i,
                    color_gain[rx][6].i, color_gain[rx][7].i, color_gain[rx][8].i);
            UHPRINTF("RX[%d] Color Gain QOFF: %u, %u, %u, %u, %u, %u, %u, %u, %u\n", rx,
                    color_gain[rx][0].q, color_gain[rx][1].q, color_gain[rx][2].q,
                    color_gain[rx][3].q, color_gain[rx][4].q, color_gain[rx][5].q,
                    color_gain[rx][6].q, color_gain[rx][7].q, color_gain[rx][8].q);
            UHPRINTF("RX[%d] Color DC IOFF: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", rx,
                    color_dc[rx][0].i, color_dc[rx][1].i, color_dc[rx][2].i,
                    color_dc[rx][3].i, color_dc[rx][4].i, color_dc[rx][5].i,
                    color_dc[rx][6].i, color_dc[rx][7].i, color_dc[rx][8].i);
            UHPRINTF("RX[%d] Color DC QOFF: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", rx,
                    color_dc[rx][0].q, color_dc[rx][1].q, color_dc[rx][2].q,
                    color_dc[rx][3].q, color_dc[rx][4].q, color_dc[rx][5].q,
                    color_dc[rx][6].q, color_dc[rx][7].q, color_dc[rx][8].q);
            if (!all)
                break;
        }
    }

    void set_defaults();

};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DC_CALB_FLASHDATA_H
