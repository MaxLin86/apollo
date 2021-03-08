#ifndef SRS_HDR_WINDOW_H
#define SRS_HDR_WINDOW_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

SRS_DECLARE_NAMESPACE()

extern const CHAR* rdc_window_names[];
enum RDC_WindowType
{
    WINDOW_BOXCAR,
    WINDOW_HANN,
    WINDOW_HAMMING,
    WINDOW_GAUSS,
    WINDOW_BLACKMAN,
    WINDOW_BLACKMAN_NUTTALL,
    WINDOW_BLACKMAN_HARRIS,
    WINDOW_CHEBYSHEV_30,
    WINDOW_REMEZ,
    WINDOW_TAYLOR_30,
    WINDOW_CHEBYSHEV_45,
    WINDOW_CHEBYSHEV_60,
    WINDOW_TAYLOR_55,

    // Beamforming-only window options
    WINDOW_SVA,
    MAX_WINDOW_TYPE
};

enum REMEZ_WINDOW_SIZE
{
    REMEZ_WINDOW_16 = 0,
    REMEZ_WINDOW_48,
    REMEZ_WINDOW_144,
    REMEZ_WINDOW_224,
    REMEZ_WINDOW_240,
    REMEZ_WINDOW_252,
    REMEZ_WINDOW_288,
    REMEZ_WINDOW_336,
    REMEZ_WINDOW_NUM_SIZES
};

struct ChanRemezData
{
    uint32_t     window_size;
    const FLOAT *window_buf;
};

extern const ChanRemezData *chan_remez_info;

void calc_window(FLOAT*win, uint32_t n, RDC_WindowType wtype = WINDOW_BOXCAR);
bool is_remez_size_available(uint32_t size);

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_WINDOW_H
