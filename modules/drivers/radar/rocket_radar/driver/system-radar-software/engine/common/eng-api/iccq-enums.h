#ifndef SRS_HDR_ICCQ_ENUMS_H
#define SRS_HDR_ICCQ_ENUMS_H 1
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

SRS_DECLARE_NAMESPACE()

enum ICCQQueueEnum
{
    /* loopback events are posted by local CPU, possibly via timer */
    ICCQ_SCP_TO_DSP1,
    ICCQ_SCP_TO_DSP1_HIGH_PRIORITY,
    ICCQ_SCP_TO_DSP2,
    ICCQ_SCP_TO_DSP2_HIGH_PRIORITY,
    ICCQ_SCP_TO_CCP,
    ICCQ_SCP_TO_HSM,
    ICCQ_DSP1_TO_SCP,
    ICCQ_DSP1_TO_SCP_HIGH_PRIORITY,
    ICCQ_DSP2_TO_SCP,
    ICCQ_DSP2_TO_SCP_HIGH_PRIORITY,
    ICCQ_CCP_TO_SCP,
    ICCQ_HSM_TO_SCP,

    NUM_ICCQ_QUEUES,

    ICCQ_LOOPBACK,   /*! special queue ID means local source */
};

enum ICCQServiceEnum
{
#define DEFINE_ICCQ(name) name,
#if SRS_BOOT_IMAGE_1
#include "boot-iccqdef.inc"
#else
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/engine-iccqdef.inc"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/environment-iccqdef.inc"
#endif
#undef DEFINE_ICCQ
    NUM_ICCQ_SERVICES,

    ICCQ_SERVICE_DUP, /*! (as source service) same as destination port */
    ICCQ_SERVICE_NOP, /*! (as source service) no response is expected */
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ICCQ_ENUMS_H
