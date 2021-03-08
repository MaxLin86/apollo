#ifndef SRS_HDR_ENGINE_INTERRUPTS_H
#define SRS_HDR_ENGINE_INTERRUPTS_H 1
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
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/irq-enums.h"

SRS_DECLARE_NAMESPACE()

void PGU_int0handler(void *);
void PGU_int1handler(void *);
void ICU_int0handler(void *);
#if WITH_DIAGS && WITH_CAL_DIAGS
void ICUint1handler(void *); // diag_measure.cpp
#endif

struct ISRDescriptor
{
    ISR_interrupt   type;
    ISR_trigger     trigger;
    ISR_target_cpu  targetCPU;
    void           (*handler)(void *);
};

/* The engine requires these interrupts to be configured by the environment */
static const ISRDescriptor engine_isr_descs[] =
{
    { GIC_SPI_RSP_PGU_INT_0, EDGE_TRIGGERED, GIC_SCP_TARGET, &PGU_int0handler },
    { GIC_SPI_RSP_PGU_INT_1, EDGE_TRIGGERED, GIC_SCP_TARGET, &PGU_int1handler },
    { GIC_SPI_RSP_ICU_INT_0, EDGE_TRIGGERED, GIC_SCP_TARGET, &ICU_int0handler },
#if WITH_DIAGS && WITH_CAL_DIAGS
    { GIC_SPI_RSP_ICU_INT_1, EDGE_TRIGGERED, GIC_SCP_TARGET, &ICUint1handler },
#endif
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ENGINE_INTERRUPTS_H
