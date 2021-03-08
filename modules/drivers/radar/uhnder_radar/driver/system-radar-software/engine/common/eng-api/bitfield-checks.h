#pragma once
#ifndef SRS_HDR_BITFIELD_CHECKS_H
#define SRS_HDR_BITFIELD_CHECKS_H 1
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
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhnder-helpers.h"

SRS_DECLARE_NAMESPACE()

namespace
{
    enum AnEnum { VAL = 1 };
    AnEnum an_enum;
};

static void check_bitmap_compiler_support()
{
    StaticAssert<sizeof(an_enum) == 4>::istrue();

    uint32_t fake_reg;

    // ensure bits are filled from LSB
    struct Test1
    {
        uint32_t bit : 1;
    } *t1 = (Test1*)&fake_reg;

    fake_reg = 0U; t1->bit = 1U;
    UHASSERT(fake_reg == 1U);

    struct Test2
    {
        uint32_t reserved : 31;
        uint32_t bit : 1;
    } *t2 = (Test2*)&fake_reg;

    fake_reg = 0U; t2->bit = 1U;
    UHASSERT(fake_reg == (1U << 31U));

    struct Test3
    {
        uint32_t bit : 1;
        uint32_t field : 3;
    } *t3 = (Test3*)&fake_reg;

    // multi-bit field
    fake_reg = 0U; t3->field = 7U;
    UHASSERT(fake_reg == (7U << 1));

    // preserves fields
    fake_reg = 1U; t3->field = 7U;
    UHASSERT(fake_reg == ((7U << 1U) | 1U));
}

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_BITFIELD_CHECKS_H
