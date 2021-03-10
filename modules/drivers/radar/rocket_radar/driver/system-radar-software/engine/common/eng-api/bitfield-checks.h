// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_BITFIELD_CHECKS_H
#define SRS_HDR_BITFIELD_CHECKS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhnder-helpers.h"

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
