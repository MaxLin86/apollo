// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"


namespace RRA
{
#include "rra-revdesc.h" // machine generated header in build folder
}

namespace SRS
{
#include "srs-revdesc.h" // machine generated header in build folder
}

const char* get_rra_version_string()
{
    return RRA::swrev + strlen("hg-tag-string:");
}

const char* get_compiled_srs_version_string(uint32_t* revids)
{
    if (revids)
    {
        revids[0] = SRS::swrev_id[0];
        revids[1] = SRS::swrev_id[1];
    }
    return SRS::swrev + strlen("hg-tag-string:");
}

const char* get_compiled_srs_timestamp_string()
{
    return SRS::timestamp;
}

const char* get_compiled_srs_path_string()
{
    return SRS::srs_path;
}
