// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"

class LogControl
{
public:

    virtual ~LogControl() {}

    virtual uint32_t    get_num_log_producers() const = 0;

    virtual const char* get_log_producer_name(uint32_t prod_id) const = 0;

    virtual uint32_t    get_num_log_producer_subunits(uint32_t prod_id) const = 0;

    virtual const char* get_log_producer_subunit_name(uint32_t prod_id, uint32_t subunit_id) const = 0;

    virtual void        set_log_filter(uint32_t prod_id, uint32_t subunit_id, LogLevelEnum level) = 0;

    virtual void        apply_log_filter_changes() = 0;
};
