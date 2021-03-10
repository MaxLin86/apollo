// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

struct LogEvent;

class UserProfileAgent
{
public:

    virtual ~UserProfileAgent() {}

    virtual void set_num_producers(uint32_t producer_count) = 0;

    virtual void set_producer(uint32_t producer_id, const char* prod_name, uint32_t num_subunits) = 0;

    virtual void set_producer_subunit(uint32_t producer_id, uint32_t subunit_id, const char* subunit_name) = 0;

    virtual void set_task_names(uint32_t num_tasks, const char* const* names) = 0;

    virtual void received_ppa_events(uint32_t cpu_id, const LogEvent* events, uint32_t count) = 0;
};
