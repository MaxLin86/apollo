// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

struct ProtHandlerBase
{
    bool disabled;

    ProtHandlerBase() : disabled(false) {}
    virtual ~ProtHandlerBase() {}

    virtual void handle_packet(const char* payload, uint32_t len) = 0;
    virtual void disable() { disabled = true; }
};
