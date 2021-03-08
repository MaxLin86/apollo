#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

struct ProtHandlerBase
{
    bool disabled;

    ProtHandlerBase() : disabled(false) {}
    virtual ~ProtHandlerBase() {}

    virtual void handle_packet(const char* payload, uint32_t len) = 0;
    virtual void disable() { disabled = true; }
};

struct IgnoreHandler : public ProtHandlerBase
{
    void handle_packet(const char* payload, uint32_t len) {}
};

