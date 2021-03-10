// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_EVENT_ENUMS_H
#define SRS_HDR_EVENT_ENUMS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

enum EventEnum
{
    EV_NULL = 0, /*! special NULL event, ignored when posted */
#define DEFINE_EVENT(name) name,
#if SRS_BOOT_IMAGE_1
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/boot-eventdef.inc"
#else
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/engine-eventdef.inc"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/environment-eventdef.inc"
#endif
#undef DEFINE_EVENT
    NUM_EVENT_ENUMS
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_EVENT_ENUMS_H
