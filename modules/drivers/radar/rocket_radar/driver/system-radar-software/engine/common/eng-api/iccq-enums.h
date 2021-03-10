// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_ICCQ_ENUMS_H
#define SRS_HDR_ICCQ_ENUMS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

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
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/environment-iccqdef.inc"
#endif
#undef DEFINE_ICCQ
    NUM_ICCQ_SERVICES,

    ICCQ_SERVICE_DUP, /*! (as source service) same as destination port */
    ICCQ_SERVICE_NOP, /*! (as source service) no response is expected */
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ICCQ_ENUMS_H
