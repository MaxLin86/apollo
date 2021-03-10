// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_ENGINE_INTERRUPTS_H
#define SRS_HDR_ENGINE_INTERRUPTS_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/irq-enums.h"
#include "pvt.h"

SRS_DECLARE_NAMESPACE()

struct ISRDescriptor
{
    ISR_interrupt   type;
    ISR_trigger     trigger;
    ISR_target_cpu  targetCPU;
    void           (*handler)(void *);
    ISR_intr_mode   mode;
};

/* The engine requires these interrupts to be configured by the environment */
static const ISRDescriptor engine_isr_descs[] =
{
    { GIC_SPI_PVT_TS_ALARMS_COMBINED_INT, LEVEL_TRIGGERED, GIC_SCP_TARGET, &pvt_ts_alarms_irq_handler, NON_SECURE },
    { GIC_SPI_PVT_VM_ALARMS_COMBINED_INT, LEVEL_TRIGGERED, GIC_SCP_TARGET, &pvt_vm_alarms_irq_handler, NON_SECURE },
    { GIC_SPI_PVT_PD_ALARMS_COMBINED_INT, LEVEL_TRIGGERED, GIC_SCP_TARGET, &pvt_pd_alarms_irq_handler, NON_SECURE },
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ENGINE_INTERRUPTS_H
