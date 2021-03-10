// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_SYS_OS_TIMER_H
#define SRS_HDR_SYS_OS_TIMER_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "sys_timer_regs.h"
#include "uhio.h"

SRS_DECLARE_NAMESPACE()

#if __SCP__
void sys_timer1_init(uint32_t hw_divider);
#endif

// DIAG use only
void sys_timer1_start(void);
void sys_timer1_stop(void);

void sys_timer1_get_dividers(uint32_t& numerator, uint32_t& denominator);

//! read LSB 32bits of uptime counter. the counter wraps at maxint32
UHINLINE uint32_t sys_timer1_read(void)
{
    return fast_ior32(SYS_TIMER_COUNTER_COUNT_VALUE_C1);
}

//! read full 64bits of uptime counter. the counter wraps at maxint64
//
//!     uint32_t num, denom;
//!     sys_timer1_get_dividers(num, denom);
//!     uint64_t clock = sys_timer1_read_64bit_sw_timer();
//!     double microseconds = double(clock) * num / denom;
//
//! Note we don't recommend using the 64bit counter this way, only do
//! microsecond conversions on the PC or only do them on differences or deltas
//! on the radar
uint64_t sys_timer1_read_64bit_sw_timer(void);

void sys_timers_reset(void);

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_SYS_OS_TIMER_H
