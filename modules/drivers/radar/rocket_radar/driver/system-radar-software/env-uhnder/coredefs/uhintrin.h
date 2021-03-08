#ifndef SRS_HDR_UHINTRIN_H
#define SRS_HDR_UHINTRIN_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

#if SRS_PLATFORM_ARMCC

/* ARMCC defines both of these as intrinsics */
#define uh_nop()                        __nop()
#define uh_data_memory_barrier()        __dmb(0xFU)

#elif SRS_PLATFORM_GCCARM

/* these definitions are needed for GCC ARM */
static UHINLINE void uh_data_memory_barrier() { asm volatile ("dmb"); } // data memory barrier intrinsic
static UHINLINE void uh_nop()                 { asm volatile ("nop"); }

#elif SRS_PLATFORM_GHS

#define uh_nop()                        __NOP()
#define uh_data_memory_barrier()        __DMB_OPT(0xFU)
// others..
#define __dsb( opt )        __DSB_OPT( (opt) )
#define __isb()             __ISB()
#define CHANGE_SEC(sec, name) PRAGMA( ghs section sec = name )

#else

#define uh_nop()                        uh_sw_nop()
#define uh_data_memory_barrier()

#endif

#endif // SRS_HDR_UHINTRIN_H
