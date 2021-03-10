// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_ICCQ_MEMORY_H
#define SRS_HDR_ICCQ_MEMORY_H 1

#if SRS_BOOT_IMAGE

#include "b1_mem.h"

#if __SCP__
#define SYS_SRAM_ICCQ_AREA   SCP_B1_SCP_TCMB_IPC_START
#else
#define SYS_SRAM_ICCQ_AREA   HSM_B1_SCP_TCMB_IPC_START
#endif //__SCP__

#else // !SRS_BOOT_IMAGE

#define SYS_SRAM_ICCQ_AREA   0X113FC200

#endif//SRS_BOOT_IMAGE

// Currently we need only 13K out of 15K
// Please be careful when changing any of these values for increasing the ICCQ memory
// Top of SRAM from address 0X200FFE00 to 0X20100000 is used for exception core dump
#define SYS_SRAM_ICCQ_AREA_SIZE  0x3C00

#endif // SRS_HDR_ICCQ_MEMORY_H
