// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_SABINE_B_CONFIG_H
#define SRS_HDR_SABINE_B_CONFIG_H 1

#define TCMA_START_ADDRESS_SYS_VIEW   0x12000000
#define TCMB_START_ADDRESS_SYS_VIEW   0x12800000
#define TCMA_START_ADDRESS            0x00000000
#define TCMA_SIZE                     (128 * 1024)
#define TCMA_RESERVED_SIZE            1024            // EXCEPTION_VECTOR_START, EXCEPTION_PAGE_SIZE

#define TCMB_START_ADDRESS            0x00800000
#define TCMB_SIZE                     (256 * 1024)
#define SCP_STACK_HEAP_ADDR           0x00800000

// Note: The SCP stack and heap are mapped to the same TCMB address. The stack
// is allocated from the top of the buffer, the heap is allocated from the
// bottom. The engine should not use much, if any at all, heap memory.
#define TCMB_SCP_STACK_SIZE           0x3400
#define TCMB_SCP_HEAP_SIZE            0x400

// Reserve bytes from end of TCMB for persistent modulecfg
#define TCMB_END_RESERVED_SIZE        2048
#define TCMB_END_RESERVED_ADDRESS     (TCMB_START_ADDRESS + TCMB_SIZE - TCMB_END_RESERVED_SIZE)
#define TCMB_END_RESERVED_ADDRESS_SYS_VIEW (TCMB_END_RESERVED_ADDRESS +TCMA_START_ADDRESS_SYS_VIEW)
#define SCP_STACK_ADDR_SYS_VIEW       (TCMA_START_ADDRESS_SYS_VIEW + SCP_STACK_HEAP_ADDR)

#define SYS_RAM_START_ADDRESS         0x11000000
#if __CERVELO_DSP__
// SCP_SRAM_CODE_START_ADDRESS + DSP1 CODE SIZE (0x50000)
#define SCP_SRAM_CODE_START_ADDRESS   0x110A0000
#else
#define SCP_SRAM_CODE_START_ADDRESS   0x11050000
#endif
#define SYS_SRAM_SIZE                 (4 * 1024 * 1024)

// SabineB will have at least 1 GB if there is DDR at all
// the actual amount might have to be detected at boot
#define DRAM_START_ADDRESS            0x40000000
#define DRAM_ECC_RESRV_SIZE           (128*1024*1024)//We loose last 128MB when we enable ECC on DDR mem
#define DRAM_SIZE                     ((1024 * 1024 * 1024) - DRAM_ECC_RESRV_SIZE)

// Reserve some DDR for code
#define DRAM_P5_RESERVED_SIZE         (512 * 1024)
#define DRAM_SCP_RESERVED_SIZE        (3 * 1024 * 1024)
#define DRAM_SCP_RESERVED_START_ADDR  (DRAM_START_ADDRESS + DRAM_P5_RESERVED_SIZE)
#define DRAM_USER_START_ADDRESS       (DRAM_SCP_RESERVED_START_ADDR + DRAM_SCP_RESERVED_SIZE)

#define DCU_START_ADDRESS             0x1B000000

/* HW BUG: even though DCU SRAM is 12MB, only lower 8MB is accessible from the CPU  */
/* the remaining 4MB is used for buffers used within RSPSS e.g. irdc2               */
/* there is no capability to handle upper 4MB from devmm. this memory is statically */
/* allocated in RHAL                                                                */
#define DCU_SIZE                      (8 * 1024 * 1024)

#define EXCEPTION_VECTOR_START        0x00000000UL
#define EXCEPTION_PAGE_SIZE           0x200UL

#define IRQ_STACK_SIZE                (4 * 1024)

#define EXCEPTION_STACK_SIZE          0x200 // 512 bytes
#define EXCEPTION_STACK               (SYS_RAM_START_ADDRESS + SYS_SRAM_SIZE)

/* Exception dump starts at */
#define EXCEPTION_REGION              (EXCEPTION_STACK - EXCEPTION_STACK_SIZE)
#define EXCEPTION_DUMP_REGION_SIZE    0x200 // 512 bytes
#define EXCEPTION_DUMP_MAGIC          (EXCEPTION_REGION)
#define EXCEPTION_DUMP_TYPE           (EXCEPTION_DUMP_MAGIC + 4)
#define EXCEPTION_EXCEPTION_DUMP_ADDR (EXCEPTION_DUMP_TYPE + 4)
#define EXCEPTION_REG_DUMP            (EXCEPTION_EXCEPTION_DUMP_ADDR + 4)

#define NUM_ETH_DEV 3

#endif // SRS_HDR_SABINE_B_CONFIG_H
