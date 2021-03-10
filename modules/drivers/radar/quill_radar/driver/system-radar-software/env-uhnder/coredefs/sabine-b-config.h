// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2017 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE
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
#define SCP_SRAM_CODE_START_ADDRESS   0x11050000
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
