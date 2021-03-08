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

#ifndef SRS_HDR_SABINE_A_CONFIG_H
#define SRS_HDR_SABINE_A_CONFIG_H 1

#define TCMA_START_ADDRESS            0x00000000
#define TCMA_SIZE                     (64 * 1024)
#define TCMA_RESERVED_SIZE            2048

#define TCMB_START_ADDRESS            0x10000000
#define TCMB_SIZE                     (128 * 1024)
#define TCMB_SCP_STACK_SIZE           0x3400

#define SYS_RAM_START_ADDRESS         0x20000000
#define SCP_SRAM_CODE_START_ADDRESS   0x20040000
#define SYS_SRAM_SIZE                 (1024 * 1024)

#define DRAM_START_ADDRESS            0x60000000
#define DRAM_SIZE                     (256 * 1024 * 1024)

#define DCU_START_ADDRESS             0x80000000
#define DCU_SIZE                      (4 * 1024 * 1024)

// Reserve some DDR for code
#define DRAM_P5_RESERVED_SIZE         (128 * 1024)
#define DRAM_SCP_RESERVED_SIZE        (512 * 1024)
#define DRAM_SCP_RESERVED_START_ADDR  (DRAM_START_ADDRESS + DRAM_P5_RESERVED_SIZE)
#define DRAM_USER_START_ADDRESS       (DRAM_SCP_RESERVED_START_ADDR + DRAM_SCP_RESERVED_SIZE)

#define EXCEPTION_VECTOR_START        0X0UL
#define EXCEPTION_PAGE_SIZE           0x400UL

#define IRQ_STACK_SIZE                (4 * 1024)

#define EXCEPTION_STACK               0x20100000
#define EXCEPTION_STACK_SIZE          0x200 //512 bytes

/* Exception dump starts at */
#define EXCEPTION_REGION              (EXCEPTION_STACK - EXCEPTION_STACK_SIZE)
#define EXCEPTION_DUMP_REGION_SIZE    0x200 //512 bytes
#define EXCEPTION_DUMP_MAGIC          (EXCEPTION_REGION)
#define EXCEPTION_DUMP_TYPE           (EXCEPTION_DUMP_MAGIC + 4)
#define EXCEPTION_EXCEPTION_DUMP_ADDR (EXCEPTION_DUMP_TYPE + 4)
#define EXCEPTION_REG_DUMP            (EXCEPTION_EXCEPTION_DUMP_ADDR + 4)

#endif // SRS_HDR_SABINE_A_CONFIG_H
