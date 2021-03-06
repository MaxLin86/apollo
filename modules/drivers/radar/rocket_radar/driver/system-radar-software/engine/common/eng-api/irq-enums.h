#ifndef SRS_HDR_IRQ_ENUMS_H
#define SRS_HDR_IRQ_ENUMS_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

enum ISR_target_cpu
{
    GIC_CCP_TARGET  = 0x01U,
    GIC_HSM_TARGET  = 0x02U,
    GIC_SCP0_TARGET = 0x04U,
    GIC_SCP1_TARGET = 0x08U,
    GIC_DSP0_TARGET = 0x10U,
    GIC_DSP1_TARGET = 0x20U,

    // send interrupt to both (potentially) lockstep cores
    GIC_SCP_TARGET  = (GIC_SCP0_TARGET | GIC_SCP1_TARGET)
};

enum ISR_trigger
{
    LEVEL_TRIGGERED,
    EDGE_TRIGGERED  = 0x2U
};


enum ISR_group
{
    GIC_INT_GROUP_0,
    GIC_INT_GROUP_1
};

enum ISR_target_int
{
    ISR_INT_IRQ,
    ISR_INT_FIQ
};

enum ISR_intr_mode
{
    SECURE,
    NON_SECURE
};

enum ISR_interrupt
{
    // Define the IRQ numbers per GIC-400 Technical Reference Manual (0-31)
    SGI_IRQID0,
    SGI_IRQID1,
    SGI_IRQID2,
    SGI_IRQID3,
    SGI_IRQID4,
    SGI_IRQID5,
    SGI_IRQID6,
    SGI_IRQID7,
    SGI_IRQID8,
    SGI_IRQID9,
    SGI_IRQID10,
    SGI_IRQID11,
    SGI_IRQID12,
    SGI_IRQID13,
    SGI_IRQID14,
    SGI_IRQID15,
    NU_IRQID16,
    NU_IRQID17,
    NU_IRQID18,
    NU_IRQID19,
    NU_IRQID20,
    NU_IRQID21,
    NU_IRQID22,
    NU_IRQID23,
    NU_IRQID24,
    PPI_IRQID25,
    PPI_IRQID26,
    PPI_IRQID27,
    PPI_IRQID28,
    PPI_IRQID29,
    PPI_IRQID30,
    PPI_IRQID31,
    // Shared Peripheral Interrupts based on the order in cpu_ss_top.sv as defined in the wire [159:0] GIC400_IRQS for SABINE_A
    // and [191:0] GIC400_IRQS for SABINE_B
#if SABINE_A
    GIC_SPI_SCP_FS_OKNOK_0,
    GIC_SPI_SCP_FS_OKNOK_1,
    GIC_SPI_TIMERS_INTERRUPT_1,
    GIC_SPI_TIMERS_INTERRUPT_2,
    GIC_SPI_TIMERS_INTERRUPT_3,
    GIC_SPI_TIMERS_INTERRUPT_4,
    GIC_SPI_TIMERS_INTERRUPT_5,
    GIC_SPI_TIMERS_INTERRUPT_6,
    GIC_SPI_TIMERS_INTERRUPT_7,
    GIC_SPI_TIMERS_INTERRUPT_8,
    GIC_SPI_CCP_WDT_WD_IRQ,
    GIC_SPI_ETHERNET_INT,
    GIC_SPI_UART_INT,
    GIC_SPI_FLEXRAY_IRQ,
    GIC_SPI_GPIO_INT,
    GIC_SPI_OSPI_INTERRUPT,
    GIC_SPI_CANFD0_INTR,
    GIC_SPI_CANFD1_INTR,
    GIC_SPI_I2C_1_I2C_IRQ,
    GIC_SPI_I2C_0_I2C_IRQ,
    GIC_SPI_SSPI0_INTERRUPT,
    GIC_SPI_SSPI1_INTERRUPT,
    GIC_SPI_A2J_COMBINED_INTERRUPT,
    GIC_SPI_CONTROLLER_INT,         // DDR
    GIC_SPI_TOP_WDT_WD_IRQ,
    GIC_SPI_ETHERNET_IRQ_0,
    GIC_SPI_ETHERNET_IRQ_1,
    GIC_SPI_ETHERNET_IRQ_2,
    GIC_SPI_ETHERNET_IRQ_3,
    GIC_SPI_ETHERNET_IRQ_4,
    GIC_SPI_ETHERNET_IRQ_5,
    GIC_SPI_ETHERNET_IRQ_6,
    GIC_SPI_ETHERNET_IRQ_7,
    GIC_SPI_H264_IRQ_WD,
    GIC_SPI_H264_IRQ_ER,
    GIC_SPI_H264_IRQ_GP,
    GIC_SPI_IPC_PROC_INT_0,
    GIC_SPI_IPC_PROC_INT_1,
    GIC_SPI_IPC_PROC_INT_2,
    GIC_SPI_IPC_PROC_INT_3,
    GIC_SPI_IPC_PROC_INT_4,
    GIC_SPI_IPC_PROC_INT_5,
    GIC_SPI_IPC_PROC_INT_6,
    GIC_SPI_IPC_PROC_INT_7,
    GIC_SPI_IPC_PROC_INT_8,
    GIC_SPI_IPC_PROC_INT_9,
    GIC_SPI_IPC_PROC_INT_10,
    GIC_SPI_IPC_PROC_INT_11,
    GIC_SPI_IPC_PROC_INT_12,
    GIC_SPI_IPC_PROC_INT_13,
    GIC_SPI_IPC_PROC_INT_14,
    GIC_SPI_IPC_PROC_INT_15,
    GIC_SPI_IPC_PROC_INT_16,
    GIC_SPI_IPC_PROC_INT_17,
    GIC_SPI_IPC_PROC_INT_18,
    GIC_SPI_IPC_PROC_INT_19,
    GIC_SPI_IPC_PROC_INT_20,
    GIC_SPI_IPC_PROC_INT_21,
    GIC_SPI_IPC_PROC_INT_22,
    GIC_SPI_IPC_PROC_INT_23,
    GIC_SPI_IPC_PROC_INT_24,
    GIC_SPI_IPC_PROC_INT_25,
    GIC_SPI_IPC_PROC_INT_26,
    GIC_SPI_IPC_PROC_INT_27,
    GIC_SPI_IPC_PROC_INT_28,
    GIC_SPI_IPC_PROC_INT_29,
    GIC_SPI_IPC_PROC_INT_30,
    GIC_SPI_IPC_PROC_INT_31,
    GIC_SPI_IPC_ACCESS_ERR_INT,
    GIC_SPI_DMA_INT_0,              // 4 DMAC active irqs
    GIC_SPI_DMA_INT_1,
    GIC_SPI_DMA_INT_2,
    GIC_SPI_DMA_INT_3,
    GIC_SPI_UNDEF_105,              // 23 DMAC Not Used irqs
    GIC_SPI_UNDEF_106,
    GIC_SPI_UNDEF_107,
    GIC_SPI_UNDEF_108,
    GIC_SPI_UNDEF_109,
    GIC_SPI_UNDEF_110,
    GIC_SPI_UNDEF_111,
    GIC_SPI_UNDEF_112,
    GIC_SPI_UNDEF_113,
    GIC_SPI_UNDEF_114,
    GIC_SPI_UNDEF_115,
    GIC_SPI_UNDEF_116,
    GIC_SPI_UNDEF_117,
    GIC_SPI_UNDEF_118,
    GIC_SPI_UNDEF_119,
    GIC_SPI_UNDEF_120,
    GIC_SPI_UNDEF_121,
    GIC_SPI_UNDEF_122,
    GIC_SPI_UNDEF_123,
    GIC_SPI_UNDEF_124,
    GIC_SPI_UNDEF_125,
    GIC_SPI_UNDEF_126,
    GIC_SPI_UNDEF_127,
    GIC_SPI_RSP_APS_INT_0,          // rsp_ss (64 lines)
    GIC_SPI_RSP_APS_INT_1,
    GIC_SPI_RSP_APS_INT_2,
    GIC_SPI_RSP_APS_INT_3,
    GIC_SPI_RSP_EFS_INT_0,
    GIC_SPI_RSP_EFS_INT_1,
    GIC_SPI_RSP_EFS_INT_2,
    GIC_SPI_RSP_EFS_INT_3,
    GIC_SPI_RSP_CRU_INT_0,
    GIC_SPI_RSP_CRU_INT_1,
    GIC_SPI_RSP_CRU_INT_2,
    GIC_SPI_RSP_CRU_INT_3,
    GIC_SPI_RSP_FEU_INT_0,
    GIC_SPI_RSP_FEU_INT_1,
    GIC_SPI_RSP_FEU_INT_2,
    GIC_SPI_RSP_FEU_INT_3,
    GIC_SPI_RSP_SEU_INT_0,
    GIC_SPI_RSP_SEU_INT_1,
    GIC_SPI_RSP_SEU_INT_2,
    GIC_SPI_RSP_SEU_INT_3,
    GIC_SPI_RSP_RAU_INT_0,
    GIC_SPI_RSP_RAU_INT_1,
    GIC_SPI_RSP_RAU_INT_2,
    GIC_SPI_RSP_RAU_INT_3,
    GIC_SPI_RSP_DCU_INT_0,
    GIC_SPI_RSP_DCU_INT_1,
    GIC_SPI_RSP_DCU_INT_2,
    GIC_SPI_RSP_DCU_INT_3,
    GIC_SPI_RSP_SCU_INT_0,
    GIC_SPI_RSP_SCU_INT_1,
    GIC_SPI_RSP_SCU_INT_2,
    GIC_SPI_RSP_SCU_INT_3,
    GIC_SPI_RSP_CIU_INT_0,
    GIC_SPI_RSP_CIU_INT_1,
    GIC_SPI_RSP_CIU_INT_2,
    GIC_SPI_RSP_CIU_INT_3,
    GIC_SPI_RSP_QDU_INT_0,
    GIC_SPI_RSP_QDU_INT_1,
    GIC_SPI_RSP_QDU_INT_2,
    GIC_SPI_RSP_QDU_INT_3,
    GIC_SPI_RSP_FCU_INT_0,
    GIC_SPI_RSP_FCU_INT_1,
    GIC_SPI_RSP_FCU_INT_2,
    GIC_SPI_RSP_FCU_INT_3,
    GIC_SPI_RSP_ICU_INT_0,
    GIC_SPI_RSP_ICU_INT_1,
    GIC_SPI_RSP_ICU_INT_2,
    GIC_SPI_RSP_ICU_INT_3,
    GIC_SPI_RSP_RSU_INT_0,
    GIC_SPI_RSP_RSU_INT_1,
    GIC_SPI_RSP_RSU_INT_2,
    GIC_SPI_RSP_RSU_INT_3,
    GIC_SPI_RSP_PGU_INT_0,
    GIC_SPI_RSP_PGU_INT_1,
    GIC_SPI_RSP_PGU_INT_2,
    GIC_SPI_RSP_PGU_INT_3,
    GIC_SPI_RSP_NSU_INT_0,
    GIC_SPI_RSP_NSU_INT_1,
    GIC_SPI_RSP_NSU_INT_2,
    GIC_SPI_RSP_NSU_INT_3,
    GIC_SPI_RSP_MCE_INT_0,
    GIC_SPI_RSP_MCE_INT_1,
    GIC_SPI_RSP_MCE_INT_2,
    GIC_SPI_RSP_MCE_INT_3,

#elif SABINE_B

GIC_SPI_SCP_FS_OKNOK_0,
GIC_SPI_SCP_FS_OKNOK_1,
GIC_SPI_TIMERS_INTERRUPT_1,
GIC_SPI_TIMERS_INTERRUPT_2,
GIC_SPI_TIMERS_INTERRUPT_3,
GIC_SPI_TIMERS_INTERRUPT_4,
GIC_SPI_TIMERS_INTERRUPT_5,
GIC_SPI_TIMERS_INTERRUPT_6,
GIC_SPI_TIMERS_INTERRUPT_7,
GIC_SPI_TIMERS_INTERRUPT_8,
GIC_SPI_CCP_WDT_WD_IRQ,
GIC_SPI_ETHERNET_INT,
GIC_SPI_UART_INT,
GIC_SPI_UART1_INT,
GIC_SPI_GPIO_INT,
GIC_SPI_OSPI_INTERRUPT,
GIC_SPI_CANFD0_INTR,
GIC_SPI_CANFD1_INTR,
GIC_SPI_I2C_1_I2C_IRQ,
GIC_SPI_I2C_0_I2C_IRQ,
GIC_SPI_SSPI0_INTERRUPT,
GIC_SPI_SSPI1_INTERRUPT,
GIC_SPI_A2J_COMBINED_INTERRUPT,
GIC_SPI_CONTROLLER_INT,         // DDR
GIC_SPI_SCP_WDT_WD_IRQ,
GIC_SPI_ETHERNET_IRQ_0,
GIC_SPI_ETHERNET_IRQ_1,
GIC_SPI_ETHERNET_IRQ_2,
GIC_SPI_ETHERNET_IRQ_3,
GIC_SPI_ETHERNET_IRQ_4,
GIC_SPI_ETHERNET_IRQ_5,
GIC_SPI_ETHERNET_IRQ_6,
GIC_SPI_ETHERNET_IRQ_7,
RESERVED_INT_0,
RESERVED_INT_1,
GIC_SPI_DMAC_FAULT_INT_2,
GIC_SPI_IPC_PROC_INT_0,
GIC_SPI_IPC_PROC_INT_1,
GIC_SPI_IPC_PROC_INT_2,
GIC_SPI_IPC_PROC_INT_3,
GIC_SPI_IPC_PROC_INT_4,
GIC_SPI_IPC_PROC_INT_5,
GIC_SPI_IPC_PROC_INT_6,
GIC_SPI_IPC_PROC_INT_7,
GIC_SPI_IPC_PROC_INT_8,
GIC_SPI_IPC_PROC_INT_9,
GIC_SPI_IPC_PROC_INT_10,
GIC_SPI_IPC_PROC_INT_11,
GIC_SPI_IPC_PROC_INT_12,
GIC_SPI_IPC_PROC_INT_13,
GIC_SPI_IPC_PROC_INT_14,
GIC_SPI_IPC_PROC_INT_15,
GIC_SPI_IPC_PROC_INT_16,
GIC_SPI_IPC_PROC_INT_17,
GIC_SPI_IPC_PROC_INT_18,
GIC_SPI_IPC_PROC_INT_19,
GIC_SPI_IPC_PROC_INT_20,
GIC_SPI_IPC_PROC_INT_21,
GIC_SPI_IPC_PROC_INT_22,
GIC_SPI_IPC_PROC_INT_23,
GIC_SPI_IPC_PROC_INT_24,
GIC_SPI_IPC_PROC_INT_25,
GIC_SPI_IPC_PROC_INT_26,
GIC_SPI_IPC_PROC_INT_27,
GIC_SPI_IPC_PROC_INT_28,
GIC_SPI_IPC_PROC_INT_29,
GIC_SPI_IPC_PROC_INT_30,
GIC_SPI_IPC_PROC_INT_31,
GIC_SPI_IPC_ACCESS_ERR_INT,
GIC_SPI_DMAC_INT_0,              // 4 DMAC active irqs
GIC_SPI_DMAC_INT_1,
GIC_SPI_DMAC_INT_2,
GIC_SPI_DMAC_INT_3,
GIC_SPI_SOC_NOC_INT,              //Timeout_Sideband_MainFault
GIC_SPI_HSM_WDT_IRQ_INT,
GIC_SPI_SOC_ATB_MAINFAULT_0_INT,
GIC_SPI_PVT_IRQ_INT,
GIC_SPI_PVT_PD_ALARMS_COMBINED_INT,
GIC_SPI_PVT_VM_ALARMS_COMBINED_INT,
GIC_SPI_PVT_TS_ALARMS_COMBINED_INT,
GIC_SPI_R5CM_SOC_INT_0,
GIC_SPI_R5CM_SOC_INT_1,
GIC_SPI_R5CM_SOC_INT_2,
GIC_SPI_R5CM_SOC_INT_3,
GIC_SPI_FSM_SOC_INT_0,
GIC_SPI_FSM_SOC_INT_1,
GIC_SPI_FSM_SOC_INT_2,
GIC_SPI_FSM_SOC_INT_3,
GIC_SPI_RSP_ACE_INT_0,
GIC_SPI_RSP_ACE_INT_1,
GIC_SPI_RSP_ACE_INT_2,
GIC_SPI_RSP_ACE_INT_3,
GIC_SPI_RSP_HSIO_INT_0,
GIC_SPI_RSP_HSIO_INT_1,
GIC_SPI_RSP_HSIO_INT_2,
GIC_SPI_RSP_HSIO_INT_3,
GIC_SPI_RSP_FEUB_INT_0,          // rsp_ss (64 lines)
GIC_SPI_RSP_FEUB_INT_1,
GIC_SPI_RSP_FEUB_INT_2,
GIC_SPI_RSP_FEUB_INT_3,
GIC_SPI_RSP_EFS_INT_0,
GIC_SPI_RSP_EFS_INT_1,
GIC_SPI_RSP_EFS_INT_2,
GIC_SPI_RSP_EFS_INT_3,
GIC_SPI_RSP_CRU_INT_0,
GIC_SPI_RSP_CRU_INT_1,
GIC_SPI_RSP_CRU_INT_2,
GIC_SPI_RSP_CRU_INT_3,
GIC_SPI_RSP_FEUA_INT_0,
GIC_SPI_RSP_FEUA_INT_1,
GIC_SPI_RSP_FEUA_INT_2,
GIC_SPI_RSP_FEUA_INT_3,
GIC_SPI_RSP_SEU_INT_0,
GIC_SPI_RSP_SEU_INT_1,
GIC_SPI_RSP_SEU_INT_2,
GIC_SPI_RSP_SEU_INT_3,
GIC_SPI_RSP_RAU_INT_0,
GIC_SPI_RSP_RAU_INT_1,
GIC_SPI_RSP_RAU_INT_2,
GIC_SPI_RSP_RAU_INT_3,
GIC_SPI_RSP_DCU_INT_0,
GIC_SPI_RSP_DCU_INT_1,
GIC_SPI_RSP_DCU_INT_2,
GIC_SPI_RSP_DCU_INT_3,
GIC_SPI_RSP_SCU_INT_0,
GIC_SPI_RSP_SCU_INT_1,
GIC_SPI_RSP_SCU_INT_2,
GIC_SPI_RSP_SCU_INT_3,
GIC_SPI_RSP_CIU_INT_0_OR_GIC_SPI_RSP_PTU_INT_0,
GIC_SPI_RSP_CIU_INT_1_OR_GIC_SPI_RSP_PTU_INT_1,
GIC_SPI_RSP_CIU_INT_2_OR_GIC_SPI_RSP_PTU_INT_2,
GIC_SPI_RSP_CIU_INT_3_OR_GIC_SPI_RSP_PTU_INT_3,
GIC_SPI_RSP_QDU_INT_0,
GIC_SPI_RSP_QDU_INT_1,
GIC_SPI_RSP_QDU_INT_2,
GIC_SPI_RSP_QDU_INT_3,
GIC_SPI_RSP_FCU_INT_0,
GIC_SPI_RSP_FCU_INT_1,
GIC_SPI_RSP_FCU_INT_2,
GIC_SPI_RSP_FCU_INT_3,
GIC_SPI_RSP_ICU_INT_0,
GIC_SPI_RSP_ICU_INT_1,
GIC_SPI_RSP_ICU_INT_2,
GIC_SPI_RSP_ICU_INT_3,
GIC_SPI_RSP_RSU_INT_0,
GIC_SPI_RSP_RSU_INT_1,
GIC_SPI_RSP_RSU_INT_2,
GIC_SPI_RSP_RSU_INT_3,
GIC_SPI_RSP_PGU_INT_0_OR_GIC_SPI_RSP_TXUB_INT_0,
GIC_SPI_RSP_PGU_INT_1_OR_GIC_SPI_RSP_TXUB_INT_1,
GIC_SPI_RSP_PGU_INT_2_OR_GIC_SPI_RSP_TXUB_INT_2,
GIC_SPI_RSP_PGU_INT_3_OR_GIC_SPI_RSP_TXUB_INT_3,
GIC_SPI_RSP_NSU_INT_0,
GIC_SPI_RSP_NSU_INT_1,
GIC_SPI_RSP_NSU_INT_2,
GIC_SPI_RSP_NSU_INT_3,
GIC_SPI_RSP_MCE_INT_0,
GIC_SPI_RSP_MCE_INT_1,
GIC_SPI_RSP_MCE_INT_2,
GIC_SPI_RSP_MCE_INT_3,
GIC_SPI_RSP_RAUB_INT_0, /* Only for SabineB.Ref: RTL Code */
GIC_SPI_RSP_RAUB_INT_1,
GIC_SPI_RSP_RAUB_INT_2,
GIC_SPI_RSP_RAUB_INT_3,
GIC_SPI_RSP_SEUB_INT_0,
GIC_SPI_RSP_SEUB_INT_1,
GIC_SPI_RSP_SEUB_INT_2,
GIC_SPI_RSP_SEUB_INT_3,
GIC_SPI_RSP_NSUB_INT_0,
GIC_SPI_RSP_NSUB_INT_1,
GIC_SPI_RSP_NSUB_INT_2,
GIC_SPI_RSP_NSUB_INT_3,
GIC_SPI_HSM_LOCKUP_INT,
GIC_SPI_CCP_LOCKUP_INT,
GIC_SPI_ETHERNET2_INT,
GIC_SPI_ETHERNET1_INT,
GIC_SPI_ETHERNET2_IRQ_0,
GIC_SPI_ETHERNET2_IRQ_1,
GIC_SPI_ETHERNET2_IRQ_2,
GIC_SPI_ETHERNET2_IRQ_3,
GIC_SPI_ETHERNET2_IRQ_4,
GIC_SPI_ETHERNET2_IRQ_5,
GIC_SPI_ETHERNET2_IRQ_6,
GIC_SPI_ETHERNET2_IRQ_7,
GIC_SPI_ETHERNET1_IRQ_0,
GIC_SPI_ETHERNET1_IRQ_1,
GIC_SPI_ETHERNET1_IRQ_2,
GIC_SPI_ETHERNET1_IRQ_3,
GIC_SPI_ETHERNET1_IRQ_4,
GIC_SPI_ETHERNET1_IRQ_5,
GIC_SPI_ETHERNET1_IRQ_6,
GIC_SPI_ETHERNET1_IRQ_7
#endif

};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_IRQ_ENUMS_H
