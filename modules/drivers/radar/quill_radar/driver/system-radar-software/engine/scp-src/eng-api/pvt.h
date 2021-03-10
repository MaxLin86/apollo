#ifndef SRS_HDR_PVT_H
#define SRS_HDR_PVT_H 1
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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

enum PVT_error
{
    PVT_ERC_SUCCESS = 0,
    PVT_ERC_FAILURE,
    PVT_ERC_DEVICE_ID_FAILURE,
    PVT_ERC_SCRATCH_REGISTER_FAILURE,
    PVT_ERC_UNLOCK_ERROR,
    PVT_SIGNATURE_FAILED,
    PVT_SIGNATURE_PASSED
};

enum Shutdown_Source
{
    THERMAL_BREACH,
    VOLTAGE_BREACH,
    PROCESS_BREACH
};

/* PVT running modes */
enum PVT_SamplingMode
{
    PVT_SAMPLE_ONCE,
    PVT_SAMPLE_CONTINOUS
};

/* TS Calibration modes */
enum TS_CalibrationMode
{
    TS_CALIBRATED,
    TS_NON_CALIBRATED
};

/*! alarms */
enum PVT_AlarmInstance
{
    PVT_ALARM_A,
    PVT_ALARM_B,
    PVT_MAX_ALARMS
};

enum SignatureMode
{
    DIRECT_MODE = 0x8U,
    DIRECT_MODE_INVERTED = 0x9U,
    CONVERSION_MODE = 0xAU,
    CONVERSION_MODE_INVERTED = 0xBU,
};

/*  PVT initialisation
    -----------------------------------------------------
    |                                                   |
    -----------------------------------------------------
    |   1.DEVICE ID CONFIRMATION                        |
    |   2.SCRATCH REGISTER ACCESS                       |
    |   3.PVT_REG_LOCK_STATUS                           |
    |   4.PVT_IP_CONFIG                                 |
    |   5.Enabling IRQ for TS,VM & PD to Processor      |
    -----------------------------------------------------
    |   Successful Initialization                       |
    -----------------------------------------------------
*/
PVT_error pvt_init(void);

// Temperature Sensor

/*
   RESOLUTION

   -------------------------------------------------------------
   |    DESCRIPTION    |Resolution(7:5) | Accuracy Degradation |
   -------------------------------------------------------------
   | 12 BIT OPERATION  | 000            |   -                  |
   | 10 BIT OPERATION  | 001            |     (+/-)0.4 C       |
   | 08 BIT OPERATION  | 010            |     (+/-)1.5 C       |
   -------------------------------------------------------------

   Conversion Mode(3:0)

   -------------------------------------------------------------
   |    Mode        | Conv_Mode         | DESCRIPTION          |
   -------------------------------------------------------------
   | Mode 1         | 0000              | A & B (Calibrated)   |
   | Mode 2         | 0001              | G & H (Uncalibrated) |
   -------------------------------------------------------------


   Type(4)

   -----------------------------------------
   | Type           |   DESCRIPTION        |
   -----------------------------------------
   | 0              |  Parallel Output     |
   | 1              |  Serial Output       |
   -----------------------------------------

   -----------------------------------------------------
   |                                                   |
   -----------------------------------------------------
   |    0.Enable TS interrupt in PVT IRQ ENABLE reg.   |
   |    1.HALT TS SDIF INTERFACE                       |
   |    2.Program ip_tmr for TS_SDIF                   |
   |    3.Program ip_cfg for TS_SDIF                   |
   |    4.Program ip_cfga for TS_SDIF(not sure)        |
   |    5.TS HiLo Reset                                |
   -----------------------------------------------------
   |    Successful Configuration                       |
   -----------------------------------------------------
*/
PVT_error PVT_TS_Config(uint8_t Resolution, TS_CalibrationMode Conv_Mode, PVT_SamplingMode mode = PVT_SAMPLE_CONTINOUS);

/*
   Run Mode

   -----------------------------------------
   | Type           |DESCRIPTION           |
   -----------------------------------------
   | 0              |RUN ONCE              |
   | 1              |RUN CONTINOUS         |
   -----------------------------------------

   1. Enable Sample Done IRQ and Fault IRQ in TS
*/
bool PVT_TS_Read(FLOAT& output);

uint32_t TS_Signature_Mode(SignatureMode Mode);
FLOAT pvt_ts_get_last_measurement();

void pvt_ts_disable_shutdown();
void pvt_ts_enable_shutdown();
void pvt_ts_set_shutdown_limit(FLOAT raw_volt, void (*handler)(void*));
void pvt_ts_set_warning_limit(FLOAT raw_volt, void (*handler)(void*));

// Voltage Monitor

enum VM_CHANNEL
{
    PVT_VM_CHANNEL_1,
    PVT_VM_CHANNEL_2,
    PVT_VM_CHANNEL_3,
    PVT_VM_CHANNEL_4,
    PVT_VM_CHANNEL_5,
    PVT_VM_CHANNEL_6,
    PVT_VM_CHANNEL_7,
    PVT_VM_CHANNEL_8,
    PVT_VM_CHANNEL_9,
    PVT_VM_CHANNEL_10,
    PVT_VM_CHANNEL_11,
    PVT_VM_CHANNEL_12,
    PVT_VM_CHANNEL_13,
    PVT_VM_CHANNEL_14,
    PVT_VM_CHANNEL_15,
    PVT_VM_CHANNEL_16,
};

PVT_error PVT_VM_Config(uint8_t Resolution, uint8_t input_sel, PVT_SamplingMode mode);
int32_t  PVT_VM_Read(FLOAT* vmChannelData);
uint32_t VM_Signature_Mode(SignatureMode Mode);

void pvt_vm_disable_shutdown(VM_CHANNEL channel);
void pvt_vm_enable_shutdown(VM_CHANNEL channel);
void pvt_vm_set_shutdown_limit(FLOAT raw_volt, VM_CHANNEL channel, void (*handler)(void*));
void pvt_vm_set_warning_limit(FLOAT raw_volt, VM_CHANNEL channel, void (*handler)(void*));

//PD
//Process detector instances

enum PDinstance
{
    PD_INSTANCE_1,
    PD_INSTANCE_2,
    PD_INSTANCE_MAX
};

/*PD delay chain */
enum PD_PortSelectControl
{
    PD_NO_DELAY_CHAIN,
    PD_BUILT_IN_DELAY_1,
    PD_BUILT_IN_DELAY_2,
    PD_BUILT_IN_DELAY_3,
    PD_BUILT_IN_DELAY_4,
    PD_BUILT_IN_DELAY_5,
    PD_BUILT_IN_DELAY_6,
    PD_EXTERNAL_INPUT,
    PD_MAX_DELAY_CHAINS
};

/* PD Prescalar values */
enum PD_PreScale
{
    PD_PRE_SCALE_VALUE_4,
    PD_PRE_SCALE_VALUE_8,
    PD_PRE_SCALE_VALUE_16,
    PD_PRE_SCALE_VALUE_1
};

/* PD Count Window */
enum PD_CountWindow
{
    PD_COUNT_WIN_SIZE_255,
    PD_COUNT_WIN_SIZE_127,
    PD_COUNT_WIN_SIZE_63,
    PD_COUNT_WIN_SIZE_31
};

PVT_error PVT_PD_Config(PDinstance pdInstance, PD_PortSelectControl portSelect,
                        PD_PreScale preScale, PD_CountWindow countWin, PVT_SamplingMode mode = PVT_SAMPLE_CONTINOUS);
FLOAT PVT_PD_Read(PDinstance pdInstance);
uint32_t PD_Signature_Mode(SignatureMode Mode);

/* ISR functions */
extern "C"
{
    void pvt_ts_alarms_irq_handler(void *arg);
    void pvt_vm_alarms_irq_handler(void *arg);
    void pvt_pd_alarms_irq_handler(void *arg);
}

SRS_CLOSE_NAMESPACE()

#endif // ifndef SRS_HDR_PVT_H
