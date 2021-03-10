#ifndef SRS_HDR_RDC_LAYER_DESC_H
#define SRS_HDR_RDC_LAYER_DESC_H 1
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
/*! \file */

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/devmm.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-errors.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" // RDC_clutter_image_format, etc
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "cru_enums.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#if SABINE_A
#include "ldo-cal-flashdata.h"
#include "qilo-cal-flashdata.h"
#include "temp-cal-flashdata.h"
#include "vtr-cal-flashdata.h"
#include "tcs-cal-flashdata.h"
#endif

SRS_DECLARE_NAMESPACE()

//
//! Descriptor containing parameters passed to RDC_Layer::init()
//
struct RDC_LayerDesc
{
public:

    StackEnum               fc_stack;                       //!< Where to allocate memory RDC_FrameConfig
    StackEnum               sc_stack;                       //!< Where to allocate memory RDC_ScanConfig
    StackEnum               sc_cached_stack;                //!< Where to allocate memory RDC_SC_Cached
    StackEnum               rsi_stack;                      //!< Where to allocate memory RDC_ScanInstance
    StackEnum               detections_stack;               //!< Where to allocate memory DetectionData
    StackEnum               clutter_image_stack;            //!< Where to allocate memory RDC_ClutterImage
    StackEnum               scratchpad_stack;               //!< Where to allocate memory for malloca type uses

    StackEnum               sd_pd_store;                    //!< Where to allocate memory SD and PD
    StackEnum               scan_inst_store;                //!< Where to allocate memory SI
    StackEnum               scan_output_store;              //!< Where to allocate memory for RDC1
    StackEnum               proc_inst_store;                //!< Where to allocate memory PI, COVI, and SVDI
    StackEnum               chan_output_store;              //!< Where to allocate memory for Channelizer RDC2
    StackEnum               proc_output_store;              //!< Where to allocate memory for RDC3, Static Slice, Histograms
    StackEnum               proc_desc_buf_store;            //!< Where to allocate memory for RHAL_ProcDescImpl buffers (STV, FFT window)
    StackEnum               proc_ss_output_store;           //!< Where to allocate memory Static Slice
    StackEnum               mce_output_store;               //!< Where to allocate memory for MCE

    RHAL_LOMode             ext_lo_enable_flags;            //!<

    uint16_t                max_num_scan_desc;
    uint16_t                max_num_scan_gain_mgr;
    uint16_t                max_num_proc_desc;
    uint16_t                max_num_proc_gain_mgr;

#if SABINE_A
    RDC_Sampling_rate       adc_sample_rate;
    FLOAT                   carrier_frequency;
    INT                     adc_ldo_code;

    QiloCalData             initial_qilo_dcap;
    LDOCalData              initial_ldo_config;
    VTRCalData              initial_vtr_config;
    TempCalData             temperature_table;
    TCSCalData              initial_tx_carrier_suppression;
#elif SABINE_B
    CLOCK_MODE              initial_power_mode;             //!<
    RDC_Sampling_rate       adc_sample_rate_pod;            //! adc sampling rate: 0:Invalid, 1:1GHz, 2:2GHz
    FLOAT                   carrier_frequency_pod;          //! specify carrier frequency (RF)
    RDC_DAC_Rate            dac_rate_pod;                   //!<
    int8_t                  txu_dll_phase_pod;              //!<
#endif
};

SRS_CLOSE_NAMESPACE()

#endif
