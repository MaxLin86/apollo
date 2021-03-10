// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RDC_LAYER_DESC_H
#define SRS_HDR_RDC_LAYER_DESC_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/devmm.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-errors.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h" // RDC_clutter_image_format, etc
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "cru_enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

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

    StackEnum               sd_pd_store;                    //!< Where to allocate memory for SD and PD
    StackEnum               scan_inst_store;                //!< Where to allocate memory for SI
    StackEnum               scan_output_store;              //!< Where to allocate memory for RDC1
    StackEnum               proc_inst_store;                //!< Where to allocate memory for PI, COVI, and SVDI
    StackEnum               proc_desc_buf_store;            //!< Where to allocate memory for RHAL_ProcDescImpl buffers (STV, FFT window)
    StackEnum               mce_output_store;               //!< Where to allocate memory for MCE

    uint16_t                max_num_scan_desc;
    uint16_t                max_num_scan_gain_mgr;
    uint16_t                max_num_proc_desc;
    uint16_t                max_num_proc_gain_mgr;
    uint16_t                max_num_frame_config;
    uint16_t                max_num_scan_config;
    uint16_t                max_num_scan_instance;

    CLOCK_MODE              initial_power_mode;             //!<
    RDC_Sampling_rate       adc_sample_rate_pod;            //! adc sampling rate: 0:Invalid, 1:1GHz, 2:2GHz
    FLOAT                   carrier_frequency_pod;          //! specify carrier frequency (RF)
    RDC_DAC_Rate            dac_rate_pod;                   //!<
    int8_t                  txu_dll_phase_pod;              //!<
    bool                    disable_angle_variance;         //!< If set, use Angle-DLCR on Static Slice instead of Angle-Variance on Special Activations
    bool                    enable_loio_pa;                 //!< If set, enable LOIO PA (loio0_config__en_pa)
};

SRS_CLOSE_NAMESPACE()

#endif
