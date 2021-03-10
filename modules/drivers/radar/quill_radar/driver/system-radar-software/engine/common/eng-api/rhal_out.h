#ifndef SRS_HDR_RHAL_OUT_H
#define SRS_HDR_RHAL_OUT_H 1
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
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

#define DEFAULT_RB_CAL_TX_EARLY_SQUELCH (0x01U)
#define DEFAULT_RB_CAL_RX_DELAY         (67)

#define ADC_CAPTURE_RX 0
#define ADC_CAPTURE_TX 1

SRS_DECLARE_NAMESPACE()

// Pure structures that are output by the RHAL Layer
// This header file can be included on both SCP and DSP

enum                { MAX_VRX           = 96    };  // Per ProcInstance (one TDM sub-scan)

enum                { MAX_ROUGH_ANGLES  = 384   };  // Per ProcInstance (one post-processing pass)

enum                { MAX_TX_PRN        = 12    };  // Max number of simultaneously active TX antennas

enum Multi_Sabine
{
    SABINE_1X    = 1,   // Use 1 sabine chip
    SABINE_2X    = 2,   // Use 2 sabine chips
    SABINE_3X    = 3,   // Use 3 sabine chips
    SABINE_4X    = 4,   // Use 4 sabine chips
    SABINE_COUNT = 4
};

// !enum for Rx Channel bitmap for ADC Capture
enum ADC_RX
{
    ADC_RX0 = 1,
    ADC_RX1 = 2,
    ADC_RX2 = 4,
    ADC_RX3 = 8,
    ADC_RX4 = 16,
    ADC_RX5 = 32,
    ADC_RX6 = 64,
    ADC_RX7 = 128
};

// !enum for Tx Channel bitmap for ADC Capture
enum ADC_TX
{
    ADC_TX0 = 1,
    ADC_TX1 = 2,
    ADC_TX2 = 4,
    ADC_TX3 = 8,
    ADC_TX4 = 16,
    ADC_TX5 = 32,
    ADC_TX6 = 64,
    ADC_TX7 = 128,
    ADC_TX8 = 256,
    ADC_TX9 = 512,
    ADC_TX10 = 1024,
    ADC_TX11 = 2048
};
#if SABINE_B
#elif SABINE_A
enum                { MAX_VRX           = 64    };  // Per ProcInstance (one TDM sub-scan)

enum                { MAX_ROUGH_ANGLES  = 128   };  // Per ProcInstance (one post-processing pass)

enum                { MAX_TX_PRN        = 8     };  // Max number of simultaneously active TX antennas
#endif
enum                { TXUT_DEF_TABLE_SIZE = 198360, TXUT_HEADER_SIZE = 12, TXUT_PREAMBLE_SIZE = 72 , TXUT_TABLE_SIZE = 512};

enum                { MAX_RANGE_BINS    = 512   };  // Max double-pumping the correlator

enum                { MAX_FFT_SIZE      = 2520,     // Currently not allowing the 2520 or 5040 FFT sizes
                      MAX_CHAN_SIZE     = 168   };

enum                { NUM_TX            = 12    };  // Total number of TX antennas

enum                { ALL_TX_MASK       = 0xFFFU };  // Mask of all TX antennas

enum                { NUM_RX_PER_BANK   = 8     };  // Number of RX antennas per bank

enum                { NUM_RX_BANK       = 2     };  // Number of RX antennas bank

enum                { BANK_RX_MASK      = 0xFFU  };   // Mask of all 8 RX antennas in one bank

enum                { ALL_RX_MASK       = 0xFFFFU }; // Mask of all 16 RX antennas (all banks)

enum                { DCU_WORD_SIZE_IN_BYTES  = 8 };    // DCU is 64-bit memory

enum                { MAX_SPL_Q_ENTRIES      = 31 };    // max number of Spl Q entries per PD

enum                { DUMMY_SCAN_VRX = 64, DUMMY_SCAN_LC = 3072, DUMMY_SCAN_R = 128, DUMMY_SCAN_N = 8 };    // Scan Params for dummy scan after ADC capture

enum                { MAX_RESAMPLE_BITMAP_SIZE = 65536 };

enum                { MAX_MEASURE_RANGE_BINS    = 128   };

enum                { MAX_MEASURE_LC    = 8192   };

enum                { MAX_MEASURE_N    = 32   };

enum                { MAX_LC    = 1228800 };  // (1200 * 1024)

enum                { MAX_RX_DELAY = 150 };

enum                { RF_LO_NOM = 16000 };

enum                { RF_NOM = 80000 };

// NOTE: It is possible to extend till 4194304

//! enum for BPSK Mapper mode configuration
enum BPSKMapperMode
{
    MODULATE_I,        //!< MODULATE_I Indicates modulation on I only
    MODULATE_Q,        //!< MODULATE_Q Indicates modulation on Q only
    MODULATE_I_Q,      //!< MODULATE_I_Q Indicates modulation alternately on I then Q
    MODULATE_I_NEG_Q   //!< MODULATE_I_NEG_Q Indicates modulation alternately on I then -Q
};

enum RHAL_RxAntennaBank
{
    RX_ANTENNA_BANK0 = 0,
    RX_ANTENNA_BANK1,
    NUM_RX_BANKS                // 2 Banks
};

enum RHAL_DopplerMode
{
    DOPPLER_FFT,
    DOPPLER_FFT_CHANNELIZER,
    DOPPLER_CHANNELIZER         // Not supported?
};

enum RHAL_DummyScanMode
{
    NO_DUMMY_SCAN      = 0,
    BEFORE_ADC_CAPTURE = 1, // In this case, QDU memory filled with Synthetic Input
    AFTER_ADC_CAPTURE  = 2
};

enum RHAL_MIMOMode
{
    APERIODIC_PRBS = 0,
    PERIODIC_HADAMARD = 2,
    PERIODIC_RANGE_DOMAIN = 3,
    PERIODIC_RANGE_MIMO = 4
};

enum { FEU_NUM_STAGES = 4 };

// Define the valid FFT sizes for Sabine V1 FEU
// Note:
//    * This list MUST be in sorted order (smaller to larger FFT sizes).
//    * This list MUST be kept in sync with RHAL_ProcDescImpl::FEU::feu_int_fft_data[] in feu_init_consts.h
enum RHAL_FFT_SIZE
{
    FFT_SIZE_2 = 0,    // First entry must be 0
    FFT_SIZE_3,
    FFT_SIZE_4,
    FFT_SIZE_5,
    FFT_SIZE_6,
    FFT_SIZE_7,
    FFT_SIZE_8,
    FFT_SIZE_9,
    FFT_SIZE_10,
    FFT_SIZE_12,
    FFT_SIZE_14,
    FFT_SIZE_15,
    FFT_SIZE_16,
    FFT_SIZE_18,
    FFT_SIZE_20,
    FFT_SIZE_21,
    FFT_SIZE_24,
    FFT_SIZE_28,
    FFT_SIZE_30,
    FFT_SIZE_35,
    FFT_SIZE_36,
    FFT_SIZE_40,
    FFT_SIZE_42,
    FFT_SIZE_45,
    FFT_SIZE_48,
    FFT_SIZE_56,
    FFT_SIZE_60,
    FFT_SIZE_63,
    FFT_SIZE_70,
    FFT_SIZE_72,
    FFT_SIZE_80,
    FFT_SIZE_84,
    FFT_SIZE_90,
    FFT_SIZE_105,
    FFT_SIZE_112,
    FFT_SIZE_120,
    FFT_SIZE_126,
    FFT_SIZE_140,
    FFT_SIZE_144,
    FFT_SIZE_168,
    FFT_SIZE_180,
    FFT_SIZE_210,
    FFT_SIZE_240,
    FFT_SIZE_252,
    FFT_SIZE_280,
    FFT_SIZE_315,
    FFT_SIZE_336,
    FFT_SIZE_360,
    FFT_SIZE_420,
    FFT_SIZE_504,
    FFT_SIZE_560,
    FFT_SIZE_630,
    FFT_SIZE_720,
    FFT_SIZE_840,
    FFT_SIZE_1008,
    FFT_SIZE_1260,
    FFT_SIZE_1680,
    FFT_SIZE_2520,
    FFT_SIZE_5040,      // Not allowed
    FFT_SIZE_NUM_SIZES  // Must be last entry
};

extern const CHAR* rx_gain_names[];
enum RHAL_RxVGAGainPresets
{
    // Sabine-A
    VGAGain_11_15_7_1_1,
    VGAGain_11_13_0_0_0,         // Constant Power Mode default
    VGAGain_7_15_15_1_1,
    VGAGain_6_15_15_1_1,
    VGAGain_7_15_13_1_1,
    VGAGain_7_13_15_1_1,
    VGAGain_7_15_15_5_1,
    VGAGain_5_13_13_5_1,
    VGAGain_4_15_15_5_1,
    VGAGain_11_15_2_0_0,
    VGAGain_7_15_9_0_0,
    VGAGain_1_1_1_1_1,
    VGAGain_0_15_15_7_7,
    VGAGain_1_15_15_7_7,
    VGAGain_2_15_15_7_7,
    VGAGain_3_15_15_7_7,
    VGAGain_4_15_15_7_7,
    VGAGain_5_15_15_7_7,
    VGAGain_0_15_15_1_7,         // Variable Power Mode default (low gain VGA1 setting for no humps)
    VGAGain_11_9_0_0_0,
    VGAGain_11_5_0_0_0,
    VGAGain_11_1_0_0_0,
    VGAGain_9_13_0_0_0,
    VGAGain_9_9_0_0_0,
    VGAGain_0_15_9_0_0,
    VGAGain_0_15_5_0_0,
    VGAGain_0_15_3_0_0,

    // Sabine-B
    // From Jama (Dec 3, 2019)
    Rx_Gain_51dB_VP_7_1_3_3_3,
    Rx_Gain_41dB_VP_7_0_0_3_3,      // POR for VP mode
    Rx_Gain_35dB_VP_5_0_1_1_3,
    Rx_Gain_31dB_CP_3_0_0_3_1,
    Rx_Gain_28dB_CP_3_1_0_1_1,      // POR for CP mode
    Rx_Gain_38dB_CP_3_1_2_3_3,      // Spillover and ADC gain calibration

    // New enums go HERE
    NUM_VGAGAIN_PRESETS,

    // Do Not change DEFAULT
    VGAGain_DEFAULT = 255             // force SRS default settings if this is chosen
};

enum RHAL_LOMode
{
    LO_Internal = 0,
    LO_Onboard,
    LO_External,

    // New enums go above this line
    NUM_LO_MODES,

    // Do Not change DEFAULT
    LO_MODE_DEFAULT = 255     // force SRS default settings if this is chosen
};

extern const CHAR* tx_gain_names[];
enum RHAL_TxGainPresets
{
    // Sabine-A
    Tx_Gain_4_1,         // Variable Power Mode default
    Tx_Gain_1_3,         // Constant Power Mode default
    Tx_Gain_7_1,
    Tx_Gain_1_1,
    Tx_Gain_2_1,
    Tx_Gain_3_1,

    // Sabine-B
    // From Jama (Dec 3, 2019)
    Tx_Gain_Maximum_7_4_44,         // POR for VP mode
    Tx_Gain_Efficient_7_4_4,        // POR for CP mode
    Tx_Gain_Backoff20_0_4_4,

    // New enums go HERE
    NUM_TxGAIN_PRESETS,

    // Do Not change DEFAULT
    Tx_Gain_DEFAULT = 255     // force SRS default settings if this is chosen
};

extern const CHAR* tx_bw_names[];
enum RHAL_TxBWPresets    // Enums for RHAL_TxBandWidth
{
    // Sabine-A
    Tx_BW_00_00_00_00_03_03,
    Tx_BW_16_16_08_08_02_02,
    Tx_BW_28_28_12_12_02_02,

    // Sabine-B
    // From Jama (Dec 3, 2019)
    Tx_BW_1200MHz_1700MHz,
    Tx_BW_700MHz_900MHz,            // POR
    Tx_BW_500MHz_600MHz,
    Tx_BW_300MHz_400MHz,

    // New enums go above this line
    NUM_TXBW_PRESETS,

    // Do Not change DEFAULT
    Tx_BW_DEFAULT = 255       // force SRS default settings if this is chosen
};

extern const CHAR* rx_bw_names[];
enum RHAL_RxBWPresets      // Enums for RHAL_RxBandWidth
{
    // Sabine-A
    Rx_BW_c31313131313131_f15151515150707,
    Rx_BW_c00000000000000_f00000000000000,
    Rx_BW_c31313122221515_f00000000000000,
    Rx_BW_c31192222221515_f00000000000000,
    Rx_BW_c31172022221515_f00000000000000,
    Rx_BW_c31131622221515_f00000000000000,
    Rx_BW_c30091314140101_f00000000000000,
    Rx_BW_c30051014140101_f00000000000000,
    Rx_BW_c31000514140101_f00000000000000,
    Rx_BW_c29000514140101_f00000000000000,
    Rx_BW_c23091314140101_f00000000000000,

    // Sabine-B
    // From Jama (Dec 3, 2019)
    Rx_BW_2000MHz_Tandem_3_3_3,              // POR
    Rx_BW_720MHz_Tandem_2_2_2,

    // New enums go above this line
    NUM_RXBW_PRESETS,

    // Do Not change DEFAULT
    Rx_BW_DEFAULT = 255     // force SRS default settings if this is chosen
};

struct RHAL_IQ_CorrRx
{
    // IQ Correction info
    FLOAT       mat_cd[2];
    uint8_t     qilo_xc;
};

struct RHAL_IQ_Corr
{
    RHAL_IQ_CorrRx rx[NUM_RX_PER_BANK];
};

struct FEUIntFftData
{
    uint32_t    fft_size;
    uint8_t     stage_sizes[FEU_NUM_STAGES];        // Stage size, or 1 if stage is disabled
    uint8_t     winograd_stages[FEU_NUM_STAGES];    // Stage index, or 0 if stage is disabled
    const uint32_t    *reordering_indices;
    const uint32_t    *chan_reordering_indices;
};

extern const FEUIntFftData* feu_fft_size_info;

enum RHAL_Fsamp_RSU1Out_Ratio
{
    FSAMP_RSU1OUT_ONE = 1,
    FSAMP_RSU1OUT_TWO = 2,
    FSAMP_RSU1OUT_FOUR = 4,
    FSAMP_RSU1OUT_EIGHT = 8
};
enum RHAL_Fsamp_Fsym_Ratio
{
    FSAMP_FSYMB_ONE = 1,
    FSAMP_FSYMB_TWO = 2,
    FSAMP_FSYMB_FOUR = 4,
    FSAMP_FSYMB_EIGHT = 8
};

enum RHAL_RspClockVal
{
    RHAL_RSP_CLOCK_VAL_2GHZ = 2000,
    RHAL_RSP_CLOCK_VAL_1GHZ = 1000,
    RHAL_RSP_CLOCK_VAL_800MHZ = 800,
    RHAL_RSP_CLOCK_VAL_500MHZ = 500,
    RHAL_RSP_CLOCK_VAL_400MHZ = 400,
    RHAL_RSP_CLOCK_VAL_250MHZ = 250,
    RHAL_RSP_CLOCK_VAL_200MHZ = 200,
    RHAL_RSP_CLOCK_VAL_40MHZ = 40,
};

enum TXUT_ClockVal
{
    TXUT_CLOCKVAL_1GHZ   = 1000,
    TXUT_CLOCKVAL_500MHZ = 500,
    TXUT_CLOCKVAL_250MHZ = 250,
    TXUT_CLOCKVAL_125MHZ = 125,
};

enum RHAL_PeriodicCodeType
{
    CODE_APAS_424,
    CODE_APAS_208,
    //CODE_GOLAY_xxx,
    //CODE_MSEQ_xxx,
    CODE_CUSTOM  // SW provides a ptr to codes for all Txs. HAL does not generatate and manage codes
};

enum RHAL_Thresh_Bucket
{
    LO              = 0,
    HI              = 1,
    NUM_TH_BUCKETS  = 2,         // Do not renumber CUSTOM_RD must equal THRESH_NUM
    CUSTOM          = 2,
    NUM_TH_Q        = 3
};

enum RHAL_Rdc3_Bucket
{
    THRESH_LO_A  = 0,
    THRESH_LO_B  = 1,
    THRESH_HI_A  = 2,
    THRESH_HI_B  = 3,
    THRESH_NUM   = 4,         // Do not renumber CUSTOM_RD must equal THRESH_NUM
    CUSTOM_RD_A  = 4,         // Do not renumber CUSTOM_RD must equal THRESH_NUM
    CUSTOM_RD_B  = 5,         // Do not renumber CUSTOM_RD must equal THRESH_NUM
    NUM_RD_BUF   = 6,         // Do not renumber
};

//! Number of pumps & range-bin combining configuration for Pulsed mode
enum RHAL_ScanModes
{
    VP_INVALID = -1,
    // Add all variable power mode scans here
    VP_SCAN_1 = 0, //!< Near Output 128 Range-Bins for Double pump scans: VP1a, VP1b, VP1bb VP1c
    VP_SCAN_2,     //!< Far Output 256 Range-Bins for Quad pump
    VP_SCAN_3,     //!< Near Output 128 Range-Bins for Quad pump
    VP_SCAN_4,     //!< Near Output 256 Range-Bins for Quad pump scans: VP4, VP12, VP13
    VP_SCAN_5,     //!< Far Output 128 Range-Bins for Triple pump
    VP_SCAN_6,     //!< Far Output 128 combine Range-Bins for Double pump
    VP_SCAN_7,     //!< Far Output 256 Range-Bins for Triple pump
    VP_SCAN_8,     //!< 0-300m Quad pump *VP8
    VP_SCAN_9,     //!< 0-160m Quad pump, higher range res scan: VP9
    VP_SCAN_10,     //!< 0-160m Quad pump, high velocity scan: VP11
    VP_SCAN_11,     //!< 0-300m Quad pump, high velocity - Autobahn scan
    VP_SCAN_12,     //!< 0-238m Quad pump, max 55kmph - 20 fps scan
    VP_SCAN_13,     //!< 0-160m Quad pump, max 77kmph
    VP_SCAN_14,     //!< 0-317m Quad pump, Max ~200kmph, uniform range bin combine - New Autobahn scan: VP14
    VP_SCAN_14f,     //!< 0-230m Quad pump, Max ~200kmph, uniform range bin combine - New Autobahn scan
    NUM_VP_SCAN_MODES,
    // Add all non variable power mode scans here
    NVP_SCAN_1,
    TOTAL_SCAN_MODES
};

enum RHAL_ScanWaveModes
{
    SCAN_NON_VP_MODE = 0,  //!< Continuous mode
    SCAN_VP_MODE = 1,      //!< Variable Power moe
    NUM_SCAN_MODES
};

enum RHAL_ModulationModes
{
    SCAN_BPSK = 0,        //!< BPSK
    SCAN_UMSK = 1,        //!< UMSK
    NUM_MODULATION_MODES
};

enum RHAL_RSU1_Output_Rate
{
    RSU1_OUTPUT_RATE_2G   = 2000,
    RSU1_OUTPUT_RATE_1G   = 1000,
    RSU1_OUTPUT_RATE_500M = 500,
    RSU1_OUTPUT_RATE_250M = 250,
    RSU1_OUTPUT_RATE_125M = 125,
    NUM_RSU1_OUTPUT_RATES = 5
};

enum RHAL_RSU1_Chiprate_Ratio
{
    RSU2_RATIO_ONE   = 1,
    RSU2_RATIO_TWO   = 2,
    RSU2_RATIO_THREE = 3,
    RSU2_RATIO_FOUR  = 4,
    RSU2_RATIO_FIVE  = 5,
    RSU2_RATIO_SIX   = 6,
    RSU2_RATIO_SEVEN = 7,
    RSU2_RATIO_EIGHT = 8,
};

enum RHAL_ChipRates
{
    CHIP_RATE_62_5      = 63,
    CHIP_RATE_71_4      = 71,
    CHIP_RATE_83_3      = 83,
    CHIP_RATE_100       = 100,
    CHIP_RATE_125       = 125,
    CHIP_RATE_142_8     = 143,
    CHIP_RATE_166_7     = 167,
    CHIP_RATE_200       = 200,
    CHIP_RATE_250       = 250,
    CHIP_RATE_333       = 333,
    CHIP_RATE_500       = 500,
    CHIP_RATE_666_7     = 667,
    CHIP_RATE_1000      = 1000,
    CHIP_RATE_2000      = 2000,
    NUM_CHIP_RATES
};

enum RHAL_HW_SVA_NyquistRate
{
    HW_SVA_NYQ_RATE_INVALID = 0,
    HW_SVA_NYQ_RATE_1 = 1,
    HW_SVA_NYQ_RATE_2 = 2,
    HW_SVA_NYQ_RATE_3 = 3,
    HW_SVA_NYQ_RATE_4 = 4,
    MAX_HW_SVA_VERT_NYQ_RATE = 4,   //! Maximum vertical Nyquist rate for HW-SVA
    HW_SVA_NYQ_RATE_5 = 5,
    HW_SVA_NYQ_RATE_6 = 6,
    HW_SVA_NYQ_RATE_7 = 7,
    HW_SVA_NYQ_RATE_8 = 8,
    MAX_HW_SVA_HORZ_NYQ_RATE = 8,   //! Maximum horizontal Nyquist rate for HW-SVA
    NUM_HW_SVA_NYQ_RATES
};


enum RHAL_Supported_Angles
{
    MAX_REVA_ROUGH_ANGLES = 128,
    MAX_REVB_ROUGH_ANGLES = 192,
    MAX_REVB_ROUGH_ANGLES_DUAL_PASS = 384,
};

struct RHAL_TxAntennaEn
{
    uint32_t Tx0_en  : 1;
    uint32_t Tx1_en  : 1;
    uint32_t Tx2_en  : 1;
    uint32_t Tx3_en  : 1;
    uint32_t Tx4_en  : 1;
    uint32_t Tx5_en  : 1;
    uint32_t Tx6_en  : 1;
    uint32_t Tx7_en  : 1;
    uint32_t Tx8_en  : 1;
    uint32_t Tx9_en  : 1;
    uint32_t Tx10_en : 1;
    uint32_t Tx11_en : 1;
    uint32_t res     : 20;

    uint32_t    get_antenna_map() const
    {
        return *reinterpret_cast<const uint32_t*>(this) & 0x00000FFFUL;
    }

    uint32_t    get_antenna_map_slow() const
    {
        return (uint32_t)(Tx0_en)
               + (uint32_t)(Tx1_en << 1)
               + (uint32_t)(Tx2_en << 2)
               + (uint32_t)(Tx3_en << 3)
               + (uint32_t)(Tx4_en << 4)
               + (uint32_t)(Tx5_en << 5)
               + (uint32_t)(Tx6_en << 6)
               + (uint32_t)(Tx7_en << 7)
               + (uint32_t)(Tx8_en << 8)
               + (uint32_t)(Tx9_en << 9)
               + (uint32_t)(Tx10_en << 10)
               + (uint32_t)(Tx11_en << 11);
    }


    void set_antenna_map(uint32_t tx_map)
    {
        Tx0_en = (tx_map & (1 << 0)) >> 0;
        Tx1_en = (tx_map & (1 << 1)) >> 1;
        Tx2_en = (tx_map & (1 << 2)) >> 2;
        Tx3_en = (tx_map & (1 << 3)) >> 3;
        Tx4_en = (tx_map & (1 << 4)) >> 4;
        Tx5_en = (tx_map & (1 << 5)) >> 5;
        Tx6_en = (tx_map & (1 << 6)) >> 6;
        Tx7_en = (tx_map & (1 << 7)) >> 7;
        Tx8_en = (tx_map & (1 << 8)) >> 8;
        Tx9_en = (tx_map & (1 << 9)) >> 9;
        Tx10_en = (tx_map & (1 << 10)) >> 10;
        Tx11_en = (tx_map & (1 << 11)) >> 11;
        res = 0;
    }
};

struct RHAL_RxAntennaEn
{
    // Azimuth Rxs
    uint32_t Rx_00_en  : 1;
    uint32_t Rx_10_en  : 1;
    uint32_t Rx_20_en  : 1;
    uint32_t Rx_30_en  : 1;
    uint32_t Rx_40_en  : 1;
    uint32_t Rx_50_en  : 1;
    uint32_t Rx_60_en  : 1;
    uint32_t Rx_70_en  : 1;

    // Elevation Rxs
    uint32_t Rx_01_en  : 1;
    uint32_t Rx_11_en  : 1;
    uint32_t Rx_21_en  : 1;
    uint32_t Rx_31_en  : 1;
    uint32_t Rx_41_en  : 1;
    uint32_t Rx_51_en  : 1;
    uint32_t Rx_61_en  : 1;
    uint32_t Rx_71_en  : 1;

    uint32_t res : 16;

    uint32_t    get_active_rx_channels() const
    {
        uint32_t bits = *reinterpret_cast<const uint32_t*>(this) & 0x0000FFFFUL;
        return ((bits & 0xFFUL) | ((bits >> 8) & 0xFFUL));
    }

    uint32_t    get_antenna_map() const
    {
        return *reinterpret_cast<const uint32_t*>(this) & 0x0000FFFFUL;
    }

    uint32_t    get_antenna_map_slow() const
    {
        return (uint32_t)(Rx_00_en)
               + (uint32_t)(Rx_10_en << 1)
               + (uint32_t)(Rx_20_en << 2)
               + (uint32_t)(Rx_30_en << 3)
               + (uint32_t)(Rx_40_en << 4)
               + (uint32_t)(Rx_50_en << 5)
               + (uint32_t)(Rx_60_en << 6)
               + (uint32_t)(Rx_70_en << 7)
               + (uint32_t)(Rx_01_en << 8)
               + (uint32_t)(Rx_11_en << 9)
               + (uint32_t)(Rx_21_en << 10)
               + (uint32_t)(Rx_31_en << 11)
               + (uint32_t)(Rx_41_en << 12)
               + (uint32_t)(Rx_51_en << 13)
               + (uint32_t)(Rx_61_en << 14)
               + (uint32_t)(Rx_71_en << 15);
    }


    void set_antenna_map(uint32_t rx_map)
    {
        Rx_00_en = (rx_map & (1 << 0)) >> 0;
        Rx_10_en = (rx_map & (1 << 1)) >> 1;
        Rx_20_en = (rx_map & (1 << 2)) >> 2;
        Rx_30_en = (rx_map & (1 << 3)) >> 3;
        Rx_40_en = (rx_map & (1 << 4)) >> 4;
        Rx_50_en = (rx_map & (1 << 5)) >> 5;
        Rx_60_en = (rx_map & (1 << 6)) >> 6;
        Rx_70_en = (rx_map & (1 << 7)) >> 7;
        Rx_01_en = (rx_map & (1 << 8)) >> 8;
        Rx_11_en = (rx_map & (1 << 9)) >> 9;
        Rx_21_en = (rx_map & (1 << 10)) >> 10;
        Rx_31_en = (rx_map & (1 << 11)) >> 11;
        Rx_41_en = (rx_map & (1 << 12)) >> 12;
        Rx_51_en = (rx_map & (1 << 13)) >> 13;
        Rx_61_en = (rx_map & (1 << 14)) >> 14;
        Rx_71_en = (rx_map & (1 << 15)) >> 15;
        res = 0;
    }
};

struct RHAL_AntennaCtrl
{
    RHAL_TxAntennaEn        tx_enable;                // bitmap of POWERED-ON Tx antennas
    RHAL_RxAntennaEn        rx_enable;                // bitmap of enabled Rx antennas (Sabine supports only Max 8)
    int8_t                  prn_idx[NUM_TX];          // PRN id for each of the 12 Tx antennas (0-MAX_TX_PRN-1 selects PRN source, MAX_TX_PRN=disabled)   (in HW-12 order)
    uint8_t                 tx_symbol_delay[NUM_TX];  // Number of symbols to delay Tx before modulator:  CONTROL_TXnn : symbol_delay      (in HW-12 order)
    uint8_t                 tx_sub_sym_delay[NUM_TX]; // Number of sub-symbols (1/4 symbol) to delay Tx:  CONTROL_TXnn : subsymbol_delay   (in HW-12 order)
    vec3f_t                 pos_offset;               // Offset (in meters) from antenna positions (to center virtual array)

    uint8_t                 pa_tx_num_centers;              // Phased Array:  number of Tx phase centers (0 if PA disabled)
    vec3f_t                 pa_tx_centers[MAX_TX_PRN];      // Phased Array:  positions of Tx phase centers (in Y-major order)

    void from_env_antenna_ctrl(struct AntennaCtrl);

    bool is_phased_array_enabled() const
    {
        return pa_tx_num_centers > 0;
    }

    bool compare_antenna_config(const RHAL_AntennaCtrl& as2) const
    {
        return (tx_enable.get_antenna_map() == as2.tx_enable.get_antenna_map()) &&
               (rx_enable.get_antenna_map() == as2.rx_enable.get_antenna_map());
    }

    int8_t get_bank_num(uint8_t rx) const
    {
        uint32_t rx_en = rx_enable.get_antenna_map();
        uint8_t rx_bank0 = rx_en & 0xFFU;
        uint8_t rx_bank1 = (rx_en >> NUM_RX_PER_BANK) & 0xFFU;

        if ((rx_bank0 >> rx) & 0x01U)
        {
            return 0;
        }
        else if ((rx_bank1 >> rx) & 0x01U)
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }
};

struct RHAL_RDC1Info
{
    cint16   *rdc1;
    uint16_t *rdc1_exponents;
    uint32_t  rdc1_size_bytes;
    uint32_t  rdc1exp_size_bytes;
    bool      rdc1_stride_en;
};

struct RHAL_PeriodicCustomCode
{
    uint32_t periodic_custom_Lc;
    uint8_t *code;
};

struct RHAL_TxAnalogGains
{
    uint8_t  tx_gain;                 //! values 1 .. 7 where 7 is highest
    uint8_t  tx_power_mode;           //! enum TxPowerMode 1 .. 3 where 1 is highest
};

struct RHAL_RxAnalogGains
{
    uint8_t  rx_vga[4];               //! gain_vga1 .. gain_vga4
    uint8_t  rx_bqf;                  //! gain_bqf
};

struct RHAL_TxBandWidth
{
    uint8_t             bbc1i;
    uint8_t             bbc1q;
    uint8_t             bbc2i;
    uint8_t             bbc2q;
    uint8_t             bbgm1;
    uint8_t             bbgm2;
};

struct RHAL_RxBandWidth
{
    uint8_t bw_vga1_c;
    uint8_t bw_vga2_c;
    uint8_t bw_vga3_c;
    uint8_t ibw_bqf_s1_c;
    uint8_t qbw_bqf_s1_c;
    uint8_t ibw_bqf_s2_c;
    uint8_t qbw_bqf_s2_c;
    uint8_t bw_vga1_f;
    uint8_t bw_vga2_f;
    uint8_t bw_vga3_f;
    uint8_t ibw_bqf_s1_f;
    uint8_t qbw_bqf_s1_f;
    uint8_t ibw_bqf_s2_f;
    uint8_t qbw_bqf_s2_f;
};

enum RHAL_Tx_Num
{
    TX_0 = 0,
    TX_1,
    TX_2,
    TX_3,
    TX_4,
    TX_5,
    TX_6,
    TX_7,
    TX_8,
    TX_9,
    TX_10,
    TX_11,

    RHAL_NUM_TX
};

enum RHAL_Rx_Num
{
    RX_0 = 0,
    RX_1,
    RX_2,
    RX_3,
    RX_4,
    RX_5,
    RX_6,
    RX_7,

    RHAL_NUM_RX
};

enum Aprobe_Madc_GroupId
{
    APROBE_INVALID_GROUP = -1,
    APROBE_TX = 0,
    APROBE_SH,
    APROBE_RX,
    NUM_MADC_GROUPS
};

struct RHAL_DProbe_IQ
{
    uint32_t mag;
    uint32_t phase;
};

struct RHAL_DProbe_RMS
{
    uint32_t rms_val;
};

struct RHAL_DProbe_Corr
{
    uint32_t corr_i[7];
    uint32_t corr_q[7];
};

struct RHAL_DProbe_Cov
{
    uint32_t cov_i[7];
    uint32_t cov_q[7];
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RHAL_OUT_H
