#ifndef SRS_HDR_STRING_TABLES_H
#define SRS_HDR_STRING_TABLES_H 1
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

// NOTE: This file should only be included in one location per project. In SRS,
// it is included by rdc/src/cal/CalFlashData.cpp. In pysra, it is included by
// conwrapper.cpp in sabine-radar-api/python/src

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

// Strings for printing calibration keys

SRS_DECLARE_NAMESPACE()

const CHAR* rx_gain_names[] =
{
    // Sabine-A
    "VGAGain_11_15_7_1_1",
    "VGAGain_11_13_0_0_0",
    "VGAGain_7_15_15_1_1",
    "VGAGain_6_15_15_1_1",
    "VGAGain_7_15_13_1_1",
    "VGAGain_7_13_15_1_1",
    "VGAGain_7_15_15_5_1",
    "VGAGain_5_13_13_5_1",
    "VGAGain_4_15_15_5_1",
    "VGAGain_11_15_2_0_0",
    "VGAGain_7_15_9_0_0",
    "VGAGain_1_1_1_1_1",
    "VGAGain_0_15_15_7_7",
    "VGAGain_1_15_15_7_7",
    "VGAGain_2_15_15_7_7",
    "VGAGain_3_15_15_7_7",
    "VGAGain_4_15_15_7_7",
    "VGAGain_5_15_15_7_7",
    "VGAGain_0_15_15_1_7",
    "VGAGain_11_9_0_0_0",
    "VGAGain_11_5_0_0_0",
    "VGAGain_11_1_0_0_0",
    "VGAGain_9_13_0_0_0",
    "VGAGain_9_9_0_0_0",
    "VGAGain_0_15_9_0_0",
    "VGAGain_0_15_5_0_0",
    "VGAGain_0_15_3_0_0",

    // Sabine-B
    // All Sabine-B names MUST have a suffix of "__B"
    "Rx_Gain_51dB_VP_7_1_3_3_3__B",
    "Rx_Gain_41dB_VP_7_0_0_3_3__B",
    "Rx_Gain_35dB_VP_5_0_1_1_3__B",
    "Rx_Gain_31dB_CP_3_0_0_3_1__B",
    "Rx_Gain_28dB_CP_3_1_0_1_1__B",
    "Rx_Gain_38dB_CP_3_1_2_3_3__B"
};

const CHAR* tx_gain_names[] =
{
    // Sabine-A
    "Tx_Gain_4_1",
    "Tx_Gain_1_3",
    "Tx_Gain_7_1",
    "Tx_Gain_1_1",
    "Tx_Gain_2_1",
    "Tx_Gain_3_1",

    // Sabine-B
    // All Sabine-B names MUST have a suffix of "__B"
    "Tx_Gain_Maximum_7_4_44__B",
    "Tx_Gain_Efficient_7_4_4__B",
    "Tx_Gain_Backoff20_0_4_4__B"
};

const CHAR* tx_bw_names[] =
{
    // Sabine-A
    "Tx_BW_00_00_00_00_03_03",
    "Tx_BW_16_16_08_08_02_02",
    "Tx_BW_28_28_12_12_02_02",

    // Sabine-B
    // All Sabine-B names MUST have a suffix of "__B"
    "Tx_BW_1200MHz_1700MHz__B",
    "Tx_BW_700MHz_900MHz__B",
    "Tx_BW_500MHz_600MHz__B",
    "Tx_BW_300MHz_400MHz__B"
};

const CHAR* rx_bw_names[] =
{
    // Sabine-A
    "Rx_BW_c31313131313131_f15151515150707",
    "Rx_BW_c00000000000000_f00000000000000",
    "Rx_BW_c31313122221515_f00000000000000",
    "Rx_BW_c31192222221515_f00000000000000",
    "Rx_BW_c31172022221515_f00000000000000",
    "Rx_BW_c31131622221515_f00000000000000",
    "Rx_BW_c30091314140101_f00000000000000",
    "Rx_BW_c30051014140101_f00000000000000",
    "Rx_BW_c31000514140101_f00000000000000",
    "Rx_BW_c29000514140101_f00000000000000",
    "Rx_BW_c23091314140101_f00000000000000",

    // Sabine-B
    // All Sabine-B names MUST have a suffix of "__B"
    "Rx_BW_2000MHz_Tandem_3_3_3__B",
    "Rx_BW_720MHz_Tandem_2_2_2__B"
};

const CHAR* rdc_preset_descriptions[] =
{
    "VP1a",    "Standard (short range)",
    "VP1as",   "Standard (short range), longer dwell time",
    "VP1b",    "Standard (medium range)",
    "VP1bb",   "Standard (intermediate range)",
    "VP1c",    "Standard (long range)",
    "VP4",     "Long range (better angle res)",
    "VP8",     "Ultra long range",
    "VP9",     "High range res, long range (low duty)",
    "VP9f",    "High range res, long range (medium duty)",
    "VP11",    "Ultra long range, higher velocity",
    "VP12",    "Low velocity (high duty)",
    "VP13",    "Med velocity (high duty)",
    "VP14",    "High velocity",
    "VP14f",   "High velocity, fast scan rate",
    "VP14ff",  "High velocity, faster scan rate",
    "VP15f",   "medium range, medium velocity (country roads), fast scan rate",
    "VP15m",   "medium range, medium velocity (country roads), medium scan rate",
    "VP15s",   "medium range, medium velocity (country roads), slow scan rate",
    "VP16",    "medium range, low velocity (city roads)",
    "CP1a",    "Standard (short range)",
    "CP1as",   "Standard (short range), longer dwell time",
    "CP1b",    "Standard (medium range)",
    "CP1bb",   "Standard (intermediate range)",
    "CP1c",    "Standard (long range)",
    "CP2",     "High range res, (long range, high velocity)",

    "VP101",   "SabineB - VP1a with 256 range bins, 1GHz Sampling Rate",
    "VP201",   "SabineB - VP1a with 256 range bins, 2GHz Sampling Rate",
    "VP102",   "SabineB - City, 1GHz Sampling Rate",
    "VP202",   "SabineB - City, 2GHz Sampling Rate",
    "VP103",   "SabineB - Highway, 1GHz Sampling Rate",
    "VP203",   "SabineB - Highway, 2GHz Sampling Rate",
    "VP104",   "SabineB - Highway-2, Magna POR, 1GHz Sampling Rate",
    "VP105",   "SabineB - City-1, Magna POR, 1GHz Sampling Rate",
    "CP101",   "SabineB - CP1a with 256 range bins, 1GHz Sampling Rate",
    "CP201",   "SabineB - CP1a with 256 range bins, 2GHz Sampling Rate",
    "CP102",   "SabineB - CpCity, 1GHz Sampling Rate",
    "CP202",   "SabineB - CpCity, 2GHz Sampling Rate",
    "CP103",   "SabineB - Highway - CP mode, 1GHz Sampling Rate",
    "CP203",   "SabineB - Highway - CP mode, 2GHz Sampling Rate",
    "CP104",   "SabineB - Basic Scan @ 1G Sampling",
};


// these must be in the exact same order as ThresholdPresetEnum
const CHAR* threshold_preset_descriptions[] =
{
    "low",  "low sensitivity, for indoor use",
    "high", "high sensitivity, for driving",
};

const CHAR* rdc_window_names[] =
{
    "boxcar",       "basic boxcar FFT window (default)",
    "hann",         "Hann FFT window",
    "hamming",      "Hamming FFT window",
    "gauss",        "Gauss FFT window",
    "blackman",     "Blackman FFT window",
    "nuttall",      "Blackman-Nuttall FFT window",
    "harris",       "Blackman-Harris FFT window",
    "chebyshev",    "Chebyshev FFT window (30dB)",
    "remez",        "Remez Channelizer window",
    "taylor30",     "Taylor FFT window (30dB)",
    "chebyshev45",  "Chebyshev FFT window (45dB)",
    "chebyshev60",  "Chebyshev FFT window (60dB)",
    "taylor55",     "Taylor FFT window (55dB)",
    "sva",          "Disable windowing, apply SVA"
};

const CHAR *clutter_image_enum_names[] =
{
    "reserved_0",      "allow RDC layer to use defaults",
    "CIMG_M8C",        "(Unsupported)",
    "CIMG_M8P",        "(Unsupported)",
    "CIMG_M16C",       "16-bit Magnitude Cartesian 2D image (X,Y) (OLD)",
    "CIMG_M16P",       "16-bit Magnitude Polar     2D image (R,A)",
    "CIMG_M16C_D8C",   "16-bit Magnitude Cartesian 2D image (X,Y), with Cartesian Doppler image",
    "CIMG_M16C_D7P",   "16-bit Magnitude Cartesian 2D image (X,Y), with Polar     Doppler image",
    "CIMG_M16P_D7P",   "16-bit Magnitude Polar     2D image (R,A), with Polar     Doppler image",

    "reserved_08", "N/A",
    "reserved_09", "N/A",
    "reserved_10", "N/A",
    "reserved_11", "N/A",
    "reserved_12", "N/A",
    "reserved_13", "N/A",
    "reserved_14", "N/A",
    "reserved_15", "N/A",
    "reserved_16", "N/A",
    "reserved_17", "N/A",
    "reserved_18", "N/A",
    "reserved_19", "N/A",
    "reserved_20", "N/A",

    "CIMG_M16Z",         "16-bit Magnitude Cartesian 3D voxel cuboid (OLD)",
    "CIMG_M16Z_D7P",     "16-bit Magnitude Cartesian 3D voxel cuboid, with Polar Doppler image (Unsupported)",
    "CIMG_M16C_H8C_D8C", "16-bit Magnitude Cartesian, with 8-bit Height and Doppler Polar images",
    "CIMG_M16C_H7P_D7P", "16-bit Magnitude Cartesian, with 7-bit Height and Doppler Polar images (NEW)",
    "CIMG_M16P_H7P_D7P", "16-bit Magnitude Polar,     with 7-bit Height and Doppler Polar images",
    "CIMG_M16PP_D7PP",   "16-bit Magnitude Polar Azimuth and Polar Elevation, with 7-bit Doppler Polar/Polar image",
};

const CHAR* exception_type_strings[] =
{
    "No exception",
    "Undefined Exception",
    "SVC Exception",
    "Prefetch Exception",
    "Abort Exception",
    "Interrupt Triggered Debug",
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_STRING_TABLES_H
