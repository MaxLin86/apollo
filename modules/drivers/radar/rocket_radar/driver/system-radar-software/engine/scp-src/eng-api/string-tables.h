// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_STRING_TABLES_H
#define SRS_HDR_STRING_TABLES_H 1

// NOTE: This file should only be included in one location per project. In SRS,
// it is included by rdc/src/cal/CalFlashData.cpp. In pysra, it is included by
// conwrapper.cpp in sabine-radar-api/python/src

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

// Strings for printing calibration keys

SRS_DECLARE_NAMESPACE()

const CHAR* rx_gain_names[] =
{
    // Sabine-A
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",

    // Sabine-B
    "Rx_Gain_51dB_VP_7_1_3_3_3",
    "Rx_Gain_41dB_VP_7_0_0_3_3",
    "Rx_Gain_35dB_VP_5_0_1_1_3",
    "Rx_Gain_31dB_CP_3_0_0_3_1",
    "Rx_Gain_28dB_CP_3_1_0_1_1",
    "Rx_Gain_38dB_CP_3_1_2_3_3"
};

const CHAR* tx_gain_names[] =
{
    // Sabine-A
    "",
    "",
    "",
    "",
    "",
    "",

    // Sabine-B
    "Tx_Gain_Maximum_7_4_44",
    "Tx_Gain_Efficient_7_4_4",
    "Tx_Gain_Backoff20_0_4_4"
};

const CHAR* tx_bw_names[] =
{
    // Sabine-A
    "",
    "",
    "",

    // Sabine-B
    "Tx_BW_1200MHz_1700MHz",
    "Tx_BW_700MHz_900MHz",
    "Tx_BW_500MHz_600MHz",
    "Tx_BW_300MHz_400MHz"
};

const CHAR* rx_bw_names[] =
{
    // Sabine-A
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",

    // Sabine-B
    "Rx_BW_2000MHz_Tandem_3_3_3",
    "Rx_BW_720MHz_Tandem_2_2_2"
};

const CHAR* rdc_preset_descriptions[] =
{
    "VP101",   "VP1a with 256 range bins, 1GHz Sampling Rate",
    "VP201",   "VP1a with 256 range bins, 2GHz Sampling Rate",
    "VP102",   "City, 1GHz Sampling Rate",
    "VP202",   "City, 2GHz Sampling Rate",
    "VP103",   "Highway, 1GHz Sampling Rate",
    "VP203",   "Highway, 2GHz Sampling Rate",
    "VP104",   "Highway-2, 1GHz Sampling Rate",
    "VP105",   "City-1, 1GHz Sampling Rate",
    "VP211",   "1G chip rate scan",
    "CP101",   "CP1a with 256 range bins, 1GHz Sampling Rate",
    "CP201",   "CP1a with 256 range bins, 2GHz Sampling Rate",
    "CP102",   "CpCity, 1GHz Sampling Rate",
    "CP202",   "CpCity, 2GHz Sampling Rate",
    "CP103",   "Highway - CP mode, 1GHz Sampling Rate",
    "CP203",   "Highway - CP mode, 2GHz Sampling Rate",
    "CP104",   "Basic Scan @ 1G Sampling",
    "CP210",   "1G chip rate scan",
    "VP114",   "Like VP101 but fewer pings per PRI (11/15)",
    "VP114U",  "Like VP114 but correlating all pings per PRI (15/15)",
    "VP104N",  "VP104 with support for nuking",
    "VP105N",  "VP105 with support for nuking",
    "VP204",   "SabineB - Highway-2, 2GHz Sampling Rate",
    "VP205",   "SabineB - City-1, 2GHz Sampling Rate",
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
