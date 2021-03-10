// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_PKDET_CALB_FLASHDATA_H
#define SRS_HDR_PKDET_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

enum PeakDetReturnCode
{
    PEAK_DET_CALIBRATION_SUCCESS = 0,
    PEAK_DET_ERR_REQ_VALUE_NOT_SPECIFIED = 1,
    PEAK_DET_ERR_CHANNEL_OUT_OF_RANGE = 2,
    PEAK_DET_ERR_START_FAILURE = 3,
    PEAK_DET_NOT_REQUESTED = 0xFFU
};

struct PeakDetCalData
{
    enum { NUM_PEAK_DETECT_VALUES = 8 };

    //FLOAT values[NUM_TX][PD_NUM_DETECTORS][NUM_PEAK_DETECT_VALUES];
    FLOAT values[NUM_TX][NUM_PEAK_DETECT_VALUES][PD_NUM_DETECTORS];

    // reserve room for future RX peak detectors.. not used in current chips
    FLOAT rxvalues[NUM_RX_PER_BANK][PD_NUM_DETECTORS][NUM_PEAK_DETECT_VALUES];

    void set_reg_defaults()
    {
        memset(this, 0U, sizeof(*this));
    }

    void uhprint(bool all=true) const
    {
        for (INT tx = 0; tx < NUM_TX; tx++)
        {
            UHPRINTF("Peak Detector constants for TX%d\n", tx);
            UHPRINTF("CAL_DC: %.3f\n",     values[tx][0][PD_CAL_DC]);
            UHPRINTF("PA120_CAL: %.3f\n",  values[tx][0][PD_PA120_CAL]);
            UHPRINTF("PA60_CAL: %.3f\n",  values[tx][0][PD_PA60_CAL]);
            UHPRINTF("PA30_CAL: %.3f\n",  values[tx][0][PD_PA30_CAL]);
            UHPRINTF("RFMX_CAL: %.3f\n",  values[tx][0][PD_RFMX_CAL]);
            UHPRINTF("LOIOPK: %.3f\n",  values[tx][0][PD_LOIOPK]);
            UHPRINTF("LOQOPK: %.3f\n",  values[tx][0][PD_LOQOPK]);
            UHPRINTF("RFLPBK: %.3f\n",  values[tx][0][PD_RFLPBK]);

            if (!all)
                break;
        }
    }

    bool check_cal_status() { return true; }

private:

};


struct PeakDetCalBKey
{
    FLOAT                     temperature;     // Chip temperature C at which this cal was performed

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;
    uint32_t                  reserved_2;

    enum { KEY_VERSION = 1 };

    PeakDetCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(PEAKDET) Temperature: %f\n", temperature);
    }

    void upgrade_from_version(uint32_t, PeakDetCalData&) { }

    bool compare(const PeakDetCalBKey&) const
    {
        return true;
    }

    FLOAT distance(const PeakDetCalBKey& other) const
    {
        return uh_fabsf(temperature - other.temperature);
    }
};

SRS_CLOSE_NAMESPACE()

#endif
