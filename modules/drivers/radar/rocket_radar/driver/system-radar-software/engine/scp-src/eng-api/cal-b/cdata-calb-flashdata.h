// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_CDATA_CALB_FLASHDATA_H
#define SRS_HDR_CDATA_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct CDataCalBLane
{
    struct CdataCalBChan
    {
        uint64_t offset : 8;
        uint64_t c3: 3;
        uint64_t c4: 4;
        uint64_t c5: 5;
        uint64_t c6: 6;
        uint64_t c7: 7;
        uint64_t : 31;
    }
    iq[2];
};

struct CDataCalBDataRaw
{
    CDataCalBLane       cdata[NUM_RX_PER_BANK][NUM_ADC_LANE];

    static void u32_to_bin(uint32_t val, INT bits, CHAR *output)
    {
        INT ch = 0;
        for (INT i = bits-1; i >= 0; i--)
        {
            output[ch++] = (val & (1U << i)) ? '1' : '0';
        }
        output[ch] = 0;
    }

    void uhprint(bool all=true) const
    {
        CHAR i_of[9], q_of[9];
        CHAR i_c3[4], q_c3[4];
        CHAR i_c4[5], q_c4[5];
        CHAR i_c5[6], q_c5[6];
        CHAR i_c6[7], q_c6[7];
        CHAR i_c7[8], q_c7[8];

        for (uint32_t rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            for (uint32_t ln = 0; ln < NUM_ADC_LANE; ln++)
            {
                u32_to_bin(cdata[rx][ln].iq[0].offset, 8, i_of);
                u32_to_bin(cdata[rx][ln].iq[0].c3    , 3, i_c3);
                u32_to_bin(cdata[rx][ln].iq[0].c4    , 4, i_c4);
                u32_to_bin(cdata[rx][ln].iq[0].c5    , 5, i_c5);
                u32_to_bin(cdata[rx][ln].iq[0].c6    , 6, i_c6);
                u32_to_bin(cdata[rx][ln].iq[0].c7    , 7, i_c7);

                u32_to_bin(cdata[rx][ln].iq[1].offset, 8, q_of);
                u32_to_bin(cdata[rx][ln].iq[1].c3    , 3, q_c3);
                u32_to_bin(cdata[rx][ln].iq[1].c4    , 4, q_c4);
                u32_to_bin(cdata[rx][ln].iq[1].c5    , 5, q_c5);
                u32_to_bin(cdata[rx][ln].iq[1].c6    , 6, q_c6);
                u32_to_bin(cdata[rx][ln].iq[1].c7    , 7, q_c7);

                UHPRINTF("RX%1d Lane%1d: I: offset=%s c3=%s c4=%s c5=%s c6=%s c7=%s    Q: offset=%s c3=%s c4=%s c5=%s c6=%s c7=%s\n",
                        rx, ln, i_of, i_c3, i_c4, i_c5, i_c6, i_c7, q_of, q_c3, q_c4, q_c5, q_c6, q_c7);
            }

            if (!all)
                break;
        }
    }

    void set_reg_defaults()
    {
        memset(this, 0U, sizeof(*this));
    }
};

struct CDataCalBData : public CDataCalBDataRaw
{
    uint8_t padding[1460 - sizeof(CDataCalBDataRaw)];

    void set_defaults()
    {
        memset(this, 0xFFU, sizeof(*this));
        set_reg_defaults();
    }
};

struct CDataCalBKey
{
    FLOAT                     temperature;     // Chip temperature C at which this cal was performed
    uint32_t                  adc_sample_rate;

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;

    enum { KEY_VERSION = 1 };

    CDataCalBKey() { memset(this, 0, sizeof(*this)); }

    void uhprint() const
    {
        UHPRINTF("(CDATA) Temperature: %f\n", temperature);
        UHPRINTF("ADC sample rate: %d\n", adc_sample_rate);
    }

#if __SCP__
    void upgrade_from_version(uint32_t, CDataCalBData&) { }
#endif

    //!! This is the all-important key-search algorithm, find an 'exact-match' !!
    bool compare(const CDataCalBKey& other) const
    {
        return adc_sample_rate == other.adc_sample_rate;
    }

    // find the 'nearest-match' by picking the key with the shortest disance
    // from the reference key
    FLOAT distance(const CDataCalBKey& other) const
    {
        return uh_fabsf(temperature - other.temperature) +
               float(UH_ABS(INT(adc_sample_rate) - INT(other.adc_sample_rate)));
    }
};

SRS_CLOSE_NAMESPACE()

#endif
