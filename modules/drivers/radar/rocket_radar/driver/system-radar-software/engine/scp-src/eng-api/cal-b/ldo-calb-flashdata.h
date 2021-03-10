// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_LDO_CALB_FLASHDATA_H
#define SRS_HDR_LDO_CALB_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

enum LdoReturnCode
{
    LDO_CALIBRATION_SUCCESS             = 0,
    ERROR_REQUIRED_VALUE_NOT_SPECIFIED  = 1,
    ERROR_INVALID_BUS_NAME              = 2,
    ERROR_CHANNEL_OUT_OF_RANGE          = 3,
    ERROR_APROBE_NOT_INITIALIZED        = 4,
    ERROR_OUTSIDE_OF_SAFETY_LIMITS      = 5,
    ERROR_CONTROL_DIRECTION_ERROR       = 6,
    ERROR_OSCILLATION_STATE             = 7,
    ERROR_OUTPUT_DOESNT_CHANGE          = 8,
    ERROR_INPUT_SUPPLY_LOW              = 9,
    ERROR_START_FAILURE                 = 10,
    NOT_REQUESTED                       = 0xFFU
};

struct LDOCalBDataRaw
{
    // TX LDOs
    uint8_t dacs_generic__vreg0p9_ref_ctrl[NUM_TX];             // TX DAC 0p9
    uint8_t dacs_config__curr_dac0[NUM_TX];                     // TX DAC 1p5
    uint8_t synctx0_config__cur_dac[NUM_TX];                    // TX sync 0p9
    uint8_t bbca0_generic__v0p9_ctrl[NUM_TX];                   // TX bbca 0p9
    uint8_t cfg_config__qiloldo_ref_dac[NUM_TX];                // TX qilo 0p9
    uint8_t txfe2_config__vtr_ldotxdac[NUM_TX];                 // TX pa3060 0p9
    uint8_t txfe2_generic__vtr_ldo_padac[NUM_TX];               // TX pa120 0p9
    uint8_t txfe1_config__vtr_ldo1p5txdac[NUM_TX];               // TX pa3060 1p5

    // SH Section LDOs
    uint8_t xref0_config__ztr_code;                             // XREF - XVTR calibration
    uint8_t xref0_config__ictrl_xreg_vref;                      // XREF - XREG calibration
    uint8_t pll0_config__ictrl_vddlv_timing;                    // PLL  1.8V VDD
    uint8_t lo0_generic__ldo_cur_dac;                           // LO 0.9V VDD
    uint8_t loio0_generic__ldo_cur_dac;                         // LOIO 0.9V VDD
    uint8_t loio0_generic__ldopa_cur_dac;                       // LOIO 0.9V VDD
    uint8_t clkgen1_generic__vtr_curr_dac;                      // Clock divider 0.9V VDD
    uint8_t pmon0_generic__cur_dac;                             // PMON 0.9V VDD
    uint8_t lpbk0_config__ldo_cur_ctrl;                         // Loopback 0.9V VDD

    // PLL/XTAL Calibration
    uint8_t pll0_config__xvtr_ztr_code;                         // PLL - XVTR calibration
    uint8_t pll0_config__ictrl_vddlv_osc;                       // PLL - XREGVCO calibration
    uint8_t pll0_generic__ictrl_vco_gatebias;                   // PLL - XVCO GATEBIAS (VCO IBIAS) calibration
    uint8_t pll0_generic__ictrl_varbulk0;                       // PLL - XVARIREF VARBULK 0 calibration
    uint8_t pll0_generic__ictrl_varbulk1;                       // PLL - XVARIREF VARBULK 1 calibration
    uint8_t pll2_config__ictrl_ref_lock_low;                    // PLL - XLOCKDET LOW calibration
    uint8_t pll2_config__ictrl_ref_lock_high;                   // PLL - XLOCKDET HIGH calibration
    uint8_t pllremote0_config__ictrl_cpbalance_drive_loopflt;   // PLL - XDRVLPFLT calibration

    // RX LDOs
    uint8_t rxfe1_generic__lna_stg1_ldo_csdac[NUM_RX_PER_BANK]; // RX FE 09V VDD
    uint8_t rxfe0_generic__mx_1p5ldo_csdac[NUM_RX_PER_BANK];    // RX FE 15V VDD
    uint8_t qilocfg_config__qiloldo_ref_dac[NUM_RX_PER_BANK];   // RX QILO 09V LDO
    uint8_t syncrx0_config__cur_dac[NUM_RX_PER_BANK];           // RX Sync 09V LDO
    uint8_t adccfg_config__ldo0p9_bump[NUM_RX_PER_BANK];        // RX ADC 09V LDO
    uint8_t adccfg_config__ldo1p6_bump[NUM_RX_PER_BANK];        // RX ADC 16V LDO

    void uhprint(bool all=true) const
    {
        UHPRINTF("TX  DAC-1V5 DAC-0V9 SYNC-0V9 BBCA-0V9 QILO-0V9 PA3060-0V9 PA120-0V9 PA3060-1V5\n");
        for (uint32_t tx = 0; tx < NUM_TX; tx++)
        {
            UHPRINTF("%2d   %3d     %3d     %3d       %3d     %3d       %3d        %3d       %3d\n", tx,
                    dacs_config__curr_dac0[tx],
                    dacs_generic__vreg0p9_ref_ctrl[tx],
                    synctx0_config__cur_dac[tx],
                    bbca0_generic__v0p9_ctrl[tx],
                    cfg_config__qiloldo_ref_dac[tx],
                    txfe2_config__vtr_ldotxdac[tx],
                    txfe2_generic__vtr_ldo_padac[tx],
                    txfe1_config__vtr_ldo1p5txdac[tx]);
            if (!all)
                break;
        }
        UHPRINTF(" \n");

        UHPRINTF("PLL0: VDD-1V8  XVTR XREGVCO     XREF: XVTR XREG     LO-0V9 LOIO LOIOPA CLKGEN PMON LPBK\n");
        UHPRINTF("        %3d     %3d   %3d             %3d   %3d      %3d   %3d   %3d    %3d   %3d   %3d\n",
                pll0_config__ictrl_vddlv_timing,
                pll0_config__xvtr_ztr_code,
                pll0_config__ictrl_vddlv_osc,
                xref0_config__ztr_code,
                xref0_config__ictrl_xreg_vref,
                lo0_generic__ldo_cur_dac,
                loio0_generic__ldo_cur_dac,
                loio0_generic__ldopa_cur_dac,
                clkgen1_generic__vtr_curr_dac,
                pmon0_generic__cur_dac,
                lpbk0_config__ldo_cur_ctrl);
        UHPRINTF(" \n");

        UHPRINTF("RX   FE-0V9   FE-1V5   QILO-0V9   SYNC-0V9   ADC-0V9   ADC-1V6\n");
        for (uint32_t rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            UHPRINTF("%2d    %3d     %3d       %3d        %3d        %3d       %3d\n", rx,
                    rxfe1_generic__lna_stg1_ldo_csdac[rx],
                    rxfe0_generic__mx_1p5ldo_csdac[rx],
                    qilocfg_config__qiloldo_ref_dac[rx],
                    syncrx0_config__cur_dac[rx],
                    adccfg_config__ldo0p9_bump[rx],
                    adccfg_config__ldo1p6_bump[rx]);
            if (!all)
                break;
        }
    }

    void set_reg_defaults();

#if __SCP__

    //! reset all fields in status structure to 'not requested'
    void reset_status()
    {
        for (INT i = 0; i < NUM_TX; i++)
        {
            dacs_generic__vreg0p9_ref_ctrl[i] = NOT_REQUESTED;
            dacs_config__curr_dac0[i] = NOT_REQUESTED;
            synctx0_config__cur_dac[i] = NOT_REQUESTED;
            bbca0_generic__v0p9_ctrl[i] = NOT_REQUESTED;
            cfg_config__qiloldo_ref_dac[i] = NOT_REQUESTED;
            txfe2_config__vtr_ldotxdac[i] = NOT_REQUESTED;
            txfe2_generic__vtr_ldo_padac[i] = NOT_REQUESTED;
            txfe1_config__vtr_ldo1p5txdac[i] = NOT_REQUESTED;
        }

        xref0_config__ictrl_xreg_vref = NOT_REQUESTED;
        pll0_config__ictrl_vddlv_timing = NOT_REQUESTED;
        pll0_config__ictrl_vddlv_osc = NOT_REQUESTED;
        lo0_generic__ldo_cur_dac = NOT_REQUESTED;
        loio0_generic__ldo_cur_dac = NOT_REQUESTED;
        loio0_generic__ldopa_cur_dac = NOT_REQUESTED;
        clkgen1_generic__vtr_curr_dac = NOT_REQUESTED;
        pmon0_generic__cur_dac = NOT_REQUESTED;
        lpbk0_config__ldo_cur_ctrl = NOT_REQUESTED;
        xref0_config__ztr_code = NOT_REQUESTED;
        pll0_config__xvtr_ztr_code = NOT_REQUESTED;

        for (INT i = 0; i < NUM_RX_PER_BANK; i++)
        {
            rxfe1_generic__lna_stg1_ldo_csdac[i] = NOT_REQUESTED;
            rxfe0_generic__mx_1p5ldo_csdac[i] = NOT_REQUESTED;
            qilocfg_config__qiloldo_ref_dac[i] = NOT_REQUESTED;
            syncrx0_config__cur_dac[i] = NOT_REQUESTED;
            adccfg_config__ldo0p9_bump[i] = NOT_REQUESTED;
            adccfg_config__ldo1p6_bump[i] = NOT_REQUESTED;
        }
    }

    bool check_cal_status(LDOCalBDataRaw& data)
    {
        bool ok = true;
        uint32_t uncalibrated = 0;

        LDOCalBDataRaw def;
        def.set_reg_defaults();

#define CHECK_RX(rxfield, name) check_rx(uncalibrated, rxfield, data.rxfield, def.rxfield, name)
        ok &= CHECK_RX(rxfe1_generic__lna_stg1_ldo_csdac, "RX FE-0.9V");
        ok &= CHECK_RX(rxfe0_generic__mx_1p5ldo_csdac,    "RX FE-1.5V");
        ok &= CHECK_RX(qilocfg_config__qiloldo_ref_dac,   "RX QILO-0.9V");
        ok &= CHECK_RX(syncrx0_config__cur_dac,           "RX SYNC-0.9V");
        ok &= CHECK_RX(adccfg_config__ldo0p9_bump,        "ADC-0.9V");
        ok &= CHECK_RX(adccfg_config__ldo1p6_bump,        "ADC-1.6V");

#define CHECK_TX(txfield, name) check_tx(uncalibrated, txfield, data.txfield, def.txfield, name)
        ok &= CHECK_TX(dacs_generic__vreg0p9_ref_ctrl,    "DAC-0.9V");
        ok &= CHECK_TX(dacs_config__curr_dac0,            "DAC-1.5V");
        ok &= CHECK_TX(synctx0_config__cur_dac,           "TX SYNC-0.9V");
        ok &= CHECK_TX(bbca0_generic__v0p9_ctrl,          "BBCA-0.9V");
        ok &= CHECK_TX(cfg_config__qiloldo_ref_dac,       "TX QILO-0.9V");
        ok &= CHECK_TX(txfe2_config__vtr_ldotxdac,        "PA3060-0.9V");
        ok &= CHECK_TX(txfe2_generic__vtr_ldo_padac,      "PA120-0.9V");
        ok &= CHECK_TX(txfe1_config__vtr_ldo1p5txdac,     "PA3060-1.5V");

#define CHECK_SH(shfield, name) check_sh(uncalibrated, shfield, data.shfield, def.shfield, name)
        ok &= CHECK_SH(xref0_config__ictrl_xreg_vref,   "XREG");
        ok &= CHECK_SH(pll0_config__ictrl_vddlv_timing, "18V-VDD");
        ok &= CHECK_SH(pll0_config__ictrl_vddlv_osc,    "XREGVCO");
        ok &= CHECK_SH(lo0_generic__ldo_cur_dac,        "LO 0.9V");
        ok &= CHECK_SH(clkgen1_generic__vtr_curr_dac,   "CLKGEN");
        ok &= CHECK_SH(pmon0_generic__cur_dac,          "PMON");
        ok &= CHECK_SH(lpbk0_config__ldo_cur_ctrl,      "LPBK");
        ok &= CHECK_SH(loio0_generic__ldo_cur_dac,      "LOIO");
        ok &= CHECK_SH(loio0_generic__ldopa_cur_dac,    "LOIOPA");

#undef CHECK_RX
#undef CHECK_TX
#undef CHECK_SH

#if 0 // these do not appear to be calibrated
        ok &= CHECK_SH(xref0_config__ztr_code,                              "XREF-XVTR");
        ok &= CHECK_SH(pll0_config__xvtr_ztr_code,                          "XVTR");
        ok &= CHECK_SH(pll0_generic__ictrl_vco_gatebias,                    "VCO-IBIAS");
        ok &= CHECK_SH(pll0_generic__ictrl_varbulk0,                        "XVARIREF VARBULK0");
        ok &= CHECK_SH(pll0_generic__ictrl_varbulk1,                        "XVARIREF VARBULK1");
        ok &= CHECK_SH(pll2_config__ictrl_ref_lock_low,                     "XLOCKDET LOW");
        ok &= CHECK_SH(pll2_config__ictrl_ref_lock_high,                    "XLOCKDET HIGH");
        ok &= CHECK_SH(pllremote0_config__ictrl_cpbalance_drive_loopflt,    "XDRVLPFLT");
#endif

        if (uncalibrated)
        {
            UHPRINTF("%d LDOs were uncalibrated\n", uncalibrated);
        }
        return ok;
    }

private:

    bool check_cal(uint32_t& uncalibrated, uint8_t status, INT channel, const CHAR* name)
    {
        bool ok = false;
        const CHAR* err = "unknown";

        switch (status)
        {
        case LDO_CALIBRATION_SUCCESS:
            ok = true;
            break;

        case ERROR_REQUIRED_VALUE_NOT_SPECIFIED:
            err = "required value not specified";
            break;

        case ERROR_INVALID_BUS_NAME:
            err = "invalid bus name";
            break;

        case ERROR_CHANNEL_OUT_OF_RANGE:
            err = "channel out of range";
            break;

        case ERROR_APROBE_NOT_INITIALIZED:
            err = "aprobe not initialized";
            break;

        case ERROR_OUTSIDE_OF_SAFETY_LIMITS:
            err = "outside of safety limits";
            break;

        case ERROR_CONTROL_DIRECTION_ERROR:
            err = "control direction error";
            break;

        case ERROR_OSCILLATION_STATE:
            err = "oscillation state";
            break;

        case ERROR_OUTPUT_DOESNT_CHANGE:
            err = "output does not change";
            break;

        case ERROR_INPUT_SUPPLY_LOW:
            err = "input supply too low";
            break;

        case ERROR_START_FAILURE:
            err = "measurement requests refused (unknown cause)";
            break;

        case NOT_REQUESTED:
            if (channel == 0)
            {
                UHPRINTF("LDO %s is uncalibrated\n", name);
            }
            uncalibrated++;
            ok = true;
            break;
        }

        if (!ok)
        {
            UHPRINTF("Cal failed for LDO %s channel %d cause: %s\n", name, channel, err);
        }
        return ok;
    }

    bool check_rx(uint32_t& uncalibrated,
                  uint8_t status[NUM_RX_PER_BANK],
                  uint8_t data[NUM_RX_PER_BANK],
                  uint8_t def[NUM_RX_PER_BANK],
                  const CHAR* name)
    {
        bool ok = true;

        for (INT rx = 0; rx < NUM_RX_PER_BANK; rx++)
        {
            if (!check_cal(uncalibrated, status[rx], rx, name))
            {
                data[rx] = def[rx];
                ok = false;
            }
        }

        return ok;
    }

    bool check_tx(uint32_t& uncalibrated,
                  uint8_t status[NUM_TX],
                  uint8_t data[NUM_TX],
                  uint8_t def[NUM_TX],
                  const CHAR* name)
    {
        bool ok = true;

        for (INT tx = 0; tx < NUM_TX; tx++)
        {
            if (!check_cal(uncalibrated, status[tx], tx, name))
            {
                data[tx] = def[tx];
                ok = false;
            }
        }

        return ok;
    }

    bool check_sh(uint32_t& uncalibrated,
                  uint8_t status,
                  uint8_t& data,
                  uint8_t def,
                  const CHAR* name)
    {
        bool ok = true;

        if (!check_cal(uncalibrated, status, 0, name))
        {
            data = def;
            ok = false;
        }

        return ok;
    }
#endif
};

struct LDOCalBData : public LDOCalBDataRaw
{
    uint8_t padding[256 - sizeof(LDOCalBDataRaw)];

    void set_defaults()
    {
        memset(this, 0xFFU, sizeof(*this));
        set_reg_defaults();
    }
};


// registers and measurement are indexed by LDO
enum LdoCalibration {
    // LDO regulator section
    REG_TX_DAC_1P5 = 0,     // #0
    REG_TX_DAC_0P9,
    REG_TX_SYNC_0P9,
    REG_TX_BBCA_0P9,
    REG_TX_QILO_0P9,
    REG_TX_PA3060_0P9,
    REG_TX_PA120_0P9,
    REG_TX_PA3060_1P5,
    REG_SH_XREF_1P6,        // #8
    REG_SH_VCOREG_0P9,
    REG_SH_TIMREG_0P9,
    REG_SH_LO_0P9,
    REG_SH_LOIO_0P9,
    REG_SH_LOIO_PA_0P9,
    REG_SH_CLKDIV_0P9,
    REG_SH_PMON_0P9,
    REG_SH_LOOPBACK_0P9,
    REG_RX_FE_1P5,          // #17
    REG_RX_FE_0P9,
    REG_RX_QILO_0P9,
    REG_RX_SYNC_0P9,
    REG_RX_ADC_0P9,
    REG_RX_ADC_1P6,

    // other calibrations that can be done using the same cal engine
    VTR_XREF,               // #23
    VTR_PLL,
    NUMER_OF_CAL_RECORDS,
    NUMBER_OF_LDOS_ON_PART = 22,
    NUM_TX_LDOS = 8,
    NUM_SH_LDOS = 9,
    NUM_RX_LDOS = 6,
};


struct LDOCalParamKey
{
    LdoCalibration ldo_cal;
    uint32_t       reserved[3];

    enum { KEY_VERSION = 1 };

    bool compare(const LDOCalParamKey& other) const
    {
        return ldo_cal == other.ldo_cal;
    }

    FLOAT distance(const LDOCalParamKey& other) const
    {
        return compare(other) ? 0 : CAL_KEY_DO_NOT_USE;
    }

    void upgrade_from_version(uint32_t, struct LdoCalibrationParameters&) { }

    void uhprint() const
    {
        UHPRINTF("(LDO Trim) ldo_cal: %d\n", ldo_cal);
    }
};


struct LdoCalibrationParameters
{
    CHAR        name[32];
    int32_t     control_direction;          // relationship between code direction and expected change direction
    FLOAT       default_target_voltage;     // default target voltage if none specified
    FLOAT       min_delta;                  // minimum delta value (used for step calculation)
    FLOAT       vstep_min;                  // minimum step voltage below which it is considered to be a 'small step'
    int8_t      min_code;                   // minimum code value
    int8_t      max_code;                   // maximum code value
    int8_t      max_code_step;              // size of maximum code step allowed
    int8_t      max_small_steps;            // max number of successive small steps allowed before abort
    FLOAT       min_step_voltage;           // minimum voltage step below which the LDO is not considered to have moved
    uint32_t    settling_time;              // delay time in us between register update and measure
    FLOAT       min_abort_voltage;          // any voltage below this triggers abort
    FLOAT       max_abort_voltage;          // any voltage above this triggers abort
    FLOAT       max_abs_final_error;        // max error final permitted before success
    uint32_t    ldo_supply_check_node;      // node to measure before calibration to make sure supply is OK
    FLOAT       ldo_supply_check_voltage;   // minimum supply voltage required for calibration
    FLOAT       max_final_error_voltage;    // maximum final calibrated supply error voltage for successful calibration
    int8_t      ldo_supply_num;             // enum for LDO input supply for reporting
    int8_t      on_fail_code;               // register restored to this value on fail
    int8_t      error_code;                 // error reported on abort
    int8_t      pack0;                      // pack structure to integer long-word length (21 long-words, 84 bytes)

    void uhprint() const
    {
        UHPRINTF("LDO Cal Param: %s\n", name);
        UHPRINTF(" target %.3f dir %d min_delta %.3f vstep_min %.3f\n",
                default_target_voltage, control_direction, min_delta, vstep_min);
        UHPRINTF(" min_code %d max_code %d max_code_step %d max_small_steps %d\n",
                min_code, max_code, max_code_step, max_small_steps);
        UHPRINTF(" settling time %d, min_abort %.3f max_abort %.3f max_error %.3f\n",
                settling_time, min_abort_voltage, max_abort_voltage, max_abs_final_error);
        UHPRINTF(" on_fail_code %d error_code %d\n",
                on_fail_code, error_code);
    }
};


struct RegulatorVoltageRecord
{
    FLOAT tx_voltage[NUM_TX_LDOS][NUM_TX];
    FLOAT sh_voltage[NUM_SH_LDOS];
    FLOAT rx_voltage[NUM_RX_LDOS][NUM_RX_PER_BANK];

    FLOAT tx_min[NUM_TX_LDOS];
    FLOAT sh_min[NUM_SH_LDOS];
    FLOAT rx_min[NUM_RX_LDOS];

    FLOAT tx_max[NUM_TX_LDOS];
    FLOAT sh_max[NUM_SH_LDOS];
    FLOAT rx_max[NUM_RX_LDOS];

    bool inside_limits(FLOAT val, FLOAT min, FLOAT max)
    {
        // always true if min or max not initialized
        return (min!=-1.0) || (max!=-1.0) || ((val >= min) && (val <= max));
    }

public:

    RegulatorVoltageRecord()
    {
        uint32_t channel, ldo;

        for (ldo = 0; ldo < NUM_TX_LDOS; ldo++)
        {
            for (channel = 0; channel < NUM_TX; channel++)
            {
                tx_voltage[ldo][channel] = 0.0F;
            }
            tx_min[ldo] = -1.0F;
            tx_max[ldo] = -1.0F;
        }

        for (ldo = 0; ldo < NUM_SH_LDOS; ldo++)
        {
            sh_voltage[ldo] = 0.0F;
            sh_min[ldo] = -1.0F;
            sh_max[ldo] = -1.0F;
        }

        for (ldo = 0; ldo < NUM_RX_LDOS; ldo++)
        {
            for (channel = 0; channel < NUM_RX_PER_BANK; channel++)
            {
                rx_voltage[ldo][channel] = 0.0F;
            }
            rx_min[ldo] = -1.0F;
            rx_max[ldo] = -1.0F;
        }
    }


    void set(uint32_t ldo_enum, uint32_t channel, FLOAT voltage)
    {
        if (ldo_enum < NUM_TX_LDOS)
        {
            if (channel < NUM_TX)
            {
                tx_voltage[ldo_enum][channel] = voltage;
            }
        }
        else if ((ldo_enum - NUM_TX_LDOS) < NUM_SH_LDOS)
        {
            sh_voltage[ldo_enum - NUM_TX_LDOS] = voltage;
        }
        else if ((ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS) < NUM_RX_LDOS)
        {
            if (channel < NUM_RX_PER_BANK)
            {
                rx_voltage[ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS][channel] = voltage;
            }
        }
    }


    FLOAT get(uint32_t ldo_enum, uint32_t channel)
    {
        if (ldo_enum < NUM_TX_LDOS)
        {
            if (channel < NUM_TX)
            {
                return tx_voltage[ldo_enum][channel];
            }
        }
        else if ((ldo_enum - NUM_TX_LDOS) < NUM_SH_LDOS)
        {
            return sh_voltage[ldo_enum - NUM_TX_LDOS];
        }
        else if ((ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS) < NUM_RX_LDOS)
        {
            if (channel < NUM_RX_PER_BANK)
            {
                return rx_voltage[ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS][channel];
            }
        }

        return 0.0F;
    }


    void limits(uint32_t ldo_enum, uint32_t channel, FLOAT min, FLOAT max)
    {
        (void)channel;

        if (ldo_enum < NUM_TX_LDOS)
        {
            tx_min[ldo_enum] = min;
            tx_max[ldo_enum] = max;
        }
        else if ((ldo_enum - NUM_TX_LDOS) < NUM_SH_LDOS)
        {
            sh_min[ldo_enum - NUM_TX_LDOS] = min;
            sh_max[ldo_enum - NUM_TX_LDOS] = max;
        }
        else if ((ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS) < NUM_RX_LDOS)
        {
            rx_min[ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS] = min;
            rx_max[ldo_enum - NUM_TX_LDOS - NUM_SH_LDOS] = max;
        }
    }


    bool all_inside_limits()
    {
        bool inside_limits_flag = true;
        INT ldo_enum, channel_num;

        for (ldo_enum = 0; ldo_enum < NUM_TX_LDOS; ldo_enum++)
        {
            for (channel_num = 0; channel_num < NUM_TX; channel_num++)
            {
                if (!inside_limits(tx_voltage[ldo_enum][channel_num], tx_min[ldo_enum], tx_max[ldo_enum]))
                {
                    inside_limits_flag = false;
                }
            }
        }

        for (; ldo_enum < NUM_TX_LDOS+NUM_SH_LDOS; ldo_enum++)
        {
            if (!inside_limits(sh_voltage[ldo_enum], sh_min[ldo_enum], sh_max[ldo_enum]))
            {
                inside_limits_flag = false;
            }
        }

        for (; ldo_enum < NUM_TX_LDOS+NUM_SH_LDOS+NUM_RX_LDOS; ldo_enum++)
        {
            for (channel_num = 0; channel_num < NUM_RX_PER_BANK; channel_num++)
            {
                if (!inside_limits(rx_voltage[ldo_enum][channel_num], rx_min[ldo_enum], rx_max[ldo_enum]))
                {
                    inside_limits_flag = false;
                }
            }
        }

        return inside_limits_flag;
    }

    void uhprint()
    {
        UHPRINTF("TX  DAC-1V5  DAC-0V9  SYNC-0V9  BBCA-0V9  QILO-0V9  PA3060-0V9  PA120-0V9  PA3060-1V5\n");
        for (uint32_t channel = 0; channel < NUM_TX; channel++)
        {
            UHPRINTF("%2d %7.3f%c %7.3f%c %7.3f%c   %7.3f%c %7.3f%c   %7.3f%c    %7.3f%c   %7.3f%c\n", channel,
                tx_voltage[0][channel], inside_limits(tx_voltage[0][channel], tx_min[0], tx_max[0]) ? ' ' : '*',
                tx_voltage[1][channel], inside_limits(tx_voltage[1][channel], tx_min[1], tx_max[1]) ? ' ' : '*',
                tx_voltage[2][channel], inside_limits(tx_voltage[2][channel], tx_min[2], tx_max[2]) ? ' ' : '*',
                tx_voltage[3][channel], inside_limits(tx_voltage[3][channel], tx_min[3], tx_max[3]) ? ' ' : '*',
                tx_voltage[4][channel], inside_limits(tx_voltage[4][channel], tx_min[4], tx_max[4]) ? ' ' : '*',
                tx_voltage[5][channel], inside_limits(tx_voltage[5][channel], tx_min[5], tx_max[5]) ? ' ' : '*',
                tx_voltage[6][channel], inside_limits(tx_voltage[6][channel], tx_min[6], tx_max[6]) ? ' ' : '*',
                tx_voltage[7][channel], inside_limits(tx_voltage[7][channel], tx_min[7], tx_max[7]) ? ' ' : '*');
        }
        UHPRINTF("\n");

        UHPRINTF("XREF-1V6  PLLVCO-1V5  PLLTIM-1V5  LO-0V9   LOIO-0V9   LOIOPA-0V9   CLKGEN-0V9  PMON-0V9  LPBK-0V9\n");
        UHPRINTF("%7.3f%c  %7.3f%c    %7.3f%c   %7.3f%c  %7.3f%c    %7.3f%c    %7.3f%c   %7.3f%c   %7.3f%c\n",
            sh_voltage[0], inside_limits(sh_voltage[0], sh_min[0], sh_max[0]) ? ' ' : '*',
            sh_voltage[1], inside_limits(sh_voltage[1], sh_min[1], sh_max[1]) ? ' ' : '*',
            sh_voltage[2], inside_limits(sh_voltage[2], sh_min[2], sh_max[2]) ? ' ' : '*',
            sh_voltage[3], inside_limits(sh_voltage[3], sh_min[3], sh_max[3]) ? ' ' : '*',
            sh_voltage[4], inside_limits(sh_voltage[4], sh_min[4], sh_max[4]) ? ' ' : '*',
            sh_voltage[5], inside_limits(sh_voltage[5], sh_min[5], sh_max[5]) ? ' ' : '*',
            sh_voltage[6], inside_limits(sh_voltage[6], sh_min[6], sh_max[6]) ? ' ' : '*',
            sh_voltage[7], inside_limits(sh_voltage[7], sh_min[7], sh_max[7]) ? ' ' : '*',
            sh_voltage[8], inside_limits(sh_voltage[8], sh_min[8], sh_max[8]) ? ' ' : '*');

        UHPRINTF("\n");

        UHPRINTF("RX   FE-0V9   FE-1V5     QILO-0V9    SYNC-0V9    ADC-0V9    ADC-1V6\n");
        for (uint32_t channel = 0; channel < NUM_RX_PER_BANK; channel++)
        {
            UHPRINTF("%2d  %7.3f%c %7.3f%c   %7.3f%c    %7.3f%c    %7.3f%c   %7.3f%c\n", channel,
                rx_voltage[0][channel], inside_limits(rx_voltage[0][channel], rx_min[0], rx_max[0]) ? ' ' : '*',
                rx_voltage[1][channel], inside_limits(rx_voltage[1][channel], rx_min[1], rx_max[1]) ? ' ' : '*',
                rx_voltage[2][channel], inside_limits(rx_voltage[2][channel], rx_min[2], rx_max[2]) ? ' ' : '*',
                rx_voltage[3][channel], inside_limits(rx_voltage[3][channel], rx_min[3], rx_max[3]) ? ' ' : '*',
                rx_voltage[4][channel], inside_limits(rx_voltage[4][channel], rx_min[4], rx_max[4]) ? ' ' : '*',
                rx_voltage[5][channel], inside_limits(rx_voltage[5][channel], rx_min[5], rx_max[5]) ? ' ' : '*');
        }

        UHPRINTF("\n");
    }
    // NOTE: ADD CODE TO PRINT MIN/MAX FOR EACH RAIL

    void uhprint_limits()
    {
        UHPRINTF("TX   DAC-1V5  DAC-0V9  SYNC-0V9  BBCA-0V9  QILO-0V9  PA3060-0V9  PA120-0V9  PA3060-1V5\n");
        UHPRINTF("MAX %7.3f  %7.3f  %7.3f    %7.3f  %7.3f    %7.3f     %7.3f    %7.3f \n",
            tx_max[0], tx_max[1], tx_max[2], tx_max[3], tx_max[4], tx_max[5], tx_max[6], tx_max[7]);
        UHPRINTF("MIN %7.3f  %7.3f  %7.3f    %7.3f  %7.3f    %7.3f     %7.3f    %7.3f \n",
            tx_min[0], tx_min[1], tx_min[2], tx_min[3], tx_min[4], tx_min[5], tx_min[6], tx_min[7]);
        UHPRINTF("\n");

        UHPRINTF("    XREF-1V6  PLLVCO-1V5  PLLTIM-1V5  LO-0V9   LOIO-0V9   LOIOPA-0V9   CLKGEN-0V9  PMON-0V9  LPBK-0V9\n");
        UHPRINTF("MAX %7.3f   %7.3f     %7.3f    %7.3f   %7.3f     %7.3f     %7.3f    %7.3f    %7.3f \n",
            sh_max[0], sh_max[1], sh_max[2], sh_max[3], sh_max[4], sh_max[5], sh_max[6], sh_max[7], sh_max[8]);
        UHPRINTF("MIN %7.3f   %7.3f     %7.3f    %7.3f   %7.3f     %7.3f     %7.3f    %7.3f    %7.3f \n",
            sh_min[0], sh_min[1], sh_min[2], sh_min[3], sh_min[4], sh_min[5], sh_min[6], sh_min[7], sh_min[8]);
        UHPRINTF("\n");

        UHPRINTF("RX    FE-0V9   FE-1V5     QILO-0V9    SYNC-0V9    ADC-0V9    ADC-1V6\n");
        UHPRINTF("MAX  %7.3f  %7.3f    %7.3f     %7.3f     %7.3f    %7.3f \n",
            rx_max[0], rx_max[1], rx_max[2], rx_max[3], rx_max[4], rx_max[5]);
        UHPRINTF("MIN  %7.3f  %7.3f    %7.3f     %7.3f     %7.3f    %7.3f \n",
            rx_min[0], rx_min[1], rx_min[2], rx_min[3], rx_min[4], rx_min[5]);
        UHPRINTF("\n");
    }
};


SRS_CLOSE_NAMESPACE()

#endif
