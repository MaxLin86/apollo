// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_TRIM_CAL_FLASHDATA_H
#define SRS_HDR_TRIM_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()


#define MV_PER_V    (1000.0f)


enum EFUSE_TRIM_ENUMS {
    SCHEMA_0 = 0,
    SCHEMA_1 = 1,
    SCHEMA_2 = 2,
    SCHEMA_3 = 3,
    NUM_EFUSE_SCHEMAS = 4
};


struct TrimKey
{
    uint32_t reserved_0;

    enum { KEY_VERSION = 1 };

    bool compare(const TrimKey&) const { return true; }

    FLOAT distance(const TrimKey&) const { return 0; }

    void upgrade_from_version(uint32_t , struct TrimCalBData&) { }

    void uhprint() const { }
};


// *******************************************************************************************************
// *******************************************************************************************************
//! Generic TrimCalBData framework used to access the schema-identification field.
//! This is used because the other fields can change in their sizing and mapping.
struct TrimCalBData
{
    struct Cal0
    {
        uint32_t not_extracted;
    } cal_0;

    struct Cal1
    {
        uint32_t not_extracted;
    } cal_1;

    struct Cal2
    {
        uint32_t not_extracted;
    } cal_2;

    struct Cal3
    {
        uint32_t schema: 5;             // the schema-definition field is the only one that we CANNOT move
        uint32_t : 27;
    } id;

    struct Cal4
    {
        uint32_t not_extracted;
    } cal_4;

    struct Cal5
    {
        uint32_t not_extracted;
    } cal_5;

    struct Cal6
    {
        uint32_t not_extracted;
    } cal_6;

    struct Cal7
    {
        uint32_t not_extracted;
    } cal_7;

    struct Cal8
    {
        uint32_t not_extracted;
    } cal_8;

    struct Cal9
    {
        uint32_t not_extracted;
    } cal_9;

    struct Cal10
    {
        uint32_t not_extracted;
    } cal_10;

    void set_defaults();

    void uhprint() const {}
};


// sanity check values
#define THERMAL_V0_MIN              (  650.0f)
#define THERMAL_V0_MAX              (  900.0f)
#define THERMAL_SLOPE_MIN           (  -1.70)
#define THERMAL_SLOPE_MAX           (  -1.30)
#define THERMAL_V0_DEFAULT          (  780.0f)
#define THERMAL_SLOPE_DEFAULT       (  -1.5f)
#define MODULATOR_SCALING_MIN       (  -2.700f)
#define MODULATOR_SCALING_MAX       (  -2.500f)
#define MODULATOR_SCALING_DEFAULT   (  -2.602f)

// *******************************************************************************************************
// *******************************************************************************************************
//! Bit mapping for Schema-0 ECID data

#define SCHEMA0_GAIN_CORRECTION        (0.027F)        // rough gain correction for data taken using schema_0
#define SCHEMA0_FUSE_MADC_GAIN_SCALE   (-0.001F)       // integer fuse units are mV
#define SCHEMA0_FUSE_FUSE_OFFSET_SCALE (-0.000001F)    // integer fuse units are uV

struct TrimCalBData_schema0
{
    struct Cal0 // efuse addr 47
    {
        uint32_t ro1: 16;
        uint32_t ro2: 16;
    } cal0;

    struct Cal1 // efuse addr 48
    {
        uint32_t ro3: 16;
        uint32_t /* reserved */: 16;
    } cal1;

    struct Cal2 // efuse addr 49
    {
        uint32_t digital: 16;
        uint32_t analog:  16;
    } iddq;

    struct Cal3 // efuse addr 50
    {
        uint32_t schema: 5;
        uint32_t madc_ptat_code: 3;
        uint32_t v_0: 10;               // Thermal diode offset, in mV (V * 1e3)
        uint32_t slope: 8;              // Thermal diode slope, in C/10uV (C/V * 1e5)
        uint32_t /* reserved */: 6;
    } thermal_diode;

    struct Cal4 // efuse addr 52
    {
        uint32_t vtr_code: 4;
        uint32_t temp_code: 4;
        uint32_t ptat_code: 4;
        uint32_t cur_prog_code: 6;
        uint32_t /* reserved */: 14;
    } master_bias;

    struct Cal5 // efuse addr 53
    {
        uint32_t v_0: 10;               // Thermal sensor 1 offset, in mV (V * 1e3)
        uint32_t vtr_code: 4;
        uint32_t slope: 8;              // Thermal sensor 1 diode slope, in C/10uV (C/V * 1e5)
        uint32_t /* reserved */: 10;
    } adie_thermal_sensor_1;

    struct Cal6 // efuse addr 54
    {
        uint32_t v_0: 10;               // Thermal sensor 2 offset, in mV (V * 1e3)
        uint32_t vtr_code: 4;
        uint32_t slope: 8;              // Thermal sensor 2 diode slope, in C/10uV (C/V * 1e5)
        uint32_t /* reserved */: 10;
    } adie_thermal_sensor_2;

    struct Cal7 // efuse addr 55
    {
        uint32_t /* reserved */: 32;
    } d_die_thermal_sensor;

    struct Cal8 // efuse addr 112
    {
        uint32_t gain : 16;             // TX modulator gain, in units of mV (V * 1e3)    16-bit signed int
        uint32_t offset : 16;           // TX modulator offset, in units of 1uV (V * 1e6) 16-bit signed int
    } madc_tx;

    struct Cal9 // efuse addr 113
    {
        uint32_t gain : 16;             // RX modulator gain, in units of mV (V * 1e3)    16-bit signed int
        uint32_t offset : 16;           // RX modulator offset, in units of 1uV (V * 1e6) 16-bit signed int
    } madc_rx;

    struct Cal10 // efuse addr 114
    {
        uint32_t gain : 16;             // SH modulator gain, in units of mV (V * 1e3)    16-bit signed int
        uint32_t offset : 16;           // SH modulator offset, in units of 1uV (V * 1e6) 16-bit signed int
    } madc_sh;

    void set_defaults();

    void uhprint() const {}
};


// *******************************************************************************************************
// *******************************************************************************************************
//! Bit mapping for Schemas 1/2/3 ECID data
//! Changes from Schema 0:
//! 1) Thermal coefficients changed to C/density from C/V.
//! 2) Bit fields shifter for MADC and thermal values to improve accuracy.
//! 3) Scaling changed for MADC to improve accuracy.

#define SCHEMA123_THERMAL_V0_SCALING          (   1.0e-3f)
#define SCHEMA123_THERMAL_SLOPE_SCALING       ( -10.0e-6f)
#define SCHEMA123_MODULATOR_GAIN_SCALING      (-100.0e-6f)
#define SCHEMA123_MODULATOR_OFFSET_SCALING    (  10.0e-6f)

struct TrimCalBData_schema123
{
    struct Cal0
    {
        uint32_t ro1: 16;
        uint32_t ro2: 16;
    } cal0;

    struct Cal1
    {
        uint32_t ro3: 16;
        uint32_t /* reserved */: 16;
    } cal1;

    struct Cal2
    {
        uint32_t digital: 16;
        uint32_t analog:  16;
    } iddq;

    struct Cal3
    {
        uint32_t schema: 5;
        uint32_t madc_ptat_code: 3;
        uint32_t v_0: 10;               // Thermal diode offset, in mV (V * 1.0e-3), range is [1023.0, 0.0] mV
        uint32_t slope: 8;              // Thermal diode slope, in -10uV/C (V * 1.0e-5 / C), range is [-0.00255, 0.0] V/C
        uint32_t /* reserved */: 6;
    } thermal_diode;

    struct Cal4
    {
        uint32_t vtr_code: 4;
        uint32_t temp_code: 4;
        uint32_t ptat_code: 4;
        uint32_t cur_prog_code: 6;
        uint32_t /* reserved */: 14;
    } master_bias;

    struct Cal5
    {
        uint32_t v_0: 10;               // Thermal diode offset, in mV (V * 1.0e-3), range is [1023.0, 0.0] mV
        uint32_t vtr_code: 4;
        uint32_t slope: 8;              // Thermal diode slope, in -10uV/C (V * 1.0e-5 / C), range is [-0.00255, 0.0] V/C
        uint32_t /* reserved */: 10;
    } adie_thermal_sensor_1;

    struct Cal6
    {
        uint32_t v_0: 10;               // Thermal diode offset, in mV (V * 1.0e-3), range is [1023.0, 0.0] mV
        uint32_t vtr_code: 4;
        uint32_t slope: 8;              // Thermal diode slope, in -10uV/C (V * 1.0e-5 / C), range is [-0.00255, 0.0] V/C
        uint32_t /* reserved */: 10;
    } adie_thermal_sensor_2;

    struct Cal7
    {
        uint32_t /* reserved */: 32;
    } d_die_thermal_sensor;

    struct Cal8
    {
        uint16_t gain : 16;             // TX modulator gain, in units of -100uV (V * 1.0e-4) 16-bit unsigned int, range is [-6.5536, 0.0] V
        int16_t offset : 16;            // TX modulator offset, in units of 10uV (V * 1.0e-5) 16-bit signed int, range is [-327.68mV, 327.67mV]
    } madc_tx;

    struct Cal9
    {
        uint16_t gain : 16;             // RX modulator gain, in units of -100uV (V * 1.0e-4) 16-bit unsigned int, range is [-6.5536, 0.0] V
        int16_t offset : 16;            // RX modulator offset, in units of 10uV (V * 1.0e-5) 16-bit signed int, range is [-327.68mV, 327.67mV]
    } madc_rx;

    struct Cal10
    {
        uint16_t gain : 16;             // SH modulator gain, in units of -100uV (V * 1.0e-4) 16-bit unsigned int, range is [-6.5536, 0.0] V
        int16_t offset : 16;            // SH modulator offset, in units of 10uV (V * 1.0e-5) 16-bit signed int, range is [-327.68mV, 327.67mV]
    } madc_sh;

    void set_defaults(uint32_t);

    void uhprint() const {}
};


SRS_CLOSE_NAMESPACE()

#endif
