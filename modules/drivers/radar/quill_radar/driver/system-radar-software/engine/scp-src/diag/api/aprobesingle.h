#ifndef SRS_HDR_APROBESINGLE_H
#define SRS_HDR_APROBESINGLE_H 1

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "aprobe_enums.h"
#if SABINE_B
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/cal-a/ldo-calb-flashdata.h"
#endif

SRS_DECLARE_NAMESPACE()

#define DEFAULT_NUM_SAMPLES 32768

// *****************************************************************************
//! Test input structure for "aprobesingle::getSingleAprobeValue"
struct aprobeSingle_getSingleAprobeValue_input
{
    //! Specify which MADC to probe: 0 = TX, 1 = SH, 2 = RX
    uint32_t whichMadc;
    //! Specify which channel (0 to 11 for TX, 1 for SH, 0 to 7 for RX)
    uint32_t channel;
    //! Spcify which point to probe
    uint32_t probe;
    //! Specify the type of measurement to make.  Typically use -1 to choose the default
    int32_t measurement;
    //! Choose the number of accumulations to make.  (16384 works well)
    uint32_t numAcc;
    //! Sticky bit - if non zero the probe input to the MADC remains until the next probe is set
    //!   otherwise the probe input reverts to default "off" state.
    uint32_t sticky;
};

//! Test input structure for "aprobesingle::getSingleAprobeValue"
struct aprobeSingle_getSingleAprobeValue_inputB
{
    //! Specify which MADC to probe: 0 = TX, 1 = SH, 2 = RX
    uint32_t bus;
    //! Specify which channel (0 to 11 for TX, 1 for SH, 0 to 7 for RX)
    uint32_t channel;
    //! Spcify which point to probe
    uint32_t hash_code;
    //! Specify the type of measurement to make.  Typically use -1 to choose the default
    int32_t measurement; //IGNORED FOR REVB
    //! Choose the number of accumulations to make.  (16384 works well)
    uint32_t num_samples;
    //! Sticky bit - if non zero the probe input to the MADC remains until the next probe is set
    //!   otherwise the probe input reverts to default "off" state.
    uint32_t sticky;
};

//! Test output structure for "aprobesingle::getSingleAprobeValue"
struct aprobeSingle_getSingleAprobeValue_output
{
    //! The output value of the aprobe normalized (typically a voltage)
    FLOAT output;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::selectAprobe"
struct aprobeSingle_selectAprobe_input
{
    //! Specify which MADC to probe: 0 = TX, 1 = SH, 2 = RX
    uint32_t bus;
    //! Specify which channel (0 to 11 for TX, 1 for SH, 0 to 7 for RX)
    int32_t channel;
    //! Spcify which point to probe
    uint32_t hash_code;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::releaseAprobe"
struct aprobeSingle_releaseAprobe_input
{
    //! Specify which MADC to probe: 0 = TX, 1 = SH, 2 = RX
    uint32_t bus;
};


// *****************************************************************************
//! Test output structure for "aprobesingle::get_aprobe_cal"
struct aprobeSingle_getAprobeCal_output
{
    FLOAT offset[NUM_APROBE_BUSES];
    FLOAT modulator_gain[NUM_APROBE_BUSES];
    FLOAT attenuator_gain[NUM_APROBE_BUSES];
};


// *****************************************************************************
struct aprobeSingle_setTestPin_input
{
    //! Specify which bus to connect: 0 = TX, 1 = SH, 2 = RX, all others=off
    int32_t bus;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::measure_ldo"
struct aprobeSingle_measureLdoVoltage_input
{
    //! Specify which LDO to measure
    uint32_t ldo_enum;
    //! Specify which channel (0 to 11 for TX, 1 for SH, 0 to 7 for RX)
    uint32_t channel;
};

//! Test output structure for "aprobesingle::measure_ldo"
struct aprobeSingle_measureLdoVoltage_output
{
    //! The output value of the aprobe normalized (typically a voltage)
    FLOAT output;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::cal_ldo"
struct aprobeSingle_calLdoVoltage_input
{
    //! Specify which LDO to measure
    uint32_t ldo_enum;
    //! Specify which channel (0 to 11 for TX, 1 for SH, 0 to 7 for RX)
    uint32_t channel;
    //! Specify the calibration target voltage
    FLOAT voltage;
};

//! Test output structure for "aprobesingle::cal_ldo"
struct aprobeSingle_calLdoVoltage_output
{
    //! The output value of the aprobe normalized (typically a voltage)
    int32_t termination_value;
    FLOAT voltage;
    uint32_t code;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::set_clock"
struct aprobeSingle_setClock_input
{
    //! Specify which clock source
    uint32_t clock_source;

    //! Specify additional divide ratio
    uint32_t target_frequency;
};

//! Test output structure for "aprobesingle::set_clock"
struct aprobeSingle_setClock_output
{
    //! The output value of the aprobe normalized (typically a voltage)
    int32_t clock_frequency;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::diagnostic_test"
struct aprobeSingle_diagnosticTest_input
{
    //! Specify which clock source
    uint32_t test;

    //! Specify additional divide ratio
    uint32_t parameter;

    uint32_t other0, other1;
};

//! Test output structure for "aprobesingle::diagnostic_test"
struct aprobeSingle_diagnosticTest_output
{
    //! The output value of the aprobe normalized (typically a voltage)
    int32_t result;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::diagnostic_test"
struct aprobeSingle_snapshot_input
{
    //! Specify operation
    uint32_t operation; // Off/On/Dump

    int32_t delay_count;
    int32_t run_count;

    //! Specify register block mask to be copied
    uint32_t mask;

    //! Specify triggering measurement
    uint32_t bus;
    int32_t channel;
    uint32_t hash;
};


#if SABINE_B
// *****************************************************************************
//! Test input structure for "aprobesingle::cal_ldo"
struct aprobeSingle_bulkCalLdo_input
{
    //! Specify which LDO's to calibrate
    uint32_t ldo_mask;

    //! Specify which channels to calibrate
    uint32_t channel_mask;
};

//! Test output structure for "aprobesingle::cal_ldo"
struct aprobeSingle_bulkCalStatus_output
{
    //! The calibration engine status
    int32_t running;
    int32_t current_ldo;
    int32_t current_channel;
    int32_t complete_message_received;

    LDOCalBDataRaw current_state;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::bulk_aprobe_read"
struct aprobeSingle_bulkAprobeRead_input
{
    //! Specify what data type to return
    uint32_t get_hash_flag;

    //! Specify which bus to return
    uint32_t bus;

    //! Specify which channel to return
    uint32_t channel;
};

#if defined(BUILD_SABINE_REMOTE_API)
enum { MAX_NUM_BULK_MEASUREMENTS = 230 };
#endif

//! Test output structure for "aprobesingle::bulk_aprobe_read"
struct aprobeSingle_bulkAprobeRead_output
{
    uint32_t data[MAX_NUM_BULK_MEASUREMENTS];
};


//! Test input structure for "aprobesingle::bulk_aprobe_write"
struct aprobeSingle_bulkAprobeWrite_input
{
    //! Specify which bus to return
    uint32_t bus;

    //! Specify which bus to return
    uint32_t channel_mask;

    //! Specify the measurement hash list
    uint32_t data[MAX_NUM_BULK_MEASUREMENTS];
};


// *****************************************************************************
//! Test output structure for "aprobesingle::measure_temperature"
struct aprobeSingle_measureTemperature_input
{
    //! Sensor to read
    uint32_t temp_sensor_enum;
    uint32_t channel;
};

struct aprobeSingle_measureTemperature_output
{
    //! Sensor temperature
    FLOAT temp_sensor_c;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::set_ldo_cal_parameters"
struct aprobeSingle_setLdoCalParameters_input
{
    //! Specify which enumerated LDO parameter set to update
    uint32_t ldo_enum;

    //! New parameters
    LdoCalibrationParameters parameters;
};


// *****************************************************************************
//! Test input structure for "aprobesingle::cal_ldo"
struct aprobeSingle_registerDump_input
{
    //! Specify which LDO's to calibrate
    uint32_t bank;
};

//! Test output structure for "aprobesingle::cal_ldo"
struct aprobeSingle_registerDump_output
{
    uint32_t length;

    //! The calibration engine status
    uint32_t data[0x2c0/4];
};


#define NUM_MEASUREMENT_POINTS  6
#define REGISTER_MAX            32
struct aprobeSingle_regSweep_output
{
    uint32_t reg_val;
    uint32_t measurement_index;
    uint32_t sweep_running;
    FLOAT measurement[NUM_MEASUREMENT_POINTS][REGISTER_MAX];
};


#endif

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_APROBESINGLE_H
