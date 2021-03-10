// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_TEMP_CAL_FLASHDATA_H
#define SRS_HDR_TEMP_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"

SRS_DECLARE_NAMESPACE()

enum RDC_TempSensor
{
    TempSensorX = 0,
    TempSensorY,
    NUM_TEMP_SENSORS
};

enum { RDC_TempCalMaxSamples = 151 };

struct TempCalData
{
    uint32_t                    num_points;                             // Number of entries in voltage[] and temperature[]
    FLOAT                       voltage[RDC_TempCalMaxSamples];         // Millivolts
    FLOAT                       temperature[RDC_TempCalMaxSamples];     // Degrees Celcius

    void uhprint() const
    {
        UHPRINTF("Number of data points:  %d%s\n",  num_points, (num_points > 10) ? "    printing first 10:" : "");
        for (uint32_t i = 0; i < uh_uintmin(10, num_points); i++)
        {
            UHPRINTF("data[%u]:  V = %8.3f,  temp = %6.1f\n", i, voltage[i], temperature[i]);
        }
    }

    void set_defaults()
    {
        num_points = 8;
        static const FLOAT v[] = { 870, 922, 972, 1024, 1078, 1131, 1190, 1256 }; // (millivolts)
        static const FLOAT t[] = { -20,   0,  20,   40,   60,   80,  100,  120 }; // (degrees C)
        for (uint32_t i = 0; i < num_points; i++)
        {
            voltage[i] = v[i];
            temperature[i] = t[i];
        }
    }
};

struct TempCalKeyPOD
{
    RDC_TempSensor              sensor_id;
};

struct TempCalKey : public TempCalKeyPOD
{
    uint32_t                    reserved_1;
    uint32_t                    reserved_2;
    uint32_t                    reserved_3;
    uint32_t                    reserved_4;
    uint32_t                    reserved_5;
    uint32_t                    reserved_6;
    uint32_t                    reserved_7;

    enum { KEY_VERSION = 0 };

    TempCalKey()
    {
        memset(this, 0, sizeof(*this));
    }


    TempCalKey(RDC_TempSensor _sensor_id)
    {
        memset(this, 0, sizeof(*this));
        sensor_id = _sensor_id;
    }


    void uhprint() const
    {
        UHPRINTF("(TEMP) sensor_id: %d\n", sensor_id);
    }


    void upgrade_from_version(uint32_t, TempCalData&)
    {
        // TODO: Upgrade an old key to the current key version, initialize
        // new key fields to sane default values
    }


    //!! This is the all-important key-search algorithm !!
    bool compare(const TempCalKey& other) const
    {
        return (sensor_id  == other.sensor_id);
    }


    FLOAT distance(const TempCalKey& other) const
    {
        return sensor_id == other.sensor_id ? 0 : CAL_KEY_DO_NOT_USE;
    }
};

extern const TempCalKeyPOD required_temp_keys[];
extern const uint32_t num_required_temp_keys;

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_TEMP_CAL_FLASHDATA_H
