#ifndef SRS_HDR_TEMP_CAL_FLASHDATA_H
#define SRS_HDR_TEMP_CAL_FLASHDATA_H 1
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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
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
