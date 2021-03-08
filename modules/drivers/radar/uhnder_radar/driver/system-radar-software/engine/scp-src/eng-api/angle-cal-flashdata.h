#ifndef SRS_HDR_ANGLE_CAL_FLASHDATA_H
#define SRS_HDR_ANGLE_CAL_FLASHDATA_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-errors.h"  // RDC layer logging subunits
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"    // MAX_VRX
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"
#if __SCP__
#include "antennas.h"
#endif

SRS_DECLARE_NAMESPACE()

struct AngleCalKey
{
    enum { KEY_VERSION = 2 };

    INT                       reserved_tx;
    INT                       reserved_rx;
    INT                       reserved_pa;
    FLOAT                     carrier_freq;
    uint32_t                  antenna_config_id;

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;
    uint32_t                  reserved_2;
    uint32_t                  reserved_3;
    uint32_t                  reserved_4;
    uint32_t                  reserved_5;

    AngleCalKey()
    {
        memset(this, 0, sizeof(*this));
    }

    void uhprint() const
    {
        UHPRINTF("(ANGLE)     RF: %.2f\n", carrier_freq);
        UHPRINTF("antenna config: %d\n", antenna_config_id);
    }

#if __SCP__
    void upgrade_from_version(uint32_t old_version, struct AngleCalData&)
    {
        if (old_version < 1)
        {
            carrier_freq = DEF_RF;
        }
        if (old_version < 2)
        {
            AntennaConfig::instance().update_angle_key(*this);
        }
    }
    void upgrade_from_version(uint32_t old_version, struct DiagonalCalData&)
    {
        if (old_version < 1)
        {
            carrier_freq = DEF_RF;
        }
        if (old_version < 2)
        {
            AntennaConfig::instance().update_angle_key(*this);
        }
    }
#endif

    bool compare(const AngleCalKey& other) const
    {
        if ((carrier_freq <= 0.0F) || (other.carrier_freq <= 0.0F))
        {
            return (antenna_config_id == other.antenna_config_id);
        }
        else
        {
            return (antenna_config_id == other.antenna_config_id) &&
                   (uh_fabsf(carrier_freq - other.carrier_freq) <= 0.2F);
        }
    }

    FLOAT distance(const AngleCalKey& other) const
    {
        return compare(other) ? 0 : CAL_KEY_DO_NOT_USE;
    }
};


struct AngleCalData
{
    cfloat matrix[MAX_VRX * MAX_VRX];

    void uhprint() const
    {
#if __BARE_METAL__
        // Check for "bad" calibrations
        FLOAT max_diag = 0.0F;
        FLOAT max_mag = 0.0F;
        for (INT x = 0; x < MAX_VRX; x++)
        {
            for (INT y = 0; y < MAX_VRX; y++)
            {
                FLOAT mag = matrix[(x * MAX_VRX) + y].abs();
                if (x == y)
                {
                    max_diag = (max_diag < mag) ? mag : max_diag;
                }
                else
                {
                    max_mag = (max_mag < mag) ? mag : max_mag;
                }
            }
        }
        max_diag = mag2db(max_diag);
        max_mag = mag2db(max_mag);
        if (max_diag > 12.0F) // dB
        {
            SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_DIAG, (uint32_t)max_diag, TWO_FRACTIONAL_DIGITS_OF(max_diag));
        }
        if (max_mag > -10.0F) // dB
        {
            SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_COUPLING, (uint32_t)max_mag, TWO_FRACTIONAL_DIGITS_OF(max_mag));
        }
#endif

        // Print the (selected portions of) data
        const uint32_t a0 = 0;
        const uint32_t a1 = 1 * MAX_VRX + 1;
        const uint32_t a2 = (MAX_VRX - 2) * MAX_VRX + (MAX_VRX - 2);
        const uint32_t a3 = (MAX_VRX - 1) * MAX_VRX + (MAX_VRX - 1);
        UHPRINTF("Diag: (%.3f, %.3f), (%.3f, %.3f), ...  (%.3f, %.3f), (%.3f, %.3f)\n",
                 matrix[a0].i, matrix[a0].q, matrix[a1].i, matrix[a1].q,
                 matrix[a2].i, matrix[a2].q, matrix[a3].i, matrix[a3].q);
    }
};

struct DiagonalCalData
{
    cfloat diag[MAX_VRX];

    void uhprint() const
    {
#if __BARE_METAL__
        // Check for "bad" calibrations
        FLOAT max_mag = 0.0F;
        for (INT x = 0; x < MAX_VRX; x++)
        {
            FLOAT mag = diag[x].abs();
            max_mag = (max_mag < mag) ? mag : max_mag;
        }
        max_mag = mag2db(max_mag);
        if (max_mag > 12.0F) // dB
        {
            SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_DIAG, (uint32_t)max_mag, TWO_FRACTIONAL_DIGITS_OF(max_mag));
        }
#endif

        // Print the (selected portions of) data
        const uint32_t a2 = MAX_VRX - 2;
        const uint32_t a3 = MAX_VRX - 1;
        UHPRINTF("Diag: (%.3f, %.3f), (%.3f, %.3f), ...  (%.3f, %.3f), (%.3f, %.3f)\n",
                 diag[0].i, diag[0].q, diag[1].i, diag[1].q,
                 diag[a2].i, diag[a2].q, diag[a3].i, diag[a3].q);
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ANGLE_CAL_FLASHDATA_H
