// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_ANGLE_CAL_FLASHDATA_H
#define SRS_HDR_ANGLE_CAL_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "rdc-errors.h"  // RDC layer logging subunits
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"    // MAX_VRX
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"
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
    void upgrade_from_version(uint32_t, struct AngleCalData&)
    {
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


struct DiagonalCalKey
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

    DiagonalCalKey()
    {
        memset(this, 0, sizeof(*this));
    }

    void uhprint() const
    {
        UHPRINTF("(DIAG)      RF: %.2f\n", carrier_freq);
        UHPRINTF("antenna config: %d\n", antenna_config_id);
    }

#if __SCP__
    void upgrade_from_version(uint32_t, struct DiagonalCalData&)
    {
    }
#endif

    bool compare(const DiagonalCalKey& other) const
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

    FLOAT distance(const DiagonalCalKey& other) const
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
        if (max_diag <= 0)
        {
            SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_DIAG, 0, 0);
        }
        else
        {
            max_diag = mag2db(max_diag);
            if (max_diag > 12.0F) // dB
            {
                SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_DIAG, (uint32_t)max_diag, TWO_FRACTIONAL_DIGITS_OF(max_diag));
            }
        }
        if (max_mag <= 0)
        {
            SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_COUPLING, 0, 0);
        }
        else
        {
            max_mag = mag2db(max_mag);
            if (max_mag > -10.0F) // dB
            {
                SystemLogger::instance().emit_warn(LOG_RDC, LOG_RDC_CALIB, ANGLE_CAL_WARN_COUPLING, (uint32_t)max_mag, TWO_FRACTIONAL_DIGITS_OF(max_mag));
            }
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

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));
        for (INT x = 0; x < MAX_VRX; x++)
        {
            matrix[(x * MAX_VRX) + x] = 1.0F;
        }
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

    void set_defaults()
    {
        for (INT x = 0; x < MAX_VRX; x++)
        {
            diag[x] = 1.0F;
        }
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ANGLE_CAL_FLASHDATA_H
