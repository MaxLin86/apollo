// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_STEERING_FLASHDATA_H
#define SRS_HDR_STEERING_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "rdc-errors.h"  // RDC layer logging subunits
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"    // MAX_VRX
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/default-presets.h"

SRS_DECLARE_NAMESPACE()

struct SteeringData
{
    uint32_t    tx_enable;                              // Bitmap of POWERED-ON Tx antennas (in HW-12 order)
    uint32_t    rx_enable;                              // Bitmap of enabled Rx antennas (in HW-16 order) (exactly 8 must be enabled)
                                                        // First set of 8 and second set of 8 must be exclusive, i.e.:
                                                        // First set of 8 XOR second set of 8 must equal 11111111

    int8_t      prn_idx[NUM_TX];                        // PRN ID for each of the 12 Tx antennas (in HW-12 order)
                                                        // 0...(MAX_TX_PRN-1) selects the PRN code, MAX_TX_PRN=disabled

    vec3f_t     pos_offset;                             // Offset (x,y,z) (in meters) from antenna positions (to center virtual array)

    uint32_t    pa_tx_num_centers;                      // Phased Array:  number of Tx phase centers (0 if PA disabled)
    vec3f_t     pa_tx_centers[MAX_TX_PRN];              // Phased Array:  positions (x,y,z) of Tx phase centers (in Y-major order)

    uint8_t     active_vrx[MAX_VRX / 8];                // Bitmap of enabled VRx, in Y-major VRx order
    uint8_t     dup_vrx[MAX_VRX / 8];                   // Bitmap of VRx with duplicated positions, in Y-major VRx order

    uint32_t    az_uniform_vrx;                         // Number of uniform array element positions in azimuth (Y)
    uint32_t    el_uniform_vrx;                         // Number of uniform array element positions in elevation (Z)

    uint32_t    az_vrx_spacing;                         // Spacing (1/2 * az_vrx_spacing_factor Lambda units ) between elements, in azimuth (Y)
    uint32_t    el_vrx_spacing;                         // Spacing (1/2 * el_vrx_spacing_factor Lambda units ) between elements, in elevation (Z)

    uint16_t    azimuth_bins;                           // Number of angle bins in azimuth (1 if elevation-only)
    uint16_t    elevation_bins;                         // Number of angle bins in elevation (1 if azimuth-only)

    uint16_t    azimuth_nyq_factor;                     // Azimuth angle Nyquist oversampling factor (or 0 if non-Nyquist spacing)
    uint16_t    elevation_nyq_factor;                   // Elevation angle Nyquist oversampling factor (or 0 if non-Nyquist spacing)

    FLOAT       azimuth_angles[MAX_ROUGH_ANGLES];       // Azimuth steering angles (radians)
    FLOAT       elevation_angles[MAX_ROUGH_ANGLES];     // Elevation steering angles (radians)
                                                        // The steering angles are in spatial order (Y-major / Z-minor)
                                                        // The angles may be non-uniform, however,
                                                        // if num_elevation_angles > 1, the angles must be on a grid, i.e.:
                                                        // Each row (azimuth) must have the same set of azimuth angles.
                                                        // Each column (elevation) must have the same set of elevation angles.

    cfloat      steering_vectors[MAX_ROUGH_ANGLES * MAX_VRX];   // Steering vectors (per angle)
                                                        // The steering vector matrix is VRx-major and Angle-minor,
                                                        // The angle dimension is in spatial order (Y-major / Z-minor),
                                                        // The VRx dimension is in spatial (Y-major / Z-minor) order.

    uint16_t    sva_second_pass_factor;                 // Angle Nyquist second pass oversampling factor (0 or 1: second pass disabled)

    uint16_t    enable_sva;                             // Enable SVA (0 = disabled, 1 = enabled)

    FLOAT       az_vrx_spacing_factor;                  // Set to 1.0 for 1/2 Lambda spacing (default) or scale appropriately
                                                        // This variable modifies the effect of az_vrx_spacing
    FLOAT       el_vrx_spacing_factor;                  // Set to 1.0 for 1/2 Lambda spacing (default) or scale appropriately
                                                        // This variable modifies the effect of el_vrx_spacing

    uint8_t     reserved[2036];

    void uhprint() const
    {
    }
};


struct SteeringKey
{
    enum { KEY_VERSION = 1 };

    uint32_t                  antenna_config_id;

    uint32_t                  reserved_0;
    uint32_t                  reserved_1;
    uint32_t                  reserved_2;
    uint32_t                  reserved_3;
    uint32_t                  reserved_4;
    uint32_t                  reserved_5;
    uint32_t                  reserved_6;

    SteeringKey()
    {
        memset(this, 0, sizeof(*this));
    }

    void uhprint() const
    {
        UHPRINTF("antenna config: %d\n", antenna_config_id);
    }

#if __SCP__
    void upgrade_from_version(uint32_t old_version, struct SteeringData& data)
    {
        if (old_version < 1)
        {
            data.az_vrx_spacing_factor = 1.0F;
            data.el_vrx_spacing_factor = 1.0F;
        }
    }
#endif

    bool compare(const SteeringKey& other) const
    {
        return (antenna_config_id == other.antenna_config_id);
    }

    FLOAT distance(const SteeringKey& other) const
    {
        return compare(other) ? 0 : CAL_KEY_DO_NOT_USE;
    }
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_STEERING_FLASHDATA_H
