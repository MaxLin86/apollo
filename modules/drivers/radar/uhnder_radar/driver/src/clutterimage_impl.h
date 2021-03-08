#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/clutterimage.h"


class ScanObject_Impl;
class ScanSerializer;

class ClutterImage_Impl : public ClutterImage
{
public:

    ClutterImage_Impl(ScanObject_Impl& s)
        : myscan(s)
        , last_err(CI_NO_ERROR)
        , mag_plane(NULL)
        , hd_plane(NULL)
        , ci_row_noise_floor(NULL)
        , ci_row_distance(NULL)
        , mag_offset(0)
        , hd_offset(0)
        , aborted(false)
    {
        allocate();
    }

    virtual ~ClutterImage_Impl()
    {
        delete [] mag_plane;
        delete [] hd_plane;
        delete [] ci_row_noise_floor;
        delete [] ci_row_distance;
    }

    virtual uint32_t get_azimuth_dimension() const;

    virtual uint32_t get_elevation_dimension() const;

    virtual uint32_t get_range_dimension() const;

    virtual bool     height_data_supported() const;

    virtual bool     doppler_data_supported() const;

    virtual PeakCountEnum get_sample(uint32_t az_idx,
                                     uint32_t el_idx,
                                     uint32_t range_idx,
                                     float& dbFS,
                                     float& snr,
                                     float& range,
                                     float& doppler,
                                     float& azimuth,
                                     float& elevation) const;

    virtual void     get_raw_sample(uint32_t az_idx,
                                    uint32_t el_idx,
                                    uint32_t range_idx,
                                    uint16_t& magnitude,
                                    uint16_t& count_height_doppler) const;

    virtual void     release();

    virtual Err      get_last_error() const { return last_err; }

            bool     deserialize(ScanSerializer& s);

            bool     serialize(ScanSerializer& s) const;

            void     allocate();

            void     handle_uhdp(const UhdpClutterImageHeader* msg, uint32_t len);

            void     setup();

            bool     post_process();

    ScanObject_Impl& myscan;

    mutable Err      last_err;

    uint16_t*        mag_plane;

    uint16_t*        hd_plane;

    uint16_t*        ci_row_noise_floor;

    uint16_t*        ci_row_distance;

    uint32_t         mag_offset;

    uint32_t         hd_offset;

    bool             aborted;
};
