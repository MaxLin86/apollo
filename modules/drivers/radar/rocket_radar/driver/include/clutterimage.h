// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

class ClutterImage
{
public:

    virtual ~ClutterImage() {}

    //! returns the number of azimuth angles in this clutter image, generally
    //! 128 (1D) or 8 (2D). The Y dimension of the clutter image is azimuth minor
    //! elevation major
    virtual uint32_t get_azimuth_dimension() const = 0;

    //! returns the number of elevation angles in the clutter image, generally
    //! 1 (1D) or 8 (2D). The Y dimension of the clutter image is azimuth minor
    //! elevation major
    virtual uint32_t get_elevation_dimension() const = 0;

    //! returns the size of the X (distance) dimension of the clutter image
    virtual uint32_t get_range_dimension() const = 0;

    //! returns true if the clutter image format included a height map plane
    virtual bool     height_data_supported() const = 0;

    //! returns true if the clutter image format included a doppler map plane
    virtual bool     doppler_data_supported() const = 0;

    //! an enumeration that describes the number of peaks determined, usually in doppler,
    //! to be found at a single position of the clutter image (indexed by range and angle)
    enum PeakCountEnum
    {
        NO_PEAK,                //!< the sample magnitude was noise
        SINGLE_PEAK,            //!< the count field had a value of 1
        TWO_PEAKS,              //!< the count field had a value of 2
        THREE_PEAKS,            //!< the count field had a value of 3
        MORE_THAN_THREE_PEAKS,  //!< the count field had a value of 0
    };

    //! sample the clutter image. the returned doppler and elevation will be 0
    //! if the clutter image format did not support that data, or if the sample
    //! had a magnitude matching the noise floor at that range
    //!
    //! @param[in]  az_idx             0..azimuth dimension - 1
    //! @param[in]  el_idx             0..elevation dimension - 1
    //! @param[in]  range_idx          0..range dimension - 1
    //! @param[out] dbFS               magnitude at sample position, relative to full scale
    //! @param[out] snr                signal to noise ratio of sample
    //! @param[out] range              meters from the radar
    //! @param[out] doppler            mps relative to estimated ground
    //! @param[out] azimuth            radians from boresight, +Y is right
    //! @param[out] elevation          radians from boresight, +Z is down
    virtual PeakCountEnum get_sample(uint32_t az_idx, uint32_t el_idx, uint32_t range_idx,
            float& dbFS, float& snr, float& range, float& doppler, float& azimuth, float& elevation) const = 0;

    //! the returned count_height_doppler field is coded within the 16bits as
    //! 2 bits peak count, 7 bits height data and 7 bits doppler data and will
    //! have a value of 0 if the clutter image format did not support doppler or
    //! height data.
    //!
    //! @param[in]  az_idx               0..azimuth dimension - 1
    //! @param[in]  el_idx               0..elevation dimension - 1
    //! @param[in]  range_idx            0..range dimension - 1
    //! @param[out] magnitude            raw CI magnitude at position (pixel intensity)
    //! @param[out] count_height_doppler raw combined height, doppler, and count
    virtual void     get_raw_sample(uint32_t az_idx, uint32_t el_idx, uint32_t range_idx,
            uint16_t& magnitude, uint16_t& count_height_doppler) const = 0;

    //! release all storage of the clutter image
    virtual void     release() = 0;

    //! an enumeration of the errors potentially returned by this class
    enum Err
    {
        CI_NO_ERROR,
        CI_INDEX_OUT_OF_RANGE,
        CI_INVALID_DATA_PLANE,
        CI_FILE_WRITE_FAILURE,
    };

    //! returns the last error encountered by this clutter image
    virtual Err      get_last_error() const = 0;
};
