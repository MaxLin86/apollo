#ifndef SRS_HDR_RDC_EXTENDED_DET_H
#define SRS_HDR_RDC_EXTENDED_DET_H 1
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
/*! \file */

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

SRS_DECLARE_NAMESPACE()

//! The RDC_ExtendedDetection class has very little internal storage, and is
//! intended to be instantiated on the stack for each extended detection. This
//! class serves as an accessor (and iterator) for the packed data buffer.
class RDC_ExtendedDetection
{
public:

    struct Header
    {
        uint32_t magic_value;
        uint32_t ext_det_flags;
        uint32_t min_range_bin;
        uint32_t max_range_bin;
    };

    static const uint32_t MAGIC_VALUE = 0xDE1EC10AUL;

    enum HdrFlags
    {
        EXT_DET_GLOBAL_DOPPLER_REGION = (1U << 0),  // doppler region defined in global header
        EXT_DET_GLOBAL_ANGLE_REGION   = (1U << 1),  // angle region defined in global header
        EXT_DET_SPECIAL_SELECTION     = (1U << 2),
    };

    RDC_ExtendedDetection(const int8_t* ptr, uint32_t size)
    {
        const Header* hdr = reinterpret_cast<const Header*>(ptr);

        if ((hdr->magic_value != MAGIC_VALUE) || (size < sizeof(Header)))
        {
            return;
        }

        min_range_bin = hdr->min_range_bin;
        max_range_bin = hdr->max_range_bin;
        uint32_t num_range_bins = max_range_bin - min_range_bin + 1;

        if (max_range_bin < min_range_bin)
        {
            return;
        }

        if (hdr->ext_det_flags & EXT_DET_GLOBAL_DOPPLER_REGION)
        {
            const uint16_t* dop_hdr = reinterpret_cast<const uint16_t*>(hdr + 1);
            min_doppler_bin = dop_hdr[0];
            max_doppler_bin = dop_hdr[1];

            range_bin_offsets = dop_hdr + 2;
        }
        else
        {
            min_doppler_bin = 0;
            max_doppler_bin = 0;
            range_bin_offsets = reinterpret_cast<const uint16_t*>(hdr + 1);
        }

        if (hdr->ext_det_flags & EXT_DET_GLOBAL_DOPPLER_REGION)
        {
            min_azimuth_bin   = *range_bin_offsets++;
            max_azimuth_bin   = *range_bin_offsets++;
            min_elevation_bin = *range_bin_offsets++;
            max_elevation_bin = *range_bin_offsets++;
        }
        else
        {
            min_azimuth_bin   = max_azimuth_bin   = 0;
            min_elevation_bin = max_elevation_bin = 0;
        }

        if (intptr_t(size) < (int8_t*)(range_bin_offsets + (num_range_bins + 1)) - ptr)
        {
            return;
        }

        // the last range_bin_offset is the total size (the offset where an
        // additional range bin would have started)
        if (range_bin_offsets[num_range_bins] != size)
        {
            return;
        }
        // the first range_bin_offset should be the end of the range_bin_offsets table
        if (range_bin_offsets[0] != (int8_t*)(&range_bin_offsets[num_range_bins + 1]) - ptr)
        {
            return;
        }

        // detections buffer checks out, is ok
        buffer = ptr;
        size_bytes = size;
    }


    bool is_ok() const { return buffer && size_bytes; }

    //! returns the range extents (inclusive) of this detection.
    void get_range_extents(uint32_t& min_range, uint32_t& max_range) const
    {
        min_range = min_range_bin;
        max_range = max_range_bin;
    }


    //! at each range, the detection will have a rectangular bounds in azimuth
    //! and elevation. At each position within this rectangular angle bound, there
    //! will be one or more samples at various doppler velocities
    bool get_angle_extents_at_range(uint32_t range_bin,
                                    uint32_t& min_azimuth,   uint32_t& max_azimuth,
                                    uint32_t& min_elevation, uint32_t& max_elevation) const
    {
        if (!is_ok() || (range_bin < min_range_bin) || (range_bin > max_range_bin))
        {
            return false;
        }

        const Header* hdr = reinterpret_cast<const Header*>(buffer);
        if (hdr->ext_det_flags & EXT_DET_GLOBAL_ANGLE_REGION)
        {
            min_azimuth   = min_azimuth_bin;
            max_azimuth   = max_azimuth_bin;
            min_elevation = min_elevation_bin;
            max_elevation = max_elevation_bin;
        }
        else
        {
            const uint16_t* bfhdr = reinterpret_cast<const uint16_t*>(buffer + range_bin_offsets[range_bin - min_range_bin]);
            min_azimuth   = *bfhdr++;
            max_azimuth   = *bfhdr++;
            min_elevation = *bfhdr++;
            max_elevation = *bfhdr++;
        }

        return true;
    }


    //! There can be multiple magnitude samples with different doppler velocities
    //! at each range angle. This method returns the count of doppler magnitude
    //! pairs that are available from the get_range_angle_sample() iterator
    uint32_t get_num_samples_at_range_angle(uint32_t range_bin, uint32_t azimuth_bin, uint32_t elevation_bin) const
    {
        uint32_t min_az, max_az, min_el, max_el;

        if (!is_ok() || !get_angle_extents_at_range(range_bin, min_az, max_az, min_el, max_el))
        {
            return 0;
        }

        if ((azimuth_bin   < min_az) || (azimuth_bin   > max_az) ||
            (elevation_bin < min_az) || (elevation_bin > max_az))
        {
            return 0;
        }

        const Header* hdr = reinterpret_cast<const Header*>(buffer);
        if (hdr->ext_det_flags & EXT_DET_GLOBAL_DOPPLER_REGION)
        {
            return max_doppler_bin - min_doppler_bin + 1;
        }
        else
        {
            // TODO: support for variable doppler
            return 0;
        }
    }


    //! at any particular range/angle location, there might be multiple samples
    //! at various doppler velocities. this method returns a specific doppler
    //! bin and sample magnitude pair.  The dopper bin has 4 fraction bits
    bool get_range_angle_sample(uint32_t range_bin,     uint32_t azimuth_bin,
                                uint32_t elevation_bin, uint32_t sample_idx,
                                uint16_t& magnitude,    uint16_t& doppler_bin_Q11_4) const
    {
        if (!is_ok())
        {
            return false;
        }
        if (sample_idx >= get_num_samples_at_range_angle(range_bin, azimuth_bin, elevation_bin))
        {
            return false;
        }

        uint32_t min_az, max_az, min_el, max_el;
        if (!get_angle_extents_at_range(range_bin, min_az, max_az, min_el, max_el))
        {
            return false;
        }

        const Header* hdr = reinterpret_cast<const Header*>(buffer);

        const uint16_t* bfhdr = reinterpret_cast<const uint16_t*>(buffer + range_bin_offsets[range_bin - min_range_bin]);
        if (hdr->ext_det_flags & EXT_DET_GLOBAL_ANGLE_REGION)
        {
            bfhdr += 4;
        }

        uint32_t idx = azimuth_bin - min_az + (elevation_bin - min_el) * (max_az - min_az + 1) + sample_idx;
        if (hdr->ext_det_flags & EXT_DET_GLOBAL_DOPPLER_REGION)
        {
            doppler_bin_Q11_4 = bfhdr[idx * 2 + 0];
            magnitude         = bfhdr[idx * 2 + 1];
            return true;
        }
        else
        {
            // TODO:
            return false;
        }
    }


    //! fills a memory buffer with RDC3 data from the specified range of indices.
    //! returns true if buffer was sufficiently large to hold all the output data
    //! any samples which are no longer available (because sparsification has
    //! discarded the data) will be stored as 0
    bool get_dense_samples(uint16_t* output_buffer, uint32_t buffer_size_shorts,
                           uint32_t min_range,     uint32_t max_range,
                           uint32_t min_doppler,   uint32_t max_doppler,
                           uint32_t min_azimuth,   uint32_t max_azimuth,
                           uint32_t min_elevation, uint32_t max_elevation) const
    {
        if (!is_ok())
        {
            return false;
        }
        if ((min_elevation > max_elevation) || (min_azimuth > max_elevation) ||
            (min_doppler > max_doppler) || (min_range > max_range))
        {
            return false;
        }
        uint32_t E = (max_elevation - min_elevation + 1);
        uint32_t A = (max_azimuth   - min_azimuth   + 1);
        uint32_t D = (max_doppler   - min_doppler   + 1);
        uint32_t R = (max_range     - min_range   + 1);
        uint32_t num_samples = E * A * D * R;
        if (!output_buffer || (buffer_size_shorts < num_samples))
        {
            return false;
        }

        memset(output_buffer, 0, sizeof(output_buffer[0]) * num_samples);

        for (uint32_t r = min_range; r <= max_range; r++)
        {
            for (uint32_t e = min_elevation; e <= max_elevation; e++)
            {
                for (uint32_t a = min_azimuth; a <= max_azimuth; a++)
                {
                    uint32_t c = get_num_samples_at_range_angle(r, a, e);

                    for (uint32_t s = 0; s < c; s++)
                    {
                        uint16_t mag;
                        uint16_t dop_Q11;

                        if (get_range_angle_sample(r, a, e, s, mag, dop_Q11))
                        {
                            uint16_t d = dop_Q11 >>= 4;
                            if ((d >= min_doppler) && (d <= max_doppler))
                            {
                                output_buffer[e * A * D + a * D + d - min_doppler] = mag;
                            }
                        }
                    }
                }
            }

            output_buffer += E * A * D;
        }

        return true;
    }


protected:

    const int8_t*   buffer;
    const uint16_t* range_bin_offsets;
    uint32_t        size_bytes;
    uint16_t        min_range_bin;
    uint16_t        max_range_bin;
    uint16_t        min_doppler_bin;
    uint16_t        max_doppler_bin;
    uint16_t        min_azimuth_bin;
    uint16_t        max_azimuth_bin;
    uint16_t        min_elevation_bin;
    uint16_t        max_elevation_bin;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RDC_EXTENDED_DET_H
