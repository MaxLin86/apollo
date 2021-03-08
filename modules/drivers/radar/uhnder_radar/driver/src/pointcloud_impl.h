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
#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/pointcloud.h"

struct PointCloudData;
class  ScanObjectImpl;

#define POINT_CLOUD_HOST_CALCULATION 1

class PointCloud_Impl : public PointCloud
{
public:

    PointCloud_Impl(ScanObject_Impl& scan)
        : myscan(scan)
        , points(NULL)
        , num_points(0)
        , aborted(false)
        , last_err(PC_NO_ERROR)
    {
        if (!thresh_init)
        {
            UhdpThresholdControl t;
            t.defaults();
            configure_thresholds(t);
        }
    }

    virtual ~PointCloud_Impl();

    virtual uint32_t              get_count() const  { return num_points; }

    virtual void                  get_points(Point* output) const;

    virtual void                  apply_threshold(float static_min_snr, float moving_min_snr);

    virtual void                  release();

    virtual Err                   get_last_error() const { return last_err; }

            bool                  deserialize(ScanSerializer&);

            bool                  serialize(ScanSerializer&) const;

            void                  handle_uhdp(PointCloudData* p, uint32_t total_size);

            void                  setup();

#if POINT_CLOUD_HOST_CALCULATION
    void apply_sva(const float* in, float* out, uint32_t num, uint8_t ovsf, bool wrap, uint32_t stride);

    void apply_jat(float* thresh_out, const float* samples, uint32_t num, float noise_floor, float base_threshold);

    void apply_jat_2D(float* thresh_out, const float* samples, uint32_t AZ, uint32_t EL, float noise_floor, float base_threshold);

    uint32_t peak_find(const float* in, const float* jat_thresh, const float* orig, uint32_t num, float noise_floor, uint16_t range, int16_t doppler, PointCloudData* output, uint32_t count, uint32_t max_count, bool wrap);

    uint32_t peak_find_2D(const float* in, const float* jat_thresh, const float* orig, uint32_t AZ, uint32_t EL, float noise_floor, uint16_t range, int16_t doppler, PointCloudData* output, uint32_t count, uint32_t max_count, bool wrap_az, bool wrap_el);

    uint32_t process_skewer(const UhdpRangeBinInfo& rbinfo, const uint16_t* magnitudes, int16_t exp, PointCloudData* output, uint32_t count, uint32_t max_count, int16_t doppler, float detection_thresh);

    uint32_t extract_points(PointCloudData* output, uint32_t max_count, float static_detection_thresh, float moving_detection_thresh);

    void detect_ridges();

    float               azimuths_rad[MAX_MAX_ROUGH_ANGLES];
    float               elevations_rad[MAX_MAX_ROUGH_ANGLES];
    float               angle_noise_floor[MAX_MAX_ROUGH_ANGLES];
    uint16_t            angle_base_doppler[MAX_MAX_ROUGH_ANGLES];

    static bool         thresh_init;

    static UhdpThresholdControl thresh;

    static void         configure_thresholds(const UhdpThresholdControl& t) { thresh = t; thresh_init = true; }
#endif

    ScanObject_Impl&    myscan;

    PointCloudData*     points;

    uint32_t            num_points;

    bool                aborted;

    mutable Err         last_err;
};
