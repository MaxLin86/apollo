#include "modules/drivers/radar/uhnder_radar/driver/include/sra.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/scanning.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/scanobject.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/activations.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/staticslice.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/serializer.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/pointcloud_impl.h"

#include"modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"

#if POINT_CLOUD_HOST_CALCULATION
#include <assert.h>

#define SHOW_RIDGE_HISTOGRAMS     0

// internal data structures and defines

enum { NOISE = 0x7FFFFFFF };
enum { MAG2DB_MAXUINT16 = 24 };

UhdpThresholdControl PointCloud_Impl::thresh;
bool PointCloud_Impl::thresh_init;

// returns interpolated angle bin index - TODO: convert to dB
static float quadratic_interp(
        float& interpmag,      // output - interpolated magnitude
        uint32_t centerbin,    // center angle bin index
        float yl,              // left (or lower) sample value magnitude
        float yc,              // center sample value magnitude
        float yu,              // right (or upper) sample value magnitude
        float max_value)       // maximum bin value (number of bins)
{
    float d1 = (yu - yl) / 2;
    float d2 = yu - yc + yl - yc;

    if (d2)
    {
        float dd = d1 / d2;
        interpmag = yc - d1 * dd / 2.0f;
        float val = (float)centerbin - dd;
        if (val < 0.0f)
        {
            /* if value is negative, assume it has wrapped */
            val += max_value;
            assert(val > 0 && val < max_value);
        }
        else if (val > max_value)
        {
            val = max_value;
        }
        return val;
    }
    else
    {
        interpmag = yc;
        return centerbin;
    }
}

#if SHOW_RIDGE_HISTOGRAMS
static void dump_hist(uint32_t* hist, uint64_t* sum, uint32_t size)
{
    uint16_t lastbin = 0;
    uint16_t maxval = 0;
    for (uint16_t h = 0; h < size; h++)
    {
        if (hist[h] > maxval)
        {
            maxval = hist[h];
        }
        if (hist[h] > 0)
        {
            lastbin = h;
        }
    }

    char evbuf[42];
    for (uint16_t h = 0; h <= lastbin; h++)
    {
        int n = hist[h] * 40 / maxval;
        int j;
        for (j = 0; j < n; j++)
        {
            evbuf[j] = '=';
        }

        evbuf[j] = 0;
        printf("Hist:  %5d: %5d % 6.2f%% |%s|\n",
               h,
               hist[h],
               99.9f - (100.0f * sum[h]) / sum[size - 1],
               evbuf);
    }
}
#endif

void PointCloud_Impl::detect_ridges()
{
    const StaticSlice* ss   = myscan.get_static_slice();
    const UhdpScanInformation& info = myscan.scan_info;
    const UhdpRangeBinInfo* range_bins = myscan.range_bins;

    uint32_t histogram[MAX_MAX_ROUGH_ANGLES][MAG2DB_MAXUINT16];
    memset(histogram, 0, sizeof(histogram));

    int prev[MAX_MAX_ROUGH_ANGLES];
    memset(prev, 0, sizeof(prev));

    uint32_t AZ = info.num_azimuth_angles;
    uint32_t EL = info.num_beamforming_angles / AZ;
    uint32_t num_slices = ss->get_num_slices();

    for (uint32_t r = 0; r < info.num_range_bins; r++)
    {
        int16_t exp = ss->get_exponent_at_range(r);
        const uint16_t* magnitudes = ss->get_skewer(r, num_slices / 2);

        float fexp = (exp >= 0) ? (float)(1U << exp) : 1.0f / (1U << -exp);

        // add each sample magnitude
        for (uint32_t a = 0; a < AZ * EL; a++)
        {
            float magdb = mag2db((float)magnitudes[a] * fexp);
            int mgdb = (int)uh_fmaxf(magdb - range_bins[r].noise_floor_max_peak_dB + 0.5f, 0.0f);
            mgdb = uh_fminf(uh_fmaxf(mgdb, 0), MAG2DB_MAXUINT16-1);  // clamp mgdb into histogram range
            if (a)
            {
                if (prev[a] > mgdb)
                {
                    for (int j = mgdb; j <= prev[a] - 1; j++)
                    {
                        histogram[a][j]++;
                    }
                }
                else if (prev[a] < mgdb)
                {
                    for (int j = prev[a] + 1; j <= mgdb; j++)
                    {
                        histogram[a][j]++;
                    }
                }
                else
                {
                    histogram[a][mgdb]++;
                }
            }
            else
            {
                histogram[a][mgdb]++;
            }
            prev[a] = mgdb;
        }
    }

    for (uint32_t a = 0; a < AZ * EL; a++)
    {
        uint64_t sum[MAG2DB_MAXUINT16];
        memset(sum, 0, sizeof(sum));

        uint16_t peak_val = 0, peak_bin = 0;
        uint64_t total = 0;
        for (uint32_t b = 0; b < MAG2DB_MAXUINT16; b++)
        {
            if (histogram[a][b] > peak_val)
            {
                peak_bin = b;
                peak_val = histogram[a][b];
            }
            sum[b] = total;
            total += histogram[a][b];
        }

        if (peak_bin > 3)
        {
            angle_noise_floor[a] = db2mag(peak_bin);

#if SHOW_RIDGE_HISTOGRAMS
            if (peak_bin)
            {
                printf("Peak: %d Az %d\n", peak_bin, a);
                dump_hist(histogram[a], sum, MAG2DB_MAXUINT16);
            }
#endif
        }
        else
        {
            angle_noise_floor[a] = 0;
        }
    }
}

void PointCloud_Impl::apply_sva(
        const float* in,    //< input sample magnitudes
        float* out,         //< SVA sample magnitude outputs
        uint32_t num,       //< number of input samples
        uint8_t ovsf,       //< Nyquist oversampling factor (ratio)
        bool wrap,          //< FOV covers entire unambiguous angle range, so wrap-around
        uint32_t stride)    //< stride of samples in input and output arrays (for elevation)
{
    uint32_t first, last;
    if (wrap)
    {
        first = 0;
        last = num - 1;
    }
    else
    {
        first = ovsf;
        last  = num - ovsf - 1;

        for (uint32_t i = 0; i < first; i++)  out[i * stride] = 0;
        for (uint32_t i = last; i < num; i++) out[i * stride] = 0;
    }

    for (uint32_t i = first; i <= last; i++)
    {
        // safely wrap indices first-to-last or last-to-first, "num + i" avoids negative modulo
        float divisor = in[((num + i - ovsf) % num) * stride] +
                        in[((i + ovsf) % num) * stride];
        if (divisor)
        {
            float wmsel = in[i * stride] / divisor;
            wmsel = uh_fminf(wmsel, 0.5f);
            wmsel = uh_fmaxf(wmsel, 0.0f);
            out[i * stride] =  in[i * stride] - (wmsel * divisor);
        }
        else
        {
            // don't trust inf behavior in uh_fminf
            out[i * stride] =  in[i * stride];
        }
        assert(out[i * stride] < 3.51399E+30);
    }
}


/* This function raises the detection threshold at various angles (without
 * affecting SNR) to account for the angle side-lobes of large targets.
 *
 * If you look at the side-lobe pattern for our current antennas there is about
 * 30dB (A) of clearance around a large magnitude target (minus some random spikes
 * around 5 degrees off-center), and then there are serious side-lobes about
 * 20dB (B) down after about 30 (C) degrees off-center. We raise the detection
 * thresholds at those angles to avoid side-lobes.
 *
 *         |   |   *
 *         B   |   *
 *         |   A   *
 * ----------+ |   *     +------------
 *           | |   *     |
 *           +-----*-----+
 *
 *           |-----C-----| */
void PointCloud_Impl::apply_jat(
        float* thresh_out,        //< output per-angle threshold array
        const float* samples,     //< input sample magnitudes
        uint32_t num,             //< number of input samples
        float noise_floor,
        float base_threshold)
{
    const float ridge_threshold = db2mag(thresh.ridge_threshold_dB);
    // A - the amount of clean dB near a large target (as linear) : within 27 degrees, allow targets 30dB down
    const float notch_depth = db2mag(thresh.notch_depth_dB);
    // B - the amount of clean dB at off-angles (as linear)       : beyond 27 degress, allow targets 22dB down
    const float outer_depth = db2mag(thresh.outer_depth_dB);

    for (uint32_t i = 0; i < num; i++)
    {
        thresh_out[i] = noise_floor * uh_fmaxf(angle_noise_floor[i] * ridge_threshold, base_threshold);
    }

    float min_thresh = uh_fminf(outer_depth, notch_depth);

    // raise thresholds around large magnitude signals
    for (uint32_t i = 0; i < num; i++)
    {
        if (samples[i] > noise_floor * base_threshold * min_thresh)
        {
            // this sample is big enough to raise the noise floor for other angle bins
            for (int32_t x = (int32_t)i - 1; x >= 0; x--)
            {
                float az_delta = azimuths_rad[i] - azimuths_rad[x];
                float t = (az_delta < thresh.notch_width_radians) ? notch_depth : outer_depth;
                thresh_out[x] = uh_fmaxf(thresh_out[x], samples[i] / t);
            }
            for (uint32_t x = i + 1; x < num; x++)
            {
                float az_delta = azimuths_rad[x] - azimuths_rad[i];
                float t = (az_delta < thresh.notch_width_radians) ? notch_depth : outer_depth;
                thresh_out[x] = uh_fmaxf(thresh_out[x], samples[i] / t);
            }
        }
    }
}

// Same as apply_jat() except it handles azimuth and elevation side-lobes
void PointCloud_Impl::apply_jat_2D(
        float* thresh_out,
        const float* samples,
        uint32_t AZ,
        uint32_t EL,
        float noise_floor,
        float base_threshold)
{
    const float ridge_threshold = db2mag(thresh.ridge_threshold_dB);
    // TODO: Add two-D settings to UhdpThresholdControl

    // C - the angle at which larger side-lobes appear
    static const float notch_width_az_rad = deg2rad(5.0f);
    // C - the angle at which larger side-lobes appear
    static const float notch_width_el_rad = deg2rad(4.0f);
    // A - the amount of clean dB near a large target (as linear) : within 27 degrees, allow targets 30dB down
    static const float notch_depth = db2mag(14.0f);
    // B - the amount of clean dB at off-angles (as linear)       : beyond 27 degress, allow targets 22dB down
    static const float outer_depth = db2mag(12.0f);

    for (uint32_t i = 0; i < AZ * EL; i++)
    {
        thresh_out[i] = noise_floor * uh_fmaxf(angle_noise_floor[i] * ridge_threshold, base_threshold);
    }

    float min_thresh = uh_fminf(outer_depth, notch_depth);

    // raise thresholds around large magnitude signals - TODO: handle wrapping?
    for (uint32_t i = 0; i < AZ * EL; i++)
    {
        if (samples[i] > noise_floor * base_threshold * min_thresh)
        {
            uint32_t e = i / AZ;
            uint32_t a = i - e * AZ;

            for (uint32_t xe = 0; xe < EL; xe++)
            {
                float el_delta = fabs(elevations_rad[i] - elevations_rad[xe * AZ]);
                float et = (el_delta < notch_width_el_rad) ? notch_depth : outer_depth;

                // this sample is big enough to raise the noise floor for other angle bins
                for (uint32_t xa = 0; xa < AZ; xa++)
                {
                    if (xe == e && xa == a) continue;

                    uint32_t c = xe * AZ + xa;

                    float az_delta = fabs(azimuths_rad[i] - azimuths_rad[c]);
                    float t = (az_delta < notch_width_az_rad) ? et : outer_depth;
                    thresh_out[c] = uh_fmaxf(thresh_out[c], samples[i] / t);
                }
            }
        }
    }
}


// returns updated count of output point detections
uint32_t PointCloud_Impl::peak_find(
        const float* in,                  //< SVA filtered samples
        const float* jat_thresh,          //< angle-sidelobe filter thresholds
        const float* orig,                //< original unfiltered samples
        uint32_t num,                     //< sample count for all three arrays
        float noise_floor,                //< range based noise floor magnitude
        uint16_t range,                   //< range (m) to use in detections
        int16_t doppler,                  //< doppler (m/s) to use in detections
        PointCloudData* output,           //< storage for output point detections
        uint32_t count,                   //< count of output points already consumed
        uint32_t max_count,               //< allocated size of output array
        bool wrap)                        //< wrap-around interpolation in azimuth
{
    if (count >= max_count)
    {
        return max_count;
    }

    float angle_nf[MAX_MAX_ROUGH_ANGLES]; // combined noise floor at each angle
    for (uint32_t a = 0; a < num; a++)
    {
        angle_nf[a] = noise_floor * angle_noise_floor[a];
    }

    for (uint32_t a = 0; a < num; a++)
    {
        uint32_t left, right;
        if (a + 2 < num)
        {
            right = a + 1;
        }
        else if (wrap)
        {
            right = 0;
        }
        else
        {
            right = NOISE;
        }
        if (a)
        {
            left = a - 1;
        }
        else if (wrap)
        {
            left = num - 1;
        }
        else
        {
            left = NOISE;
        }

        // Note that we use the SVA filtered magnitudes for threshold checking
        // (to discard side-lobes) but then we use the original magnitudes for
        // peak finding
        float center = orig[a];
        float thresh = jat_thresh[a];
        if (in[a] > thresh &&
            (left == NOISE  || center >= orig[left]) &&
            (right == NOISE || center >  orig[right]))
        {
            float interp_mag;
            float lv = (left == NOISE)  ? uh_fminf(angle_nf[a], orig[right]) : orig[left];
            float rv = (right == NOISE) ? uh_fminf(angle_nf[a], orig[left]) : orig[right];
            float az = quadratic_interp(interp_mag, a, lv, center, rv, num);

            output[count].azimuth_fbin = (uint16_t)(az * (1 << PointCloudData::AZIMUTH_FRAC_BITS));
            output[count].elevation_fbin = 0;
            output[count].snr_dB = (uint16_t)(mag2db(interp_mag / noise_floor) * (1 << PointCloudData::SNR_FRAC_BITS));
            output[count].range = range;
            output[count].doppler_bin = doppler + angle_base_doppler[a];
            count++;

            if (count >= max_count)
            {
                return count;
            }
        }
    }

    return count;
}


// returns updated count of output point detections
uint32_t PointCloud_Impl::peak_find_2D(
        const float* in,                  //< SVA filtered samples
        const float* jat_thresh,          //< angle-sidelobe filter thresholds
        const float* orig,                //< original unfiltered samples
        uint32_t AZ, uint32_t EL,         //< sample count for all three arrays
        float noise_floor,                //< range based noise floor magnitude
        uint16_t range,                   //< range (m) to use in detections
        int16_t doppler,                  //< doppler (m/s) to use in detections
        PointCloudData* output,           //< storage for output point detections
        uint32_t count,                   //< count of output points already consumed
        uint32_t max_count,               //< allocated size of output array
        bool wrap_az,                     //< wrap-around interpolation in azimuth
        bool wrap_el)                     //< wrap-around interpolation in elevation
{
    if (count >= max_count)
    {
        return max_count;
    }

    float rafloor[MAX_MAX_ROUGH_ANGLES]; // combined range/angle noise floor at each angle
    for (uint32_t a = 0; a < AZ * EL; a++)
    {
        rafloor[a] = noise_floor * angle_noise_floor[a];
    }

    for (uint32_t e = 0; e < EL; e++)
    {
        uint32_t down_el = e > 0      ? e - 1 : (wrap_el ? EL - 1 : NOISE);
        uint32_t up_el   = e + 1 < EL ? e + 1 : (wrap_el ? 0      : NOISE);

        for (uint32_t a = 0; a < AZ; a++)
        {
            uint32_t left, right;
            if (a + 2 < AZ)
            {
                right = e * AZ + a + 1;
            }
            else if (wrap_az)
            {
                right = e * AZ + 0;
            }
            else
            {
                right = NOISE;
            }
            if (a)
            {
                left = e * AZ + a - 1;
            }
            else if (wrap_az)
            {
                left = e * AZ + AZ - 1;
            }
            else
            {
                left = NOISE;
            }

            // Note that we use the SVA filtered magnitudes for threshold checking
            // (to discard side-lobes) but then we use the original magnitudes for
            // peak finding
            float center = orig[e * AZ + a];
            float thresh = jat_thresh[e * AZ + a];
            if (in[e * AZ + a] > thresh &&
                (left == NOISE    || center >= orig[left]) &&
                (right == NOISE   || center >  orig[right]) &&
                (down_el == NOISE || center >= orig[down_el * AZ + a]) &&
                (up_el == NOISE   || center >  orig[up_el * AZ + a]))
            {
                float az_interp_mag;
                float lv = (left == NOISE)  ? uh_fminf(rafloor[e * AZ + a], orig[right]) : orig[left];
                float rv = (right == NOISE) ? uh_fminf(rafloor[e * AZ + a], orig[left]) : orig[right];
                float az = quadratic_interp(az_interp_mag, a, lv, center, rv, AZ);


                float el_interp_mag;
                float bv = (down_el == NOISE) ? uh_fminf(rafloor[e * AZ + a], orig[up_el * AZ + a]) : orig[down_el * AZ + a];
                float tv = (up_el   == NOISE) ? uh_fminf(rafloor[e * AZ + a], orig[down_el * AZ + a]) : orig[up_el * AZ + a];
                float el = quadratic_interp(el_interp_mag, e, bv, center, tv, EL);


                output[count].azimuth_fbin = (uint16_t)(az * (1 << PointCloudData::AZIMUTH_FRAC_BITS));
                output[count].elevation_fbin = (uint16_t)(el * (1 << PointCloudData::ELEVATION_FRAC_BITS));
                output[count].snr_dB = (uint16_t)(mag2db(0.5f * (az_interp_mag + el_interp_mag) / noise_floor) *
                        (1 << PointCloudData::SNR_FRAC_BITS));
                output[count].range = range;
                output[count].doppler_bin = doppler + angle_base_doppler[a];
                count++;

                if (count >= max_count)
                {
                    return count;
                }
            }
        }
    }

    return count;
}

uint32_t PointCloud_Impl::process_skewer(
        const UhdpRangeBinInfo&    rbinfo,
        const uint16_t*            magnitudes,
        int16_t                    exp,
        PointCloudData*            output,
        uint32_t                   count,
        uint32_t                   max_count,
        int16_t                    doppler,
        float                      detection_thresh)
{
    if (rbinfo.distance_in_bins < 0)
    {
        return count;
    }

    const UhdpScanInformation& info = myscan.scan_info;

    uint32_t AZ = info.num_azimuth_angles;
    uint32_t EL = info.num_beamforming_angles / AZ;

    uint16_t range = rbinfo.distance_in_bins;
    float range_noise_floor_peak_max = db2mag(rbinfo.noise_floor_max_peak_dB);

    float float_mag[MAX_MAX_ROUGH_ANGLES];  // skewer magnitude casted as float (RDC3 HW exponent included)
    float sva_az_out[MAX_MAX_ROUGH_ANGLES]; // output of SVA applied across azimuth
    float sva_el_out[MAX_MAX_ROUGH_ANGLES]; // output of SVA applied across elevation
    float jat_thresh[MAX_MAX_ROUGH_ANGLES]; // output of JAT side-lobe supression thresholding

    // uint16_t magnitude[info.num_beamforming_angles]
    float fexp = (exp >= 0) ? (float)(1U << exp) : 1.0f / (1U << -exp);
    for (uint32_t i = 0; i < AZ * EL; i++)
    {
        float_mag[i] = (float)magnitudes[i] * fexp;
    }

    if (EL > 1) // TWO-D
    {
        uint32_t azimuth_nyquist_oversampling_factor = info.azimuth_nyquist_oversampling_factor;
        uint32_t elevation_nyquist_oversampling_factor = info.elevation_nyquist_oversampling_factor;
        if (azimuth_nyquist_oversampling_factor >= 1)
        {
            for (uint32_t e = 0; e < EL; e++)
            {
                apply_sva(float_mag + e * AZ, sva_az_out + e * AZ, AZ,
                          azimuth_nyquist_oversampling_factor,
                          info.angle_wrap_flags & 1, 1);
            }
        }
        else
        {
            memcpy(sva_az_out, float_mag, sizeof(float_mag));
        }
        if (elevation_nyquist_oversampling_factor >= 1)
        {
            for (uint32_t a = 0; a < AZ; a++)
            {
                apply_sva(sva_az_out + a, sva_el_out + a, EL,
                          elevation_nyquist_oversampling_factor,
                          info.angle_wrap_flags & 2, AZ);
            }
        }
        else
        {
            memcpy(sva_el_out, sva_az_out, sizeof(sva_az_out));
        }

        apply_jat_2D(jat_thresh, sva_el_out, AZ, EL, range_noise_floor_peak_max, detection_thresh);

        count = peak_find_2D(sva_el_out, jat_thresh, float_mag, AZ, EL,
                             range_noise_floor_peak_max,
                             range, doppler, output, count, max_count,
                             info.angle_wrap_flags & 1, info.angle_wrap_flags & 2);
    }
    else
    {
        if (info.azimuth_nyquist_oversampling_factor >= 1)
        {
            apply_sva(float_mag, sva_az_out, info.num_beamforming_angles,
                      info.azimuth_nyquist_oversampling_factor,
                      info.angle_wrap_flags & 1, 1);
        }
        else
        {
            memcpy(sva_az_out, float_mag, sizeof(float_mag));
        }

        apply_jat(jat_thresh, sva_az_out, info.num_beamforming_angles, range_noise_floor_peak_max, detection_thresh);

        count = peak_find(sva_az_out, jat_thresh, float_mag, info.num_beamforming_angles,
                          range_noise_floor_peak_max,
                          range, doppler, output, count, max_count, info.angle_wrap_flags & 1);
    }

    return count;
}


uint32_t PointCloud_Impl::extract_points(
        PointCloudData* output,
        uint32_t max_count,
        float static_detection_thresh,
        float moving_detection_thresh)
{
    const UhdpScanInformation& info = myscan.get_scan_info();
    if (info.overflow_underflow_flags & 1) // check for CIU overflow
    {
        return 0;
    }

    for (uint32_t i = 0; i < info.num_beamforming_angles; i++)
    {
        myscan.get_angle(i, azimuths_rad[i], elevations_rad[i]);
        angle_noise_floor[i] = 0;
    }
    // safety check that angle-bins were received for this scan
    if (myscan.get_last_error() != ScanObject::SCAN_NO_ERROR)
    {
        return 0;
    }

    const StaticSlice* ss   = myscan.get_static_slice();
    const Activations* acts = myscan.get_dynamic_activations();
    const uint16_t* ss_zero_d = myscan.get_zero_doppler_bins();
    const UhdpRangeBinInfo* range_bins = myscan.get_range_bins();

    if (ss && range_bins && thresh.ridge_threshold_dB > 0)
    {
        detect_ridges();
    }

    uint32_t count = 0;

    if (ss && range_bins && ss_zero_d)
    {
        for (uint32_t i = 0; i < info.num_beamforming_angles; i++)
        {
            angle_base_doppler[i] = ss_zero_d[i];
        }


        uint32_t num_slices = ss->get_num_slices();
        for (uint32_t slice = 0; slice < num_slices; slice++)
        {
            for (uint32_t r = 0; r < info.num_range_bins; r++)
            {
                int16_t exp = ss->get_exponent_at_range(r);
                const uint16_t* magnitudes = ss->get_skewer(r, slice);
                int16_t rel_dbin = slice - ((info.SS_size_D - 1) / 2);
                count = process_skewer(range_bins[r], magnitudes, exp,
                                       output, count, max_count, rel_dbin,
                                       db2mag(static_detection_thresh));
            }
        }
        for (uint32_t i = 0; i < count; i++)
        {
            output[i].flags = RDC_DET_FLAG_STATIC;
        }
    }

    if (acts && range_bins)
    {
        // RDC3 activations have no per-angle doppler offsets and currently have
        // no radial suppression
        for (uint32_t i = 0; i < info.num_beamforming_angles; i++)
        {
            angle_base_doppler[i] = 0;
        }

        float det_thresh_mag = db2mag(moving_detection_thresh);
        uint64_t quick_check = (uint64_t)db2mag(moving_detection_thresh - 3);

        uint32_t act_count = acts->get_count(Activations::THRESH_HI);
        for (uint32_t i = 0; i < act_count; i++)
        {
            int16_t exp;
            uint16_t rbin, dbin, max_val;
            const uint16_t* magnitudes = acts->get_raw_samples(Activations::THRESH_HI, i, exp, rbin, dbin, max_val);

            if (((uint64_t)max_val << exp) < quick_check)
                continue;

            count = process_skewer(range_bins[rbin], magnitudes, exp,
                                   output, count, max_count, dbin, det_thresh_mag);
        }

        act_count = acts->get_count(Activations::THRESH_LO);
        for (uint32_t i = 0; i < act_count; i++)
        {
            int16_t exp;
            uint16_t rbin, dbin, max_val;
            const uint16_t* magnitudes = acts->get_raw_samples(Activations::THRESH_LO, i, exp, rbin, dbin, max_val);

            if (((uint64_t)max_val << exp) < quick_check)
                continue;

            count = process_skewer(range_bins[rbin], magnitudes, exp,
                                   output, count, max_count, dbin, det_thresh_mag);
        }
    }

    return count;
}

#endif /* POINT_CLOUD_HOST_CALCULATION */


PointCloud_Impl::~PointCloud_Impl()
{
    delete [] points;
}


void  PointCloud_Impl::get_points(Point* output) const
{
    if (!output)
    {
        return;
    }

    float azimuths_rad[MAX_MAX_ROUGH_ANGLES];
    float elevations_rad[MAX_MAX_ROUGH_ANGLES];

    const UhdpScanInformation& info = myscan.scan_info;
    uint32_t AZ = info.num_azimuth_angles;
    uint32_t EL = info.num_beamforming_angles / AZ;

    for (uint32_t i = 0; i < info.num_beamforming_angles; i++)
    {
        myscan.get_angle(i, azimuths_rad[i], elevations_rad[i]);
    }
    // safety check that angle-bins were received for this scan
    if (myscan.get_last_error() != ScanObject::SCAN_NO_ERROR)
    {
        return;
    }

    for (uint32_t i = 0; i < num_points; i++)
    {
        PointCloudData& p = points[i];
        Point& o = output[i];

        o.range = p.range * info.range_bin_width;
        o.doppler = info.doppler_bin_width * (p.doppler_bin - (info.num_pulses / 2));
        o.mag_snr = (float)p.snr_dB / (1 << PointCloudData::SNR_FRAC_BITS);
        o.flags = p.flags;

        float az = (float)p.azimuth_fbin / (1 << PointCloudData::AZIMUTH_FRAC_BITS);
        int az_bin_lo = (int)az;
        float az_fract = az - az_bin_lo;
        float az_lo = azimuths_rad[az_bin_lo];
        float az_delta;
        if (az_bin_lo == (int)AZ - 1)
        {
            // assume a wrap situation, pretend there is an azimuth angle at -azimuth_angles[0]
            az_delta = azimuths_rad[1] - azimuths_rad[0];
        }
        else
        {
            az_delta = azimuths_rad[az_bin_lo + 1] - az_lo;
        }
        o.azimuth = az_lo + (az_delta * az_fract);

        float el = (float)p.elevation_fbin / (1 << PointCloudData::ELEVATION_FRAC_BITS);
        int el_bin_lo = (int)el;
        float el_fract = el - el_bin_lo;
        float el_lo = elevations_rad[el_bin_lo * AZ];
        float el_delta;
        if (el_bin_lo == (int)EL - 1)
        {
            // assume a wrap situation, pretend there is an azimuth angle at -azimuth_angles[0]
            el_delta = elevations_rad[AZ * 1] - elevations_rad[AZ * 0];
        }
        else
        {
            el_delta = elevations_rad[(el_bin_lo + 1) * AZ] - el_lo;
        }
        o.elevation = el_lo + el_delta * el_fract;
    }
}


void  PointCloud_Impl::apply_threshold(float static_min_snr, float moving_min_snr)
{
#if POINT_CLOUD_HOST_CALCULATION
    if (!points && (myscan.myact || myscan.myss))
    {
        enum { MAX_POINTS = 32 * 1024 };

        points = new PointCloudData[MAX_POINTS];
        memset(points, 0, sizeof(PointCloudData) * MAX_POINTS);
        num_points = extract_points(points, MAX_POINTS, static_min_snr, moving_min_snr);
        myscan.scan_info.total_points = num_points;
        return;
    }
#endif

    if (points)
    {
        uint16_t static_min = (uint16_t)(static_min_snr * (1 << PointCloudData::SNR_FRAC_BITS));
        uint16_t moving_min = (uint16_t)(moving_min_snr * (1 << PointCloudData::SNR_FRAC_BITS));

        uint32_t i = 0;
        while (i < num_points)
        {
            bool discard = false;

            if (points[i].flags & RDC_DET_FLAG_STATIC)
            {
                discard = points[i].snr_dB < static_min;
            }
            else
            {
                discard = points[i].snr_dB < moving_min;
            }

            if (discard)
            {
                points[i] = points[--num_points];
            }
            else
            {
                i++;
            }
        }

        myscan.scan_info.total_points = num_points;
    }
}


void  PointCloud_Impl::release()
{
    myscan.release_pointcloud(*this);
}


bool  PointCloud_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    bool ok = true;

    const UhdpScanInformation& info = myscan.scan_info;
    if (0 == info.total_points)
    {
        ok = false;
    }
    else
    {
        sprintf(fname, "scan_%06d_pointcloud.bin", info.scan_sequence_number);

        size_t len = s.begin_read_scan_data_type(fname);

        if (len == sizeof(PointCloudData) * info.total_points)
        {
            points = new PointCloudData[info.total_points];
            ok &= s.read_scan_data_type(points, sizeof(points[0]), info.total_points);
            s.end_read_scan_data_type();
            num_points = info.total_points;
        }
        else
        {
            ok = false;
        }
    }

    return ok;
}


bool  PointCloud_Impl::serialize(ScanSerializer& s) const
{
    bool ok = true;

    if (points)
    {
        char fname[128];
        const UhdpScanInformation& info = myscan.scan_info;

        sprintf(fname, "scan_%06d_pointcloud.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(points, sizeof(points[0]), num_points);
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}


void  PointCloud_Impl::handle_uhdp(PointCloudData* p, uint32_t total_size)
{
    if (aborted)
    {
        return;
    }

    uint32_t count = total_size / sizeof(points[0]);
    if (count * sizeof(points[0]) != total_size)
    {
        aborted = true;
        return;
    }

    if (count + num_points > myscan.scan_info.total_points)
    {
        aborted = true;
        return;
    }

    if (!points)
    {
        points = new PointCloudData[myscan.scan_info.total_points];
    }

    memcpy(points + num_points, p, total_size);
    num_points += count;
}


void  PointCloud_Impl::setup()
{
    if (num_points != myscan.scan_info.total_points)
    {
        aborted = true;
    }

    if (aborted)
    {
        myscan.release_pointcloud(*this);
    }
}
}
}
}n_info.total_points)
    {
        aborted = true;
    }

    if (aborted)
    {
        myscan.release_pointcloud(*this);
    }
}
