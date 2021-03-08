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
#include "modules/drivers/radar/uhnder_radar/driver/src/activations_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/serializer.h"
#include"modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"


struct Skewer
{
    RHAL_RDsummary  hdr;
    uint16_t        data[MAX_MAX_ROUGH_ANGLES];
};


Activations_Impl::BucketData::~BucketData()
{
    delete [] skewers;
}

void     Activations_Impl::handle_uhdp(UhdpRDC3Header* rdc3hdr, uint32_t total_size)
{
    if (aborted)
    {
        return;
    }
    if (rdc3hdr->angle_group_id)
    {
        printf("multiple angle groups not supported by this API\n");
        aborted = true;
        return;
    }
    if (rdc3hdr->per_tuple_size == sizeof(cint16) * myscan.scan_info.num_beamforming_angles)
    {
        printf("Complex RDC3 data not supported by this API\n");
        aborted = true;
        return;
    }
    if (rdc3hdr->bucket_index >= NUM_RD_BUF)
    {
        aborted = true;
        return;
    }
    if (rdc3hdr->per_tuple_size != sizeof(uint16_t) * myscan.scan_info.num_beamforming_angles)
    {
        printf("Unexpected RDC3 skewer size %d\n", rdc3hdr->per_tuple_size);
        aborted = true;
        return;
    }

    if (myscan.scan_info.num_beamforming_angles > MAX_MAX_ROUGH_ANGLES)
    {
        aborted = true;
        return;
    }

    BucketData& bucket = buckets[rdc3hdr->bucket_index];
    if (!bucket.skewers)
    {
        if (bucket.num_skewers || bucket.total_skewers)
        {
            aborted = true;
            return;
        }

        bucket.total_skewers = rdc3hdr->total_tuple_in_bucket;
        bucket.skewers = new Skewer[bucket.total_skewers];
    }

    total_size -= sizeof(UhdpRDC3Header);
    char* payload = (char*)(rdc3hdr + 1);

    // Consume bucket data, which is the summary data followed by the skewer data
    while (bucket.num_sum < bucket.total_skewers && total_size >= sizeof(RHAL_RDsummary))
    {
        bucket.skewers[bucket.num_sum++].hdr = *(RHAL_RDsummary*)payload;
        payload += sizeof(RHAL_RDsummary);
        total_size -= sizeof(RHAL_RDsummary);
    }
    while (bucket.num_skewers < bucket.total_skewers && total_size >= rdc3hdr->per_tuple_size)
    {
        memcpy(bucket.skewers[bucket.num_skewers++].data, payload, rdc3hdr->per_tuple_size);
        payload += rdc3hdr->per_tuple_size;
        total_size -= rdc3hdr->per_tuple_size;
    }

    if (total_size)
    {
        printf("RDC3 unexpected packet length, aborting\n");
        aborted = true;
    }
}


void     Activations_Impl::setup()
{
    if (aborted)
    {
        myscan.release_activations(*this);
        return;
    }

    for (uint32_t i = 0; i < NUM_RD_BUF; i++)
    {
        if (buckets[i].num_skewers != buckets[i].total_skewers ||
            buckets[i].num_sum != buckets[i].total_skewers)
        {
            myscan.release_activations(*this);
            return;
        }
    }

    if (!myscan.range_bins || !myscan.angle_bins)
    {
        printf("Activations are missing range or angle data, discarding\n");
        myscan.release_activations(*this);
        return;
    }
}


static const char* lbls = "lus";

bool     Activations_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    bool atleastone = false;
    bool ok = true;
    const UhdpScanInformation& info = myscan.scan_info;

    for (uint32_t i = 0; i < NUM_RD_BUF; i++)
    {
        BucketData& bucket = buckets[i];

        sprintf(fname, "scan_%06d_spsum%c.bin", info.scan_sequence_number, lbls[i]);
        size_t len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            size_t count = len / sizeof(RHAL_RDsummary);
            if (count * sizeof(RHAL_RDsummary) == len)
            {
                bucket.total_skewers = count;
                bucket.num_skewers = count;
                bucket.skewers = new Skewer[count];
            }
            else
            {
                printf("Wrong size summary file\n");
                return false;
            }

            for (uint32_t sk = 0; sk < count; sk++)
            {
                ok &= s.read_scan_data_type(&bucket.skewers[sk].hdr, sizeof(RHAL_RDsummary), 1);
            }
            s.end_read_scan_data_type();
        }
        else
        {
            continue;
        }

        sprintf(fname, "scan_%06d_sprdc3%c.bin", info.scan_sequence_number, lbls[i]);
        len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            for (uint32_t sk = 0; sk < bucket.total_skewers; sk++)
            {
                ok &= s.read_scan_data_type(bucket.skewers[sk].data, sizeof(uint16_t),
                        info.num_beamforming_angles);
            }
            s.end_read_scan_data_type();
            atleastone = ok;
        }
    }

    return atleastone && ok;
}


bool     Activations_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];
    bool ok = true;
    const UhdpScanInformation& info = myscan.scan_info;

    for (uint32_t i = 0; i < NUM_RD_BUF; i++)
    {
        const BucketData& bucket = buckets[i];

        if (!bucket.total_skewers || !ok)
        {
            continue;
        }

        sprintf(fname, "scan_%06d_sprdc3%c.bin", info.scan_sequence_number, lbls[i]);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            for (uint32_t sk = 0; sk < bucket.total_skewers; sk++)
            {
                ok &= s.write_scan_data_type(bucket.skewers[sk].data, sizeof(uint16_t),
                       info.num_beamforming_angles);
            }
            s.end_write_scan_data_type(!ok);
        }

        if (!ok)
        {
            return false;
        }

        sprintf(fname, "scan_%06d_spsum%c.bin", info.scan_sequence_number, lbls[i]);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            for (uint32_t sk = 0; sk < bucket.total_skewers; sk++)
            {
                ok &= s.write_scan_data_type(&bucket.skewers[sk].hdr, sizeof(RHAL_RDsummary), 1);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}


uint32_t  Activations_Impl::get_count(BucketEnum bucket) const
{
    return buckets[bucket].num_skewers;
}

uint32_t  Activations_Impl::get_num_angle_bins() const
{
    return myscan.scan_info.num_beamforming_angles;
}

const uint16_t* Activations_Impl::get_raw_samples(
        BucketEnum bucket,
        uint32_t act_idx,
        int16_t&  exponent,
        uint16_t& range_bin,
        uint16_t& doppler_bin,
        uint16_t& max_val) const
{
    if (bucket > CUSTOM_RD || act_idx >= buckets[bucket].num_skewers)
    {
        last_err = ACT_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = ACT_NO_ERROR;
    exponent = buckets[bucket].skewers[act_idx].hdr.exponent;
    range_bin = buckets[bucket].skewers[act_idx].hdr.R_bin;
    doppler_bin = buckets[bucket].skewers[act_idx].hdr.D_bin;
    max_val = buckets[bucket].skewers[act_idx].hdr.max_val;
    return buckets[bucket].skewers[act_idx].data;
}


ActivationIterator*     Activations_Impl::make_iterator(float thresh_snr) const
{
    return new ActivationIterator_Impl(*this, thresh_snr);
}


void     Activations_Impl::release()
{
    myscan.release_activations(*this);
}


bool ActivationIterator_Impl::get_sample(
        float& snr,
        float& range,
        float& doppler,
        float& azimuth,
        float& elevation)
{
    const UhdpRangeBinInfo* range_bins = myact.myscan.range_bins;
    const UhdpScanInformation& info = myact.myscan.scan_info;

    while (cur_bucket < NUM_RD_BUF)
    {
        const Activations_Impl::BucketData& bucket = myact.buckets[cur_bucket];

        while (cur_activation < bucket.num_skewers)
        {
            const Skewer& sk = bucket.skewers[cur_activation];

            range = range_bins[sk.hdr.R_bin].distance_in_bins * info.range_bin_width;

            while (range >= 0 && cur_vrx < info.num_beamforming_angles)
            {
                float magdb = mag2db(sk.data[cur_vrx]) + 6 * sk.hdr.exponent;
                snr = magdb - range_bins[sk.hdr.R_bin].noise_floor_max_peak_dB;

                if (snr > thresh_snr)
                {
                    doppler = ((int)sk.hdr.D_bin - (info.num_pulses / 2)) * info.doppler_bin_width;
                    azimuth = myact.myscan.get_azimuth_rad(cur_vrx);
                    elevation = myact.myscan.get_elevation_rad(cur_vrx);
                    cur_vrx++;
                    return true;
                }

                cur_vrx++;
            }

            cur_activation++;
            cur_vrx = 0;
        }

        cur_bucket++;
        cur_activation = 0;
    }

    return false;
}

