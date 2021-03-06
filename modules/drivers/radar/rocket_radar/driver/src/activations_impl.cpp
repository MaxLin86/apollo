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
#include "modules/drivers/radar/rocket_radar/driver/src/activations_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"

/* NB: SabineB doubled the number of "buckets" output by the radar, to support
 * the doubled proc hardware (two FEU, two RAU, etc).  The API for this class
 * pretends that there are still only one bucket per threshold (LO, HI, CUSTOM).
 * So this class must implicitly combine the two hardware output buckets into a
 * single "threshold" buffer.  Note that when we serialize the activations we
 * combine the doubled buffers into a single file per threshold type.  When we
 * deserialize activations we pretend as if they all arrived from the 'A' half
 * of each bucket, to keep the API consistent */

struct Skewer
{
    RDC_RDsummary   summary;
    uint16_t        data[MAX_MAX_ROUGH_ANGLES];
    cint16          cdata[MAX_MAX_ROUGH_ANGLES];
};


Activations_Impl::BucketData::~BucketData()
{
    delete [] skewers;
}

uint32_t  Activations_Impl::get_count(BucketEnum bucket) const
{
    uint32_t total_num_skewers = 0;

    switch (bucket)
    {
    case THRESH_HI:
        total_num_skewers = buckets[THRESH_HI_A].num_skewers + buckets[THRESH_HI_B].num_skewers;
        break;

    case THRESH_LO:
        total_num_skewers = buckets[THRESH_LO_A].num_skewers + buckets[THRESH_LO_B].num_skewers;
        break;

    case CUSTOM_RD:
        total_num_skewers = buckets[CUSTOM_RD_A].num_skewers + buckets[CUSTOM_RD_B].num_skewers;
        break;
    }

    return total_num_skewers;
}

uint32_t  Activations_Impl::get_num_angle_bins() const
{
    return myscan.scan_info.num_beamforming_angles;
}

bool     Activations_Impl::complex_data_available() const
{
    return !!(myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_ACT);
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
    if (rdc3hdr->bucket_index >= NUM_RD_BUF)
    {
        aborted = true;
        return;
    }

    if (complex_data_available())
    {
        if (rdc3hdr->per_tuple_size != sizeof(cint16) * myscan.scan_info.num_beamforming_angles)
        {
            printf("Unexpected RDC3 skewer size %d\n", rdc3hdr->per_tuple_size);
            aborted = true;
            return;
        }
    }
    else
    {
        if (rdc3hdr->per_tuple_size != sizeof(uint16_t) * myscan.scan_info.num_beamforming_angles)
        {
            printf("Unexpected RDC3 skewer size %d\n", rdc3hdr->per_tuple_size);
            aborted = true;
            return;
        }
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
    while (bucket.num_sum < bucket.total_skewers && total_size >= sizeof(RDC_RDsummary))
    {
        bucket.skewers[bucket.num_sum++].summary = *(RDC_RDsummary*)payload;
        payload += sizeof(RDC_RDsummary);
        total_size -= sizeof(RDC_RDsummary);
    }
    while (bucket.num_skewers < bucket.total_skewers && total_size >= rdc3hdr->per_tuple_size)
    {
        if (complex_data_available())
        {
            memcpy(bucket.skewers[bucket.num_skewers].cdata, payload, rdc3hdr->per_tuple_size);
            for (uint32_t a = 0; a < get_num_angle_bins(); a++)
            {
                bucket.skewers[bucket.num_skewers].data[a] = bucket.skewers[bucket.num_skewers].cdata[a].abs();
            }
            bucket.num_skewers++;
        }
        else
        {
            memcpy(bucket.skewers[bucket.num_skewers].data, payload, rdc3hdr->per_tuple_size);
            for (uint32_t a = 0; a < get_num_angle_bins(); a++)
            {
                bucket.skewers[bucket.num_skewers].cdata[a] = cint16(bucket.skewers[bucket.num_skewers].data[a], 0);
            }
            bucket.num_skewers++;
        }
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

    // ensure complex and magnitude versions are both available
    for (uint32_t bkt = 0; bkt < NUM_RD_BUF; bkt++)
    {
        BucketData& bucket = buckets[bkt];

        uint16_t rbin_start = 0;
        if (bkt == THRESH_HI_B || bkt == THRESH_LO_B || bkt == CUSTOM_RD_B)
        {
            rbin_start = myscan.scan_info.num_range_bins / 2;
        }

        for (uint32_t sk = 0; sk < bucket.num_skewers; sk++)
        {
            bucket.skewers[sk].summary.R_bin += rbin_start;

            if (complex_data_available())
            {
                for (uint32_t a = 0; a < get_num_angle_bins(); a++)
                {
                    bucket.skewers[sk].data[a] = bucket.skewers[sk].cdata[a].abs();
                }
            }
            else
            {
                for (uint32_t a = 0; a < get_num_angle_bins(); a++)
                {
                    bucket.skewers[sk].cdata[a] = cint16(bucket.skewers[sk].data[a], 0);
                }
            }
        }
    }
}

const uint16_t* Activations_Impl::get_raw_samples(
        BucketEnum bucket,
        uint32_t act_idx,
        int16_t&  exponent,
        uint16_t& range_bin,
        uint16_t& doppler_bin,
        uint16_t& max_val) const
{
    if (bucket > CUSTOM_RD || act_idx >= get_count(bucket))
    {
        last_err = ACT_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    RHAL_Rdc3_Bucket real_bucket = THRESH_LO_A;
    uint32_t         real_act_idx = 0;
    switch (bucket)
    {
    case THRESH_HI:
        if (act_idx < buckets[THRESH_HI_A].num_skewers)
        {
            real_bucket = THRESH_HI_A;
            real_act_idx = act_idx;
        }
        else
        {
            real_bucket = THRESH_HI_B;
            real_act_idx = act_idx - buckets[THRESH_HI_A].num_skewers;
        }
        break;

    case THRESH_LO:
        if (act_idx < buckets[THRESH_LO_A].num_skewers)
        {
            real_bucket = THRESH_LO_A;
            real_act_idx = act_idx;
        }
        else
        {
            real_bucket = THRESH_LO_B;
            real_act_idx = act_idx - buckets[THRESH_LO_A].num_skewers;
        }
        break;

    case CUSTOM_RD:
        if (act_idx < buckets[CUSTOM_RD_A].num_skewers)
        {
            real_bucket = CUSTOM_RD_A;
            real_act_idx = act_idx;
        }
        else
        {
            real_bucket = CUSTOM_RD_B;
            real_act_idx = act_idx - buckets[CUSTOM_RD_A].num_skewers;
        }
        break;
    }

    last_err = ACT_NO_ERROR;
    exponent = buckets[real_bucket].skewers[real_act_idx].summary.exponent;
    range_bin = buckets[real_bucket].skewers[real_act_idx].summary.R_bin;
    doppler_bin = buckets[real_bucket].skewers[real_act_idx].summary.D_bin;
    max_val = buckets[real_bucket].skewers[real_act_idx].summary.max_val;
    return buckets[real_bucket].skewers[real_act_idx].data;
}


const cint16* Activations_Impl::get_raw_samples_complex(
        BucketEnum bucket,
        uint32_t act_idx,
        int16_t&  exponent,
        uint16_t& range_bin,
        uint16_t& doppler_bin,
        uint16_t& max_val) const
{
    if (bucket > CUSTOM_RD || act_idx >= get_count(bucket))
    {
        last_err = ACT_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    RHAL_Rdc3_Bucket real_bucket = THRESH_LO_A;
    uint32_t         real_act_idx = 0;
    switch (bucket)
    {
    case THRESH_HI:
        if (act_idx < buckets[THRESH_HI_A].num_skewers)
        {
            real_bucket = THRESH_HI_A;
            real_act_idx = act_idx;
        }
        else
        {
            real_bucket = THRESH_HI_B;
            real_act_idx = act_idx - buckets[THRESH_HI_A].num_skewers;
        }
        break;

    case THRESH_LO:
        if (act_idx < buckets[THRESH_LO_A].num_skewers)
        {
            real_bucket = THRESH_LO_A;
            real_act_idx = act_idx;
        }
        else
        {
            real_bucket = THRESH_LO_B;
            real_act_idx = act_idx - buckets[THRESH_LO_A].num_skewers;
        }
        break;

    case CUSTOM_RD:
        if (act_idx < buckets[CUSTOM_RD_A].num_skewers)
        {
            real_bucket = CUSTOM_RD_A;
            real_act_idx = act_idx;
        }
        else
        {
            real_bucket = CUSTOM_RD_B;
            real_act_idx = act_idx - buckets[CUSTOM_RD_A].num_skewers;
        }
        break;
    }

    last_err = ACT_NO_ERROR;
    exponent = buckets[real_bucket].skewers[real_act_idx].summary.exponent;
    range_bin = buckets[real_bucket].skewers[real_act_idx].summary.R_bin;
    doppler_bin = buckets[real_bucket].skewers[real_act_idx].summary.D_bin;
    max_val = buckets[real_bucket].skewers[real_act_idx].summary.max_val;
    return buckets[real_bucket].skewers[real_act_idx].cdata;
}


void     Activations_Impl::release()
{
    myscan.release_activations(*this);
}

static const char* lbls = "lus";

bool     Activations_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    bool atleastone = false;
    bool ok = true;
    const UhdpScanInformation& info = myscan.scan_info;

    for (uint32_t i = 0; i <= CUSTOM_RD; i++)
    {
        BucketData& bucket = buckets[i * 2];

        sprintf(fname, "scan_%06d_spsum%c.bin", info.scan_sequence_number, lbls[i]);
        size_t len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            size_t count = len / sizeof(RDC_RDsummary);
            if (count * sizeof(RDC_RDsummary) == len)
            {
                bucket.total_skewers = count;
                bucket.num_skewers = count;
                bucket.num_sum = count;
                bucket.skewers = new Skewer[count];
            }
            else
            {
                printf("Wrong size summary file\n");
                return false;
            }

            for (uint32_t sk = 0; sk < count; sk++)
            {
                ok &= s.read_scan_data_type(&bucket.skewers[sk].summary, sizeof(RDC_RDsummary), 1);
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
            if (complex_data_available())
            {
                for (uint32_t sk = 0; sk < bucket.total_skewers; sk++)
                {
                    ok &= s.read_scan_data_type(bucket.skewers[sk].cdata, sizeof(cint16),
                            info.num_beamforming_angles);
                }
            }
            else
            {
                for (uint32_t sk = 0; sk < bucket.total_skewers; sk++)
                {
                    ok &= s.read_scan_data_type(bucket.skewers[sk].data, sizeof(uint16_t),
                            info.num_beamforming_angles);
                }
            }
            s.end_read_scan_data_type();
            atleastone |= ok;
        }
    }

    if (atleastone && ok)
    {
        setup();
    }

    return atleastone && ok;
}


bool     Activations_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];
    bool ok = true;
    const UhdpScanInformation& info = myscan.scan_info;

    for (uint32_t i = 0; i <= CUSTOM_RD; i++)
    {
        const BucketData& bucket1 = buckets[i * 2];
        const BucketData& bucket2 = buckets[i * 2 + 1];

        if (!get_count(BucketEnum(i)) || !ok)
        {
            continue;
        }

        sprintf(fname, "scan_%06d_sprdc3%c.bin", info.scan_sequence_number, lbls[i]);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            for (uint32_t sk = 0; sk < bucket1.total_skewers; sk++)
            {
                if (complex_data_available())
                {
                    ok &= s.write_scan_data_type(bucket1.skewers[sk].cdata, sizeof(cint16),
                            info.num_beamforming_angles);
                }
                else
                {
                    ok &= s.write_scan_data_type(bucket1.skewers[sk].data, sizeof(uint16_t),
                            info.num_beamforming_angles);
                }
            }
            for (uint32_t sk = 0; sk < bucket2.total_skewers; sk++)
            {
                if (complex_data_available())
                {
                    ok &= s.write_scan_data_type(bucket2.skewers[sk].cdata, sizeof(cint16),
                            info.num_beamforming_angles);
                }
                else
                {
                    ok &= s.write_scan_data_type(bucket2.skewers[sk].data, sizeof(uint16_t),
                            info.num_beamforming_angles);
                }
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
            for (uint32_t sk = 0; sk < bucket1.total_skewers; sk++)
            {
                ok &= s.write_scan_data_type(&bucket1.skewers[sk].summary, sizeof(RDC_RDsummary), 1);
            }
            for (uint32_t sk = 0; sk < bucket2.total_skewers; sk++)
            {
                ok &= s.write_scan_data_type(&bucket2.skewers[sk].summary, sizeof(RDC_RDsummary), 1);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}
