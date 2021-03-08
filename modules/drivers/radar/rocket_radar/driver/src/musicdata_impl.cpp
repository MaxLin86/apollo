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
#include "modules/drivers/radar/rocket_radar/driver/src/musicdata_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include <assert.h>

struct CovarianceData
{
    CovarianceData() : data(NULL) {}
    ~CovarianceData() { delete [] data; }

    uint16_t        doppler_bin_num;
    uint16_t        range_bin_num;
    uint16_t        matrix_size;
    cint64*         data;
};

struct SVDData
{
    SVDData() : data(NULL) {}
    ~SVDData() { delete [] data; }

    uint16_t        doppler_bin_num;
    uint16_t        range_bin_num;
    uint16_t        matrix_size;
    cint32*         data;
};

MUSICData_Impl::~MUSICData_Impl()
{
    delete [] rdc2_summaries;
    delete [] rdc2_skewers;
    delete [] covariances;
    delete [] svds;
    delete [] samples;
}


uint32_t        MUSICData_Impl::get_covariance_count() const
{
    return covariances ? myscan.scan_info.num_music_instances : 0;
}

uint32_t        MUSICData_Impl::get_svd_count() const
{
    return svds ? myscan.scan_info.num_music_instances : 0;
}

uint32_t        MUSICData_Impl::get_output_count() const
{
    return samples ? myscan.scan_info.num_music_instances : 0;
}

const cint16*   MUSICData_Impl::get_rdc2_bin(
        uint32_t  skidx,
        uint32_t& range_bin,
        uint32_t& doppler_bin,
        uint32_t& num_vrx,
        uint32_t& num_chan_iters) const
{
    if (skidx >= total_rdc2_rdbin)
    {
        last_err = MUSIC_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    const UhdpScanInformation& info = myscan.scan_info;
    range_bin = rdc2_summaries[skidx].R_bin;
    doppler_bin = rdc2_summaries[skidx].D_bin;
    num_vrx = info.total_vrx;
    num_chan_iters = info.num_channelizer_iters;

    last_err = MUSIC_NO_ERROR;
    return rdc2_skewers + skidx * num_vrx * num_chan_iters;
}


const cint64*   MUSICData_Impl::get_covariance(
        uint32_t covidx,
        uint32_t& range_bin,
        uint32_t& doppler_bin,
        uint32_t& size) const
{
    const UhdpScanInformation& info = myscan.scan_info;

    if (!covariances || covidx > info.num_music_instances)
    {
        last_err = MUSIC_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = MUSIC_NO_ERROR;
    range_bin = covariances[covidx].range_bin_num;
    doppler_bin = covariances[covidx].doppler_bin_num;
    size = covariances[covidx].matrix_size;
    return covariances[covidx].data;
}


const cint32*   MUSICData_Impl::get_svd(
        uint32_t covidx,
        uint32_t& range_bin,
        uint32_t& doppler_bin,
        uint32_t& size) const
{
    const UhdpScanInformation& info = myscan.scan_info;

    if (!svds || covidx > info.num_music_instances)
    {
        last_err = MUSIC_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = MUSIC_NO_ERROR;
    range_bin = svds[covidx].range_bin_num;
    doppler_bin = svds[covidx].doppler_bin_num;
    size = svds[covidx].matrix_size;
    return svds[covidx].data;
}


const RDC_MusicSampleData* MUSICData_Impl::get_output(uint32_t reqidx) const
{
    const UhdpScanInformation& info = myscan.scan_info;

    if (!samples || reqidx > info.num_music_instances)
    {
        last_err = MUSIC_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = MUSIC_NO_ERROR;
    return &samples[reqidx];
}


void            MUSICData_Impl::release()
{
    myscan.release_music(*this);
}


void            MUSICData_Impl::handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size)
{
    switch (message_type)
    {
    case UHDP_TYPE_RDC2:
        {
            const UhdpScanInformation& info = myscan.scan_info;
            UhdpRDC2Header *hdr = (UhdpRDC2Header*)payload;
            payload    += sizeof(UhdpRDC2Header);
            total_size -= sizeof(UhdpRDC2Header);

#if 0
            printf("per_tuple_size %d, total_vrx %d, total_tuple_in_scan %d, num_channelizer_iters %d, total_size %d\n",
                    hdr->per_tuple_size,
                    info.total_vrx,
                    hdr->total_tuple_in_scan,
                    info.num_channelizer_iters,
                    total_size
                  );
#endif

            if (hdr->per_tuple_size != sizeof(rdc2_skewers[0]) * info.total_vrx)
            {
                printf("Unexpected RDC2 tuple size\n");
                break;
            }

            if (!rdc2_skewers)  // First RDC2 packet of scan
            {
                total_rdc2_rdbin = hdr->total_tuple_in_scan;
                rdc2_skewers = new cint16[total_rdc2_rdbin * info.total_vrx * info.num_channelizer_iters];
                rdc2_summaries = new RDC_RDsummary[hdr->total_tuple_in_scan];
                cur_rdc2_skewer = 0;
                cur_rdc2_summary = 0;
            }

            // Consume all the RD summaries, which come first, before the data
            while ((cur_rdc2_summary < total_rdc2_rdbin) && (total_size >= sizeof(rdc2_summaries[0])))
            {
                memcpy(&rdc2_summaries[cur_rdc2_summary++], payload, sizeof(rdc2_summaries[0]));
                total_size -= sizeof(rdc2_summaries[0]);
                payload    += sizeof(rdc2_summaries[0]);
            }

            if (cur_rdc2_skewer < total_rdc2_rdbin * info.num_channelizer_iters)
            {
                // an even multiple of tuples
                assert((total_size % hdr->per_tuple_size) == 0);

                cint16* dest = rdc2_skewers + (cur_rdc2_skewer * info.total_vrx);
                cint16* src  = (cint16*)payload;

                while (total_size >= hdr->per_tuple_size)
                {
                    memcpy(dest, src, hdr->per_tuple_size);
                    dest += info.total_vrx;
                    src  += info.total_vrx;
                    total_size -= hdr->per_tuple_size;
                    cur_rdc2_skewer++;
                }
            }
            else
            {
                printf("Unexpected extra RDC2 data received, aborting\n");
                delete [] rdc2_skewers;
                delete [] rdc2_summaries;
                rdc2_skewers = NULL;
                rdc2_summaries = NULL;
                total_rdc2_rdbin = 0;
            }
        }
        break;

    case UHDP_TYPE_COVARIANCE:
        {
            UhdpCovarianceHeader* hdr = (UhdpCovarianceHeader*)payload;
            payload    += sizeof(UhdpCovarianceHeader);
            total_size -= sizeof(UhdpCovarianceHeader);

            if (!covariances)
            {
                covariances = new CovarianceData[myscan.scan_info.num_music_instances];
            }
            if (cur_covariance < myscan.scan_info.num_music_instances)
            {
                CovarianceData& c = covariances[cur_covariance];
                if (!c.data)
                {
                    c.matrix_size = hdr->matrix_size;
                    c.doppler_bin_num = hdr->doppler_bin_num;
                    c.range_bin_num = hdr->range_bin_num;
                    c.data = new cint64[c.matrix_size * c.matrix_size];
                }

                memcpy(c.data + hdr->sample_offset, payload, total_size);
                if (hdr->sample_offset + (total_size / sizeof(c.data[0])) == (size_t)(c.matrix_size * c.matrix_size))
                {
                    cur_covariance++;
                }
            }
            else
            {
                printf("Unexpected extra covariance data, aborting\n");
                delete [] covariances;
                covariances = NULL;
            }
        }
        break;

    case UHDP_TYPE_SVD:
        {
            UhdpSVDHeader* hdr = (UhdpSVDHeader*)payload;
            payload    += sizeof(UhdpSVDHeader);
            total_size -= sizeof(UhdpSVDHeader);

            if (!svds)
            {
                svds = new SVDData[myscan.scan_info.num_music_instances];
            }
            if (cur_svd < myscan.scan_info.num_music_instances)
            {
                SVDData& s = svds[cur_svd];
                if (!s.data)
                {
                    s.matrix_size = hdr->matrix_size;
                    s.doppler_bin_num = hdr->doppler_bin_num;
                    s.range_bin_num = hdr->range_bin_num;
                    s.data = new cint32[s.matrix_size * s.matrix_size + s.matrix_size];
                }

                memcpy(s.data + hdr->sample_offset, payload, total_size);
                if (hdr->sample_offset + (total_size / sizeof(s.data[0])) ==
                    (size_t)(s.matrix_size * s.matrix_size + s.matrix_size))
                {
                    cur_svd++;
                }
            }
            else
            {
                printf("Unexpected extra SVD data, aborting\n");
                delete [] svds;
                svds = NULL;
            }
        }
        break;

    case UHDP_TYPE_MUSIC_SAMPLES:
        {
            payload += sizeof(UhdpDataHeader);
            total_size -= sizeof(UhdpDataHeader);

            if (!samples)
            {
                samples = new RDC_MusicSampleData[myscan.scan_info.num_music_instances];
                cur_sample = 0;
            }
            uint32_t count = total_size / sizeof(RDC_MusicSampleData);
            if (cur_sample + count <= myscan.scan_info.num_music_instances)
            {
                assert(total_size == count * sizeof(RDC_MusicSampleData));
                memcpy(samples + cur_sample, payload, total_size);
                cur_sample += count;
            }
            else
            {
                printf("Unexpected MUSIC sample overflow\n");
                delete [] samples;
                samples = NULL;
            }
        }
        break;
    }
}


void            MUSICData_Impl::abort_uhdp(uint8_t message_type)
{
    switch (message_type)
    {
    case UHDP_TYPE_RDC2:
        delete [] rdc2_skewers;
        delete [] rdc2_summaries;
        rdc2_skewers = NULL;
        rdc2_summaries = NULL;
        total_rdc2_rdbin = 0;
        break;

    case UHDP_TYPE_COVARIANCE:
        delete [] covariances;
        covariances = NULL;
        break;

    case UHDP_TYPE_SVD:
        delete [] svds;
        svds = NULL;
        break;

    case UHDP_TYPE_MUSIC_SAMPLES:
        delete [] samples;
        samples = NULL;
        break;
    }
}


void            MUSICData_Impl::finish_uhdp(uint8_t message_type)
{
    switch (message_type)
    {
    case UHDP_TYPE_RDC2:
        if (cur_rdc2_summary != total_rdc2_rdbin ||
            cur_rdc2_skewer != total_rdc2_rdbin * myscan.scan_info.num_channelizer_iters)
        {
            delete [] rdc2_skewers;
            delete [] rdc2_summaries;
            rdc2_skewers = NULL;
            rdc2_summaries = NULL;
            total_rdc2_rdbin = 0;
        }
        break;

    case UHDP_TYPE_COVARIANCE:
        if (cur_covariance != myscan.scan_info.num_music_instances)
        {
            delete [] covariances;
            covariances = NULL;
        }
        break;

    case UHDP_TYPE_SVD:
        if (cur_svd != myscan.scan_info.num_music_instances)
        {
            delete [] svds;
            svds = NULL;
        }
        break;

    case UHDP_TYPE_MUSIC_SAMPLES:
        if (cur_sample != myscan.scan_info.num_music_instances)
        {
            delete [] samples;
            samples = NULL;
        }
        break;
    }
}

struct  RDC_MusicSampleData_058
{
    FLOAT    range;
    FLOAT    doppler;
    FLOAT    azimuth_min;
    FLOAT    azimuth_max;
    FLOAT    elevation_min;
    FLOAT    elevation_max;
    uint16_t num_azimuth_steps;
    uint16_t num_elevation_steps;
    uint32_t inv_magnitude[64];
};

// return true if _any_ music data is loaded. if false is returned this instance
// will be released
bool            MUSICData_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    const UhdpScanInformation& info = myscan.scan_info;
    bool keeper = false;

    sprintf(fname, "scan_%06d_sprdc2chu.bin", info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        size_t per_rdbin = info.total_vrx * info.num_channelizer_iters * sizeof(rdc2_skewers[0]);
        total_rdc2_rdbin = len / per_rdbin;
        if (total_rdc2_rdbin * per_rdbin == len)
        {
            rdc2_skewers = new cint16[total_rdc2_rdbin * info.total_vrx * info.num_channelizer_iters];
            s.read_scan_data_type(rdc2_skewers, sizeof(cint16), total_rdc2_rdbin * info.total_vrx * info.num_channelizer_iters);
            s.end_read_scan_data_type();

            sprintf(fname, "scan_%06d_spsumchu.bin", info.scan_sequence_number);
            len = s.begin_read_scan_data_type(fname);
            if (len == sizeof(RDC_RDsummary) * total_rdc2_rdbin)
            {
                rdc2_summaries = new RDC_RDsummary[total_rdc2_rdbin];
                if (s.read_scan_data_type(rdc2_summaries, sizeof(RDC_RDsummary), total_rdc2_rdbin))
                {
                    keeper = true;
                }
            }
            else
            {
                total_rdc2_rdbin = 0;
                delete [] rdc2_skewers;
                delete [] rdc2_summaries;
                rdc2_skewers = NULL;
                rdc2_summaries = NULL;
            }
        }
    }
    s.end_read_scan_data_type();

    sprintf(fname, "scan_%06d_covmx.bin", info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        bool ok = true;
        covariances = new CovarianceData[info.num_music_instances];
        for (uint16_t i = 0; i < info.num_music_instances; i++)
        {
            CovarianceData& c = covariances[i];
            ok &= s.read_scan_data_type(&c.range_bin_num, sizeof(c.range_bin_num), 1);
            ok &= s.read_scan_data_type(&c.doppler_bin_num, sizeof(c.doppler_bin_num), 1);
            ok &= s.read_scan_data_type(&c.matrix_size, sizeof(c.matrix_size), 1);
            uint16_t ni;
            ok &= s.read_scan_data_type(&ni, sizeof(ni), 1);
            ok &= ni == i;
            if (ok)
            {
                c.data = new cint64[c.matrix_size * c.matrix_size];
                ok &= s.read_scan_data_type(c.data, sizeof(c.data[0]), c.matrix_size * c.matrix_size);
            }
            if (!ok)
            {
                break;
            }
        }
        if (ok)
        {
            keeper = true;
        }
        else
        {
            delete [] covariances;
            covariances = NULL;
        }
    }
    s.end_read_scan_data_type();

    sprintf(fname, "scan_%06d_usmxup.bin", info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        bool ok = true;
        svds = new SVDData[info.num_music_instances];
        for (uint16_t i = 0; i < info.num_music_instances; i++)
        {
            SVDData& svd = svds[i];
            ok &= s.read_scan_data_type(&svd.range_bin_num, sizeof(svd.range_bin_num), 1);
            ok &= s.read_scan_data_type(&svd.doppler_bin_num, sizeof(svd.doppler_bin_num), 1);
            ok &= s.read_scan_data_type(&svd.matrix_size, sizeof(svd.matrix_size), 1);
            uint16_t ni;
            ok &= s.read_scan_data_type(&ni, sizeof(ni), 1);
            ok &= ni == i;
            if (ok)
            {
                uint32_t count = svd.matrix_size * svd.matrix_size + svd.matrix_size;
                svd.data = new cint32[count];
                ok &= s.read_scan_data_type(svd.data, sizeof(svd.data[0]), count);
            }
            if (!ok)
            {
                break;
            }
        }
        if (ok)
        {
            keeper = true;
        }
        else
        {
            delete [] svds;
            svds = NULL;
        }
    }
    s.end_read_scan_data_type();

    if (info.num_music_instances)
    {
        sprintf(fname, "scan_%06d_music.bin", info.scan_sequence_number);
        len = s.begin_read_scan_data_type(fname);
        if (len == sizeof(samples[0]) * info.num_music_instances)
        {
            samples = new RDC_MusicSampleData[info.num_music_instances];
            if (s.read_scan_data_type(samples, sizeof(RDC_MusicSampleData), info.num_music_instances))
            {
                keeper = true;
            }
            else
            {
                delete [] samples;
                samples = NULL;
            }
        }
        else if (len == sizeof(RDC_MusicSampleData_058) * info.num_music_instances)
        {
            samples = new RDC_MusicSampleData[info.num_music_instances];
            keeper = true;

            for (uint32_t i = 0; i < info.num_music_instances; i++)
            {
                RDC_MusicSampleData_058 old;

                if (s.read_scan_data_type(&old, sizeof(old), 1))
                {
                    memset(&samples[i], 0, sizeof(samples[i]));
                    samples[i].range = old.range;
                    samples[i].doppler = old.doppler;
                    samples[i].azimuth_min = old.azimuth_min;
                    samples[i].azimuth_max = old.azimuth_max;
                    samples[i].elevation_min = old.elevation_min;
                    samples[i].elevation_max = old.elevation_max;
                    samples[i].num_azimuth_steps = old.num_azimuth_steps;
                    samples[i].num_elevation_steps = old.num_elevation_steps;
                    memcpy(&samples[i].inv_magnitude[0], &old.inv_magnitude[0],
                           uh_uintmin(sizeof(old.inv_magnitude), sizeof(samples[i].inv_magnitude)));
                }
                else
                {
                    keeper = false;
                }
            }

            if (!keeper)
            {
                delete [] samples;
                samples = NULL;
            }
        }
        s.end_read_scan_data_type();
    }
    return keeper;
}


bool            MUSICData_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];
    const UhdpScanInformation& info = myscan.scan_info;

    bool ok = true;

    if (rdc2_skewers && rdc2_summaries && total_rdc2_rdbin && ok)
    {
        sprintf(fname, "scan_%06d_sprdc2chu.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(rdc2_skewers, sizeof(rdc2_skewers[0]), info.total_vrx * info.num_channelizer_iters * total_rdc2_rdbin);
            s.end_write_scan_data_type(!ok);
        }

        sprintf(fname, "scan_%06d_spsumchu.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(rdc2_summaries, sizeof(rdc2_summaries[0]), total_rdc2_rdbin);
            s.end_write_scan_data_type(!ok);
        }
    }

    if (covariances && ok)
    {
        sprintf(fname, "scan_%06d_covmx.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            for (uint16_t i = 0; i < info.num_music_instances; i++)
            {
                const CovarianceData& c = covariances[i];
                ok &= s.write_scan_data_type(&c.range_bin_num, sizeof(c.range_bin_num), 1);
                ok &= s.write_scan_data_type(&c.doppler_bin_num, sizeof(c.doppler_bin_num), 1);
                ok &= s.write_scan_data_type(&c.matrix_size, sizeof(c.matrix_size), 1);
                ok &= s.write_scan_data_type(&i, sizeof(i), 1);
                ok &= s.write_scan_data_type(c.data, sizeof(c.data[0]), c.matrix_size * c.matrix_size);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    if (svds && ok)
    {
        sprintf(fname, "scan_%06d_usmxup.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            for (uint16_t i = 0; i < info.num_music_instances; i++)
            {
                const SVDData& svd = svds[i];
                ok &= s.write_scan_data_type(&svd.range_bin_num, sizeof(svd.range_bin_num), 1);
                ok &= s.write_scan_data_type(&svd.doppler_bin_num, sizeof(svd.doppler_bin_num), 1);
                ok &= s.write_scan_data_type(&svd.matrix_size, sizeof(svd.matrix_size), 1);
                ok &= s.write_scan_data_type(&i, sizeof(i), 1);
                ok &= s.write_scan_data_type(svd.data, sizeof(svd.data[0]), svd.matrix_size * svd.matrix_size + svd.matrix_size);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    if (samples && ok)
    {
        sprintf(fname, "scan_%06d_music.bin", info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(samples, sizeof(RDC_MusicSampleData), info.num_music_instances);
            s.end_write_scan_data_type(!ok);
        }
    }

    return ok;
}
