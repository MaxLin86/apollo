#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/staticslice_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/serializer.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include <assert.h>


StaticSlice_Impl::~StaticSlice_Impl()
{
    delete [] ss;
    delete [] rexp;
}


uint32_t             StaticSlice_Impl::get_num_angle_bins() const
{
    return myscan.scan_info.SS_size_A;
}


uint32_t             StaticSlice_Impl::get_num_slices() const
{
    return myscan.scan_info.SS_size_D;
}


float                StaticSlice_Impl::get_slice_velocity(uint32_t slice) const
{
    if (slice >= myscan.scan_info.SS_size_D)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return 0.0f;
    }

    last_err = SS_NO_ERROR;
    int32_t rel_bin = (int32_t)slice - ((myscan.scan_info.SS_size_D - 1) / 2);
    return myscan.scan_info.doppler_bin_width * rel_bin;
}


const uint16_t*      StaticSlice_Impl::get_skewer(uint32_t range_bin, uint32_t slice) const
{
    if (slice >= myscan.scan_info.SS_size_D || range_bin >= myscan.scan_info.SS_size_R)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = SS_NO_ERROR;
    return ss + range_bin * myscan.scan_info.SS_size_D * myscan.scan_info.SS_size_A +
                                                 slice * myscan.scan_info.SS_size_A;
}


int16_t              StaticSlice_Impl::get_exponent_at_range(uint32_t rbin) const
{
    if (rbin >= myscan.scan_info.SS_size_R)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return 0;
    }

    return rexp[rbin];
}


void                 StaticSlice_Impl::allocate()
{
    ss = new uint16_t[myscan.scan_info.SS_size_R *
                      myscan.scan_info.SS_size_D *
                      myscan.scan_info.SS_size_A];

    rexp = new int16_t[myscan.scan_info.SS_size_R];
}


void                 StaticSlice_Impl::handle_uhdp(const UhdpStaticSliceHeader* msg, uint32_t total_size)
{
    if (total_size < sizeof(UhdpStaticSliceHeader) + sizeof(RHAL_SSRsummary))
    {
        fprintf(stderr, "Static Slice message is too small: %d bytes\n", total_size);
        aborted = true;
        return;
    }
    if (aborted)
    {
        return;
    }

    const char* payload = (const char*)(msg + 1);
    const UhdpScanInformation& info = myscan.scan_info;
    total_size -= sizeof(UhdpStaticSliceHeader);

    if (!skewer_size)
    {
        write_ptr = (char*)ss;

        skewer_size = msg->per_tuple_size;
        if (msg->per_tuple_size == 4 * info.SS_size_A)
        {
            printf("Complex static slice not supported\n");
            aborted = true;
            return;
        }
        else if (msg->per_tuple_size == 2 * info.SS_size_A)
        {
            // ok
        }
        else
        {
            printf("Unsupported static slice tuple size\n");
            aborted = true;
            return;
        }

        cur_range_bin   = 0;
        cur_doppler_bin = 0;
    }

    while (total_size >= msg->per_tuple_size)
    {
        memcpy(write_ptr, payload, skewer_size);
        write_ptr  += skewer_size;
        payload    += skewer_size;
        total_size -= skewer_size;

        if (++cur_doppler_bin == info.SS_size_D)
        {
            const RHAL_SSRsummary& rsum = *reinterpret_cast<const RHAL_SSRsummary*>(payload);
            if (rsum.R_bin != cur_range_bin)
            {
                printf("Static slice rbin summary is invalid\n");
                aborted = true;
                return;
            }

            rexp[cur_range_bin] = rsum.exponent;
            cur_range_bin++;
            cur_doppler_bin = 0;

            payload    += sizeof(RHAL_SSRsummary);
            total_size -= sizeof(RHAL_SSRsummary);
        }
    }

    if (total_size)
    {
        printf("Static Slice buffer mismatch (remain: %d)\n", total_size);
        aborted = true;
    }
}


void                 StaticSlice_Impl::setup()
{
    if (aborted)
    {
        release();
        return;
    }

    size_t bytes_written = write_ptr - (char*)ss;
    uint32_t ss_size = myscan.scan_info.SS_size_R *
                       myscan.scan_info.SS_size_D *
                       myscan.scan_info.SS_size_A;
    if (bytes_written / sizeof(ss[0]) != ss_size)
    {
        printf("Static slice is incomplete, discarding\n");
        release();
        return;
    }

    if (!myscan.range_bins || !myscan.angle_bins || !myscan.zero_d_bins)
    {
        printf("Static slice is missing range, angle or doppler data, discarding\n");
        release();
        return;
    }
}

struct trailer
{
   int8_t   exponent;
   uint8_t  reserved1;
   uint16_t R_bin;
   uint32_t reserved2;
};

bool                 StaticSlice_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];
    sprintf(fname, "scan_%06d_stslice.bin", myscan.scan_info.scan_sequence_number);
    bool ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        uint32_t samples = myscan.scan_info.SS_size_A * myscan.scan_info.SS_size_D;
        for (uint32_t r = 0; r < myscan.scan_info.SS_size_R; r++)
        {
            trailer t;
            t.exponent = rexp[r];
            t.reserved1 = 0;
            t.R_bin = r;
            t.reserved2 = 0;

            ok &= s.write_scan_data_type(ss + r * samples, sizeof(ss[0]), samples);
            ok &= s.write_scan_data_type(&t, sizeof(t), 1);
        }
        s.end_write_scan_data_type(!ok);
    }

    return ok;
}


bool                 StaticSlice_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    sprintf(fname, "scan_%06d_stslice.bin", myscan.scan_info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        bool ok = true;
        uint32_t samples = myscan.scan_info.SS_size_A * myscan.scan_info.SS_size_D;
        for (uint32_t r = 0; r < myscan.scan_info.SS_size_R; r++)
        {
            trailer t;

            ok &= s.read_scan_data_type(ss + r * samples, sizeof(ss[0]), samples);
            ok &= s.read_scan_data_type(&t, sizeof(t), 1);

            rexp[r] = t.exponent;
            ok &= (r == t.R_bin);
        }

        s.end_read_scan_data_type();
        return ok;
    }
    else
    {
        return false;
    }
}


void                 StaticSlice_Impl::release()
{
    myscan.release_static_slice(*this);
}
