#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/src/staticslice_impl.h"
#include "modules/drivers/radar/quill_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/quill_radar/driver/include/serializer.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include <assert.h>


StaticSlice_Impl::~StaticSlice_Impl()
{
    delete [] ss;
}


uint32_t             StaticSlice_Impl::get_num_angle_bins() const
{
    return myscan.scan_info.SS_size_A;
}


uint32_t             StaticSlice_Impl::get_num_slices() const
{
    return num_captured_slices;
}


bool                 StaticSlice_Impl::complex_data_available() const
{
    return !!(myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_SS);
}


float                StaticSlice_Impl::get_slice_velocity(uint32_t slice) const
{
    if (slice >= num_captured_slices)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return 0.0f;
    }

    last_err = SS_NO_ERROR;
    int32_t rel_bin = (int32_t)slice - ((num_captured_slices - 1) / 2);
    return myscan.scan_info.doppler_bin_width * rel_bin;
}


const uint16_t*      StaticSlice_Impl::get_skewer(uint32_t range_bin, uint32_t slice) const
{
    const UhdpScanInformation& info = myscan.scan_info;

    if (slice >= num_captured_slices || range_bin >= info.SS_size_R)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = SS_NO_ERROR;
    if (info.complex_rdc3 & RDC3_COMPLEX_SS)
    {
        static uint16_t temp_skewer[MAX_MAX_ROUGH_ANGLES];
        cint16* cplx_samples = (cint16*)ss +
                               range_bin * num_captured_slices * info.SS_size_A +
                               slice * info.SS_size_A;
        for (uint32_t i = 0; i < info.SS_size_A; i++)
        {
            temp_skewer[i] = cplx_samples[i].abs();
        }
        return &temp_skewer[0];
    }
    else
    {
        return ss + range_bin * num_captured_slices * info.SS_size_A +
                    slice * info.SS_size_A;
    }
}


const cint16*      StaticSlice_Impl::get_skewer_complex(uint32_t range_bin, uint32_t slice) const
{
    const UhdpScanInformation& info = myscan.scan_info;

    if (slice >= num_captured_slices || range_bin >= info.SS_size_R)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return NULL;
    }

    last_err = SS_NO_ERROR;
    if (info.complex_rdc3 & RDC3_COMPLEX_SS)
    {
        return (cint16*)ss +
               range_bin * num_captured_slices * info.SS_size_A +
               slice * info.SS_size_A;
    }
    else
    {
        uint16_t* mag = ss +
                        range_bin * num_captured_slices * info.SS_size_A +
                        slice * info.SS_size_A;
        static cint16 temp_skewer[MAX_MAX_ROUGH_ANGLES];
        for (uint32_t i = 0; i < info.SS_size_A; i++)
        {
            temp_skewer[i] = cint16(mag[i], 0);
        }
        return &temp_skewer[0];
    }
}


int16_t              StaticSlice_Impl::get_exponent_at_range(uint32_t rbin) const
{
    if (rbin >= myscan.scan_info.SS_size_R)
    {
        last_err = SS_INDEX_OUT_OF_RANGE;
        return 0;
    }

    return myscan.range_bins[rbin].exponent;
}


void                 StaticSlice_Impl::allocate()
{
    const UhdpScanInformation& info = myscan.scan_info;

    if (myscan.uhdp_version >= 32)
    {
        num_captured_slices = info.ss_doppler_0_only ? 1 : info.SS_size_D;
    }
    else
    {
        num_captured_slices = info.SS_size_D;
    }
    uint32_t multiplier = (info.complex_rdc3 & RDC3_COMPLEX_SS) ? 2 : 1;
    ss = new uint16_t[info.SS_size_R * info.SS_size_A * num_captured_slices *
                      multiplier];
}


void                 StaticSlice_Impl::handle_uhdp(const UhdpStaticSliceHeader* msg, uint32_t total_size, uint8_t uhdp_ver)
{
    if (aborted)
    {
        return;
    }
    else if (total_size < sizeof(UhdpStaticSliceHeader))
    {
        fprintf(stderr, "Static Slice message is too small: %d bytes\n", total_size);
        aborted = true;
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
            if (myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_SS)
            {
                ; // ok
            }
            else
            {
                printf("Unexpected static slice skewer size for complex\n");
                aborted = true;
                return;
            }
        }
        else if (msg->per_tuple_size == 2 * info.SS_size_A)
        {
            if (myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_SS)
            {
                printf("Unexpected static slice skewer size for magnitude\n");
                aborted = true;
                return;
            }
            else
            {
                ; // ok
            }
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

        if (++cur_doppler_bin == num_captured_slices && uhdp_ver < 31)
        {
            const RDC_SSRsummary& rsum = *reinterpret_cast<const RDC_SSRsummary*>(payload);
            if (rsum.R_bin != cur_range_bin)
            {
                printf("Static slice rbin summary is invalid\n");
                aborted = true;
                return;
            }

            myscan.range_bins[cur_range_bin].exponent = rsum.exponent;
            cur_range_bin++;
            cur_doppler_bin = 0;

            payload    += sizeof(RDC_SSRsummary);
            total_size -= sizeof(RDC_SSRsummary);
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
                       num_captured_slices *
                       myscan.scan_info.SS_size_A;
    uint32_t multiplier = (myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_SS) ? 2 : 1;
    if (bytes_written != ss_size * sizeof(ss[0]) * multiplier)
    {
        printf("Static slice is incomplete, discarding. %zu != %zu\n",
                bytes_written, ss_size * sizeof(ss[0]) * multiplier);
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
        cint16* ssc = (cint16*)ss;
        uint32_t samples = myscan.scan_info.SS_size_A * num_captured_slices;
        for (uint32_t r = 0; r < myscan.scan_info.SS_size_R; r++)
        {
            if (myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_SS)
            {
                ok &= s.write_scan_data_type(ssc + r * samples, sizeof(ssc[0]), samples);
            }
            else
            {
                ok &= s.write_scan_data_type(ss + r * samples, sizeof(ss[0]), samples);
            }
            if (myscan.uhdp_version < 31)
            {
                trailer t;
                t.exponent = myscan.range_bins[r].exponent;
                t.R_bin = r;
                t.reserved1 = 0;
                t.reserved2 = 0;
                ok &= s.write_scan_data_type(&t, sizeof(t), 1);
            }
        }
        s.end_write_scan_data_type(!ok);
    }

    return ok;
}


bool                 StaticSlice_Impl::deserialize(ScanSerializer& s)
{
    const UhdpScanInformation& info = myscan.scan_info;
    if (myscan.uhdp_version >= 32)
    {
        // info.ss_doppler_0_only is only valid in UhDP version 32 and above
        num_captured_slices = info.ss_doppler_0_only ? 1 : info.SS_size_D;
    }
    else
    {
        // previously, the radar set info.SS_size_D to the number of slices that
        // were captured
        num_captured_slices = info.SS_size_D;
    }

    char fname[128];
    sprintf(fname, "scan_%06d_stslice.bin", myscan.scan_info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        bool ok = true;
        cint16* ssc = (cint16*)ss;
        uint32_t samples = myscan.scan_info.SS_size_A * num_captured_slices;
        for (uint32_t r = 0; r < myscan.scan_info.SS_size_R; r++)
        {
            if (myscan.scan_info.complex_rdc3 & RDC3_COMPLEX_SS)
            {
                ok &= s.read_scan_data_type(ssc + r * samples, sizeof(ssc[0]), samples);
            }
            else
            {
                ok &= s.read_scan_data_type(ss + r * samples, sizeof(ss[0]), samples);
            }

            if (myscan.uhdp_version < 31)
            {
                trailer t;
                ok &= s.read_scan_data_type(&t, sizeof(t), 1);
                myscan.range_bins[r].exponent = t.exponent;
                ok &= (r == t.R_bin);
            }
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
