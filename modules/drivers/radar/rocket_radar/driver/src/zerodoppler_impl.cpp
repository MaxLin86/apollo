// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/zerodoppler_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"

void    ZeroDoppler_Impl::handle_uhdp(uint32_t* payload, uint32_t total_size)
{
    if (aborted || total_size < sizeof(uint32_t))
    {
        aborted = true;
        return;
    }

    if (!plane)
    {
        allocate();
    }

    uint32_t byte_offset = *payload;
    total_size -= sizeof(uint32_t);

    if (byte_offset != cur_byte_offset)
    {
        aborted = true;
        return;
    }

    if (cur_byte_offset + total_size > sizeof(cint64) * myscan.scan_info.total_vrx * num_zd_range_bins)
    {
        aborted = true;
        return;
    }

    cint64* raw_samples = (cint64*)(payload + 1);
    memcpy((char*)plane + cur_byte_offset, raw_samples, total_size);

    cur_byte_offset += total_size;
}


void     ZeroDoppler_Impl::allocate()
{
    num_zd_range_bins = uh_uintmin(myscan.scan_info.rdc2_zd_rb_halfwidth * 2 + 1, myscan.scan_info.num_range_bins);
    plane = new cint64[num_zd_range_bins * myscan.scan_info.total_vrx];
}


uint32_t ZeroDoppler_Impl::get_range_dimension() const
{
    return num_zd_range_bins;
}


uint32_t ZeroDoppler_Impl::get_vrx_dimension() const
{
    return myscan.scan_info.total_vrx;
}


const cint64* ZeroDoppler_Impl::get_range_bin(uint32_t r) const
{
    if (r < num_zd_range_bins)
    {
        return plane + r * myscan.scan_info.total_vrx;
    }
    else
    {
        return NULL;
    }
}

void     ZeroDoppler_Impl::setup()
{
    // called when the xmit should be complete
    const uint32_t num_vrx = myscan.scan_info.total_vrx;
    if (cur_byte_offset != sizeof(cint64) * num_vrx * num_zd_range_bins)
    {
        aborted = true;
    }
    else if (myscan.vrx_mapping)
    {
        // remap from Rx Major to +Y order
        cint64* swizzle = new cint64[num_vrx];

        for (uint32_t rb = 0; rb < num_zd_range_bins; rb++)
        {
            cint64* dest = plane + rb * num_vrx;
            memcpy(swizzle, dest, sizeof(cint64) * num_vrx);

            for (uint32_t i = 0; i < num_vrx; i++)
                dest[myscan.vrx_mapping[i]] = swizzle[i];
        }

        delete [] swizzle;
    }
    else
    {
        // no vrx mapping, no RDC2 ZD
        aborted = true;
    }

    if (aborted)
    {
        release();
    }
}

void     ZeroDoppler_Impl::release()
{
    myscan.release_zero_doppler(*this);
}

bool     ZeroDoppler_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];
    sprintf(fname, "scan_%06d_rdc2zdop.bin", myscan.scan_info.scan_sequence_number);
    bool ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= s.write_scan_data_type(plane, sizeof(cint64), num_zd_range_bins * myscan.scan_info.total_vrx);
        s.end_write_scan_data_type(!ok);
    }

    return ok;
}

bool     ZeroDoppler_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];
    sprintf(fname, "scan_%06d_rdc2zdop.bin", myscan.scan_info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        allocate();
        bool ok = s.read_scan_data_type(plane, sizeof(cint64), num_zd_range_bins * myscan.scan_info.total_vrx);
        s.end_read_scan_data_type();
        return ok;
    }

    return false;
}
