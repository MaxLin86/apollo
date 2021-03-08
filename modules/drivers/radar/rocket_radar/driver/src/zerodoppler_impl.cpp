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
        bool ok = s.read_scan_data_type(plane, sizeof(cint64), num_zd_range_bins * myscan.scan_info.total_vrx);
        s.end_read_scan_data_type();
        return ok;
    }

    return false;
}
