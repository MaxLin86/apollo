#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/radardatacube1_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include <assert.h>

struct ReplayHeader
{
    uint32_t magic;
    uint32_t rdc1_exp_size;
    uint32_t rdc1_info_size;
    uint32_t rdc1_size;

    size_t total_size() const
    {
        return sizeof(*this) + rdc1_exp_size + rdc1_info_size + rdc1_size;
    }

    static const uint32_t MAGIC = 0x357a9a04UL;
};

enum { RDC1_EXP_SIZE_PER_RANGEBIN = 16 }; // rsp-common.h

uint32_t             RadarDataCube1_Impl::get_range_dimension() const
{
    return myscan.scan_info.num_range_bins;
}


uint32_t             RadarDataCube1_Impl::get_vrx_dimension() const
{
    return myscan.scan_info.total_vrx;
}


uint32_t             RadarDataCube1_Impl::get_pulses_dimension() const
{
    return myscan.scan_info.num_pulses;
}


const cint16*        RadarDataCube1_Impl::get_samples() const
{
    return rdc1;
}


const char*          RadarDataCube1_Impl::get_exponent_data() const
{
    return rdc1exp;
}


const char*          RadarDataCube1_Impl::get_rdc1_info() const
{
    if (rdc1_playback_data)
    {
        const ReplayHeader* header = (const ReplayHeader*)rdc1_playback_data;
        return rdc1_playback_data + sizeof(*header) + header->rdc1_exp_size;
    }
    else
    {
        return NULL;
    }
}


size_t               RadarDataCube1_Impl::get_rdc1_size_bytes() const
{
    if (rdc1_playback_data)
    {
        const ReplayHeader* header = (const ReplayHeader*)rdc1_playback_data;
        return header->rdc1_size;
    }
    else
    {
        return sizeof(rdc1[0]) * myscan.scan_info.num_range_bins * myscan.scan_info.num_pulses * myscan.scan_info.total_vrx;
    }
}


size_t               RadarDataCube1_Impl::get_rdc1_exp_size_bytes() const
{
    if (rdc1_playback_data)
    {
        const ReplayHeader* header = (const ReplayHeader*)rdc1_playback_data;
        return header->rdc1_exp_size;
    }
    else
    {
        return myscan.scan_info.num_range_bins * RDC1_EXP_SIZE_PER_RANGEBIN;
    }
}


size_t               RadarDataCube1_Impl::get_rdc1_info_size_bytes() const
{
    if (rdc1_playback_data)
    {
        const ReplayHeader* header = (const ReplayHeader*)rdc1_playback_data;
        return header->rdc1_info_size;
    }
    else
    {
        return 0;
    }
}


void                 RadarDataCube1_Impl::allocate()
{
    rdc1 = new cint16[myscan.scan_info.num_range_bins *
                      myscan.scan_info.num_pulses *
                      myscan.scan_info.total_vrx];

    memset(rdc1, 0, sizeof(rdc1[0]) * myscan.scan_info.num_range_bins * myscan.scan_info.num_pulses * myscan.scan_info.total_vrx);

    rdc1exp = new char[myscan.scan_info.num_range_bins * RDC1_EXP_SIZE_PER_RANGEBIN];
}


void                 RadarDataCube1_Impl::handle_uhdp(const char* payload, uint32_t total_size)
{
    UhdpRDC1Header* hdr = (UhdpRDC1Header*)payload;
    const char* exp_hdr = payload + sizeof(UhdpRDC1Header);
    uint32_t exp_size = total_size - sizeof(UhdpRDC1Header);

    if (!rdc1)
    {
        allocate();
    }

    if (hdr->cur_pulse == 0xFFFF)
    {
        uint32_t numrb = exp_size / RDC1_EXP_SIZE_PER_RANGEBIN;
        if (numrb * RDC1_EXP_SIZE_PER_RANGEBIN == exp_size &&
            hdr->cur_rb + numrb <= myscan.scan_info.num_range_bins)
        {
            memcpy(rdc1exp + RDC1_EXP_SIZE_PER_RANGEBIN * hdr->cur_rb, exp_hdr, exp_size);
        }
        else
        {
            printf("Unexpected RDC1 exponent packet size\n");
            aborted = true;
        }
    }
    else
    {
        cint16* dest = rdc1 + (hdr->cur_pulse * myscan.scan_info.num_range_bins * myscan.scan_info.total_vrx)
                            + (hdr->cur_rb * myscan.scan_info.total_vrx);
        uint32_t skewer_sizebytes = sizeof(rdc1[0]) * myscan.scan_info.total_vrx;

        while (exp_size >= skewer_sizebytes)
        {
            cint16* src = (cint16*)exp_hdr;

            if (myscan.vrx_mapping)
            {
                // While storing skewer, remap from Rx Major to +Y order
                for (uint32_t i = 0; i < myscan.scan_info.total_vrx; i++)
                    dest[myscan.vrx_mapping[i]] = src[i];
            }
            else
            {
                memcpy(dest, src, sizeof(rdc1[0]) * myscan.scan_info.total_vrx);
            }
            dest       += myscan.scan_info.total_vrx;
            exp_hdr    += skewer_sizebytes;
            exp_size   -= skewer_sizebytes;
        }

        if (exp_size)
        {
            printf("Unexpected RDC1 packet size\n");
            aborted = true;
        }
    }
    return;
}


void                 RadarDataCube1_Impl::handle_replay_uhdp(const char* payload, uint32_t total_size)
{
    payload    += sizeof(UhdpDataHeader);
    total_size -= sizeof(UhdpDataHeader);
    if (!rdc1_playback_data)
    {
        if (total_size > sizeof(uint32_t) * 4)
        {
            const ReplayHeader* header = (const ReplayHeader*)payload;
            if (header->magic == ReplayHeader::MAGIC)
            {
                rdc1_playback_total_bytes = header->total_size();
                rdc1_playback_data = new char[rdc1_playback_total_bytes];
            }
            else
            {
                aborted = true;
            }
        }
    }
    if (rdc1_playback_data)
    {
        if (rdc1_playback_received + total_size <= rdc1_playback_total_bytes)
        {
            memcpy(rdc1_playback_data + rdc1_playback_received, payload, total_size);
            rdc1_playback_received += total_size;
        }
    }
}


void                 RadarDataCube1_Impl::setup()
{
    if (aborted)
    {
        release();
        return;
    }

    if (rdc1_playback_data)
    {
        if (rdc1_playback_received != rdc1_playback_total_bytes)
        {
            delete [] rdc1_playback_data;
            rdc1_playback_data = NULL;
            rdc1_playback_received = 0U;
            rdc1_playback_total_bytes = 0U;
        }
        else
        {
            delete [] rdc1;
            delete [] rdc1exp;

            const ReplayHeader* header = (const ReplayHeader*)rdc1_playback_data;
            rdc1exp = rdc1_playback_data + sizeof(*header);
            rdc1 = (cint16*)(rdc1_playback_data + sizeof(*header) + header->rdc1_info_size + header->rdc1_exp_size);
        }
    }
    else if (!rdc1)
    {
        release();
        return;
    }

    const uint16_t* exp16 = reinterpret_cast<const uint16_t*>(rdc1exp);

    // Parse raw range exponent data
    range_exponents = new RangeExponentData[myscan.scan_info.num_range_bins];

    // Adapted from RHAL_ScanInstanceImpl::get_rdc2_zero_doppler()
    uint8_t bit_offsets[9] = { 4, 17, 30, 43, 56, 5, 18, 31, 44 };
    for (uint32_t r = 0; r < myscan.scan_info.num_range_bins; r++)
    {
        const uint64_t *exp_entry = (const uint64_t*)(&exp16[r * (128 / 16)]); // Get exponent table entry for the given range bin

        // Extract all elements from this entry
        // Base Exp (4-bits)|+1 (13-bits)|+2 (13-bits)|+3 (13-bits)|+4 (13-bits)|+5 (13-bits)|+6 (13-bits)|+7 (13-bits)|+8 (13-bits)| Growth |
        //        3:0           16:4         29:17         42:30       55:43        68:56        81:69        94:82         107:95   111:108
        range_exponents[r].base_exp = (uint8_t)(exp_entry[0] & 0x000FU);
        for (INT n = 0; n < 4; n++)
        {
            range_exponents[r].incr_pulse[n] = (uint16_t)((exp_entry[0] >> bit_offsets[n]) & 0x1FFFU);
        }

        // 4th position straddles bith 64-bit parts. 8 bits from [0] and 5 bits from [1]
        range_exponents[r].incr_pulse[4] = ((exp_entry[0] >> bit_offsets[4]) & 0xFFU) | ((exp_entry[1] & 0x1FU) << 8);
        for (INT n = 5; n < 8; n++)
        {
            range_exponents[r].incr_pulse[n] = (uint16_t)((exp_entry[1] >> bit_offsets[n]) & 0x1FFFU);
        }

        range_exponents[r].max_growth = (exp_entry[1] >> bit_offsets[8]) & 0xFU;
    }
}


uint32_t             RadarDataCube1_Impl::get_exponent_at_pulse_range(uint32_t pulse, uint32_t range_bin) const
{
    if (pulse < myscan.scan_info.num_pulses && range_bin < myscan.scan_info.num_range_bins)
    {
        RangeExponentData& red = range_exponents[range_bin];
        uint32_t exp = red.base_exp;

        for (uint16_t x = 0; x < red.max_growth; x++)
        {
            if (red.incr_pulse[x] <= pulse)
            {
                exp++;
            }
            else
            {
                break;
            }
        }

        return exp;
    }
    else
    {
        return 0;
    }
}


bool                 RadarDataCube1_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];

    if (rdc1_playback_data)
    {
        sprintf(fname, "scan_%06d_rdc1_bundle.bin", myscan.scan_info.scan_sequence_number);
        bool ok = s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(rdc1_playback_data, rdc1_playback_total_bytes, 1);
            s.end_write_scan_data_type(!ok);
        }
        return ok;
    }

    sprintf(fname, "scan_%06d_rdc1.bin", myscan.scan_info.scan_sequence_number);
    bool ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= s.write_scan_data_type(rdc1, sizeof(rdc1[0]), myscan.scan_info.num_range_bins
                * myscan.scan_info.num_pulses
                * myscan.scan_info.total_vrx);
        s.end_write_scan_data_type(!ok);
    }
    sprintf(fname, "scan_%06d_rdc1exp.bin", myscan.scan_info.scan_sequence_number);
    ok &= s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= s.write_scan_data_type(rdc1exp, RDC1_EXP_SIZE_PER_RANGEBIN, myscan.scan_info.num_range_bins);
        s.end_write_scan_data_type(!ok);
    }
    return ok;
}


bool                 RadarDataCube1_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];

    sprintf(fname, "scan_%06d_rdc1_bundle.bin", myscan.scan_info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        rdc1_playback_total_bytes = len;
        rdc1_playback_received = len;
        rdc1_playback_data = new char[len];
        s.read_scan_data_type(rdc1_playback_data, 1, len);
        s.end_read_scan_data_type();
        aborted = false;
        setup();
        return true;
    }
    else
    {
        s.end_read_scan_data_type();
    }

    sprintf(fname, "scan_%06d_rdc1.bin", myscan.scan_info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        uint32_t total_rdc1 = myscan.scan_info.num_range_bins
                * myscan.scan_info.num_pulses
                * myscan.scan_info.total_vrx;
        rdc1 = new cint16[total_rdc1];

        s.read_scan_data_type(rdc1, sizeof(rdc1[0]), total_rdc1);
        s.end_read_scan_data_type();

        sprintf(fname, "scan_%06d_rdc1exp.bin", myscan.scan_info.scan_sequence_number);
        len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            rdc1exp = new char[myscan.scan_info.num_range_bins * RDC1_EXP_SIZE_PER_RANGEBIN];
            bool ok = s.read_scan_data_type(rdc1exp, sizeof(rdc1exp[0]), myscan.scan_info.num_range_bins * RDC1_EXP_SIZE_PER_RANGEBIN);
            s.end_read_scan_data_type();
            aborted = false;
            setup();
            return ok;
        }
        else
        {
            delete [] rdc1;
            rdc1 = NULL;
        }
    }

    return false;
}


void                 RadarDataCube1_Impl::release()
{
    myscan.release_rdc1(*this);
}
