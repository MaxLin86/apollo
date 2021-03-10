// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/adccapturedata_impl.h"

namespace
{
    inline int8_t saturate(int16_t big)
    {
        if (big > 127)
        {
            return 127;
        }
        else if (big < -128)
        {
            return -128;
        }
        else
        {
            return int8_t(big);
        }
    }

    inline cint8 shrink(cint16& big)
    {
        return cint8(saturate(big.i), saturate(big.q));
    }

    inline cint16 expand(cint8& small)
    {
        return cint16(int16_t(small.i), int16_t(small.q));
    }

    inline void next_lane(INT& lane)
    {
        if (lane >= 8)
        {
            lane = 0;
        }
        else
        {
            lane = lane +1;
        }
    }
}

void     ADCCaptureData_Impl::get_adc_capture_info(uint16_t& rx_channel_mask, uint16_t& tx_channel_mask, uint32_t& samples_per_channel) const
{
    samples_per_channel = desc.samples_per_channel;

    if (desc.tx_one_rx_zero)
    {
        tx_channel_mask = desc.channel_bitmap;
        rx_channel_mask = 0U;
    }
    else
    {
        tx_channel_mask = 0U;
        rx_channel_mask = desc.channel_bitmap;
    }
}


void     ADCCaptureData_Impl::get_adc_initial_rx_lanes(uint8_t initial_rx_lane[NUM_RX_PER_BANK]) const
{
    for (INT i = 0; i < NUM_RX_PER_BANK; i++)
    {
        // nibble per receiver
        initial_rx_lane[i] = (desc.initial_rx_lane_numbers >> (4 * i)) & 0xF;
    }
}

uint32_t ADCCaptureData_Impl::get_adc_samples(cint8* user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map) const
{
    if (channel >= 12 || (0 == ((1U << channel) & desc.channel_bitmap)) || (0 == (lane_select_map & 0x1FFU)))
    {
        return 0U;
    }

    if (desc.tx_one_rx_zero)
    {
        lane_select_map = 0x1FF; // disable lane selection if this is a TX capture
    }

    cint8* samples8   = reinterpret_cast<cint8*>(adc_raw);
    cint16* samples16 = reinterpret_cast<cint16*>(adc_raw);
    uint32_t outcount = 0U, incount = 0U;

    INT lane = (desc.initial_rx_lane_numbers >> (4 * channel)) & 0xF;

    switch (desc.adc_capture_mode)
    {
    case UhdpADCDescriptor::ADC_CAPTURE_MODE_DATA_SPRAY:

        int16_t chan[2], num_channel, incr_incount;
        num_channel = 0;
        for (uint32_t c = 0; c < 12; c++)
        {
            if (desc.channel_bitmap & (1U << c))
            {
                chan[num_channel] = c;
                if (++num_channel == 2)
                {
                    break;
                }
            }
        }

        if (desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT)
        {
            user_buf_size_samples &= ~15;
            uint32_t numsamples = desc.samples_per_channel & ~15;

            incr_incount = (num_channel == 1 ) ? 4 : 8;

            for (incount = (channel == chan[0] ? 0 : 4); outcount < user_buf_size_samples && incount < (numsamples * num_channel); incount += incr_incount)
            {
                for (int i = 0; i < 4; i++)
                {
                    user_buf[outcount++] = samples8[incount + i];
                }
            }
        }
        else
        {
            user_buf_size_samples &= ~7;
            uint32_t numsamples = desc.samples_per_channel & ~7;
            incr_incount = (num_channel == 1 ) ? 16 : 32;
            for (incount = (channel == chan[0] ? 0 : 16); outcount < user_buf_size_samples && incount < (numsamples * num_channel); incount += incr_incount)
            {
                for (int i = 0; i < 16; i++)
                {
                    user_buf[outcount++] = shrink(samples16[incount + i]);
                }
            }
        }
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_RAW:
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_ADIE:
        for (uint32_t c = 0; c < 12; c++)
        {
            if (c == channel)
            {
                // ADie (RSU1) 2GB (2MByte) ADC capture buffer has I I Q Q cadence
                // The hardware writes into RSU1:           Ieven Iodd  Qeven Qodd
                // But when read from RSU1 it is reversed:  Qodd  Qeven Iodd  Ieven
                assert(desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT);
                for (incount = 0; outcount < user_buf_size_samples && incount < desc.samples_per_channel; incount += 2)
                {
                    if (lane_select_map & (1U << lane))
                    {
                        user_buf[outcount].i = samples8[incount + 1].q;
                        user_buf[outcount].q = samples8[incount + 0].q;
                        outcount++;
                    }

                    next_lane(lane);

                    if (lane_select_map & (1U << lane))
                    {
                        user_buf[outcount].i = samples8[incount + 1].i;
                        user_buf[outcount].q = samples8[incount + 0].i;
                        outcount++;
                    }

                    next_lane(lane);
                }
                break;
            }
            else if (desc.channel_bitmap & (1U << c))
            {
                // Skip an earlier saved channel
                if (desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT)
                {
                    samples8 += desc.samples_per_channel;
                }
                else
                {
                    samples16 += desc.samples_per_channel;
                }
            }
        }
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_REVA_RAW:
        /* serialized format of REVA raw capture */
        if (channel < 8)
        {
            for (incount = 0; outcount < user_buf_size_samples && incount < desc.samples_per_channel; incount++)
            {
                if (lane_select_map & (1U << lane))
                {
                    user_buf[outcount++] = samples8[8 * incount + channel];
                }

                next_lane(lane);
            }
        }
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_REVA_DATASPRAY:
        for (incount = 0; outcount < user_buf_size_samples && incount < desc.samples_per_channel; incount++)
        {
            if (lane_select_map & (1U << lane))
            {
                user_buf[outcount++] = samples8[incount];
            }

            next_lane(lane);
        }
        break;
    }

    return outcount;
}


uint32_t ADCCaptureData_Impl::get_adc_samples(cint16* user_buf, uint32_t user_buf_size_samples, uint8_t channel, uint16_t lane_select_map) const
{
    if (channel >= 12 || (0 == ((1U << channel) & desc.channel_bitmap)) || (0 == (lane_select_map & 0x1FFU)))
    {
        return 0U;
    }

    cint8* samples8   = reinterpret_cast<cint8*>(adc_raw);
    cint16* samples16 = reinterpret_cast<cint16*>(adc_raw);
    uint32_t outcount = 0U, incount = 0U;

    INT lane = (desc.initial_rx_lane_numbers >> (4 * channel)) & 0xF;

    switch (desc.adc_capture_mode)
    {
    case UhdpADCDescriptor::ADC_CAPTURE_MODE_DATA_SPRAY:
        int16_t chan[2], num_channel, incr_incount;
        num_channel = 0;
        incr_incount = 0;
        for (uint32_t c = 0; c < 12; c++)
        {
            if (desc.channel_bitmap & (1U << c))
            {
                chan[num_channel] = c;
                if (++num_channel == 2)
                {
                    break;
                }
            }
        }

        if (desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT)
        {
            user_buf_size_samples &= ~15;
            uint32_t numsamples = desc.samples_per_channel & ~15;

            incount = (channel == chan[0]) ? 0 : 4;
            incr_incount = (num_channel == 1) ? 4 : 8;
            for (; outcount < user_buf_size_samples && incount < (numsamples  * num_channel ); incount += incr_incount)
            {
                for (int i = 0; i < 4; i++)
                {
                   user_buf[outcount++] = expand(samples8[incount + i]);
                }
            }
        }
        else
        {
            user_buf_size_samples &= ~7;
            uint32_t numsamples = desc.samples_per_channel & ~7;

            incount = (channel == chan[0]) ? 0 : 4;
            incr_incount = (num_channel == 1) ? 4 : 8;
            for (; outcount < user_buf_size_samples && incount < (numsamples * num_channel ); incount += incr_incount)
            {
                for (int i = 0; i < 4; i++)
                {
                   user_buf[outcount++]  = samples16[incount + i];
                }
            }
        }
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_RAW:
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_ADIE:
        for (uint32_t c = 0; c < 12; c++)
        {
            if (c == channel)
            {
                // ADie (RSU1) 2GB (2MByte) ADC capture buffer has I I Q Q cadence
                // The hardware writes into RSU1:           Ieven Iodd  Qeven Qodd
                // But when read from RSU1 it is reversed:  Qodd  Qeven Iodd  Ieven
                assert(desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT);
                for (incount = 0; outcount < user_buf_size_samples && incount < desc.samples_per_channel; incount += 2)
                {
                    if (lane_select_map & (1U << lane))
                    {
                        user_buf[outcount].i = int16_t(samples8[incount + 1].q);
                        user_buf[outcount].q = int16_t(samples8[incount + 0].q);
                        outcount++;
                    }

                    next_lane(lane);

                    if (lane_select_map & (1U << lane))
                    {
                        user_buf[outcount].i = int16_t(samples8[incount + 1].i);
                        user_buf[outcount].q = int16_t(samples8[incount + 0].i);
                        outcount++;
                    }

                    next_lane(lane);
                }
                break;
            }
            else if (desc.channel_bitmap & (1U << c))
            {
                // Skip an earlier saved channel
                if (desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT)
                {
                    samples8 += desc.samples_per_channel;
                }
                else
                {
                    samples16 += desc.samples_per_channel;
                }
            }
        }
        break;


    case UhdpADCDescriptor::ADC_CAPTURE_MODE_REVA_RAW:
        /* serialized format of REVA raw capture */
        if (channel < 8)
        {
            assert(desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT);
            for (incount = 0; outcount < user_buf_size_samples && incount < desc.samples_per_channel; incount++)
            {
                if (lane_select_map & (1U << lane))
                {
                    user_buf[outcount++] = expand(samples8[8 * incount + channel]);
                }

                next_lane(lane);
            }
        }
        break;

    case UhdpADCDescriptor::ADC_CAPTURE_MODE_REVA_DATASPRAY:
        assert(desc.sample_bit_width == UhdpADCDescriptor::SAMPLE_WIDTH_8BIT);
        for (incount = 0; outcount < user_buf_size_samples && incount < desc.samples_per_channel; incount++)
        {
            if (lane_select_map & (1U << lane))
            {
                user_buf[outcount++] = expand(samples8[incount]);
            }

            next_lane(lane);
        }
        break;
    }

    return outcount;
}


void     ADCCaptureData_Impl::handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size)
{
    if (aborted)
    {
        ;
    }
    else if (message_type == UHDP_TYPE_ADC)
    {
        UhdpADCHeader* hdr = (UhdpADCHeader*)payload;
        payload    += sizeof(UhdpADCHeader);
        total_size -= sizeof(UhdpADCHeader);

        if (!adc_raw)
        {
            if (total_raw_bytes)
            {
                aborted = true;
                return;
            }

            total_raw_bytes = hdr->total_raw_bytes;
            raw_bytes_received = 0U;
            adc_raw = malloc(total_raw_bytes);

            desc = *reinterpret_cast<const UhdpADCDescriptor*>(payload);
            payload += sizeof(desc);
            total_size -= sizeof(desc);

            if (desc.validation_check() == false)
            {
                aborted = true;
                return;
            }
        }
        else if (raw_bytes_received != hdr->cur_byte_offset)
        {
            aborted = true;
            return;
        }

        memcpy(reinterpret_cast<char*>(adc_raw) + raw_bytes_received, payload, total_size);
        raw_bytes_received += total_size;
    }
}


void     ADCCaptureData_Impl::finish_uhdp(uint8_t last_message_type)
{
    if (raw_bytes_received != total_raw_bytes)
    {
        aborted = true;
    }

    if (aborted)
    {
        release();
    }
}


void     ADCCaptureData_Impl::release()
{
    if (myscan)
    {
        myscan->release_adc(*this);
    }
    else
    {
        delete this;
    }
}


bool     ADCCaptureData_Impl::deserialize(ScanSerializer& s)
{
    const uint32_t seq = myscan ? myscan->scan_info.scan_sequence_number : 0U;

    char fname[128];
    sprintf(fname, "scan_%06d_adcdata.bin", seq);

    uint32_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        s.read_scan_data_type(&desc, sizeof(desc), 1);
        len -= sizeof(desc);
        raw_bytes_received = total_raw_bytes = len;
        adc_raw = malloc(len);
        if (adc_raw && desc.validation_check() == true)
        {
            s.read_scan_data_type(adc_raw, 1, len);
            s.end_read_scan_data_type();
            return len == desc.total_data_size_bytes;
        }
        else
        {
            s.end_read_scan_data_type();
        }
    }
    else
    {
        /* Read-only support for older SabineA ADC captures, if the scan is
         * re-serialized the ADC data will be written in the new format */

        desc.set_defaults();
        sprintf(fname, "scan_%06d_adc_rawdata.bin", seq);
        len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            desc.adc_capture_mode = UhdpADCDescriptor::ADC_CAPTURE_MODE_REVA_RAW;
            desc.total_data_size_bytes = len;
            desc.samples_per_channel = len / sizeof(cint8) / 8;
            desc.channel_bitmap = 0xFF;

            adc_raw = malloc(len);
            if (adc_raw)
            {
                s.read_scan_data_type(adc_raw, 1, desc.total_data_size_bytes);
                s.end_read_scan_data_type();
                return true;
            }
        }
        else
        {
            sprintf(fname, "scan_%06d_adc_dataspray.bin", seq);
            len = s.begin_read_scan_data_type(fname);
            if (len)
            {
                desc.adc_capture_mode = UhdpADCDescriptor::ADC_CAPTURE_MODE_REVA_DATASPRAY;
                desc.total_data_size_bytes = len;
                desc.samples_per_channel = len / sizeof(cint8);
                desc.channel_bitmap = 1;

                adc_raw = malloc(len);
                if (adc_raw)
                {
                    s.read_scan_data_type(adc_raw, 1, desc.total_data_size_bytes);
                    s.end_read_scan_data_type();
                    return true;
                }
                else
                {
                    s.end_read_scan_data_type();
                }
            }
        }
    }

    return false;
}


bool     ADCCaptureData_Impl::serialize(ScanSerializer& s)
{
    const uint32_t seq = myscan ? myscan->scan_info.scan_sequence_number : 0U;

    char fname[128];
    sprintf(fname, "scan_%06d_adcdata.bin", seq);

    bool ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= s.write_scan_data_type(&desc, sizeof(desc), 1);
        ok &= s.write_scan_data_type(adc_raw, 1, total_raw_bytes);
        s.end_write_scan_data_type(!ok);
    }

    return ok;
}


ADCCaptureData* ADCCaptureData::load_raw_capture_file(const char* filename)
{
    ADCCaptureData_Impl* adc = new ADCCaptureData_Impl(NULL);

    FILE* fp = fopen(filename, "rb");
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        off_t size = ftello(fp);
        rewind(fp);

        if (fread(&adc->desc, 1, sizeof(adc->desc), fp) != sizeof(adc->desc))
        {
            ;
        }
        else if (adc->desc.validation_check() == false)
        {
            printf("ADC descriptor header does not pass validation checks\n");
        }
        else if (adc->desc.total_data_size_bytes != size - sizeof(adc->desc))
        {
            printf("Captured size does not match header total_data_size_bytes\n");
        }
        else
        {
            adc->total_raw_bytes = size - sizeof(adc->desc);
            adc->adc_raw = malloc(adc->total_raw_bytes);
            if (fread(adc->adc_raw, 1, adc->total_raw_bytes, fp) == adc->total_raw_bytes)
            {
                fclose(fp);
                return adc;
            }
        }

        fclose(fp);
    }

    delete adc;
    return NULL;
}
