#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/include/rra.h"
#include "modules/drivers/radar/quill_radar/driver/src/fastcapture.h"
#include "modules/drivers/radar/quill_radar/driver/src/connection_impl.h"

struct UhDPFastCaptureHeader
{
    uint32_t scan_sequence_number;
    uint32_t packet_index;
    uint32_t blob_offset;
};

FastCaptureHandler::FastCaptureHandler(Connection_Impl& con)
    : my_con(con)
    , cur_blob_segment(0)
    , num_blob_segments(0)
    , extra_ack_sequence(uint32_t(-1))
    , rdc3_blob_last_received(0)
    , completion_bitmap(NULL)
    , blob_buffer(NULL)
{
}


void FastCaptureHandler::clear()
{
    delete [] completion_bitmap;
    delete [] blob_buffer;
    completion_bitmap = NULL;
    blob_buffer = NULL;
    scan_sequence_number = uint32_t(-1);
    num_blob_segments = 0;
    rdc3_blob_last_received = 0;
}


void FastCaptureHandler::handle_packet(const char* payload, uint32_t len)
{
    const UhDPFastCaptureHeader& fc = *reinterpret_cast<const UhDPFastCaptureHeader*>(payload);
    payload += sizeof(UhDPFastCaptureHeader);
    len     -= sizeof(UhDPFastCaptureHeader);

    if (fc.packet_index == 0)
    {
        memcpy(&blob_header, payload, sizeof(blob_header));

        if (blob_header.magic != 0x1CDBD146UL)
        {
            printf("Unrecognized sparse RDC3 blob header\n");
            clear();
            return;
        }

#if 0
        printf("magic: %X\n", blob_header.magic);
        printf("version: %u\n", blob_header.version);
        printf("total_blob_size_bytes: %u\n", blob_header.total_blob_size_bytes);
        printf("max_datagram_size: %u\n", blob_header.max_datagram_size);
        printf("total_num_datagrams: %u\n", blob_header.total_num_datagrams);
        printf("rle0_ss_size_bytes: %u\n", blob_header.rle0_ss_size_bytes);
        printf("bkt_range_bin_start: %u\n", blob_header.bkt_range_bin_start[0]);
        printf("num_filtered_activations: %u\n", blob_header.num_filtered_activations);
#endif

        if (blob_buffer)
        {
            // the radar has moved on, we have to follow suit
            delete [] completion_bitmap;
            delete [] blob_buffer;
        }

        blob_buffer = new char[blob_header.total_blob_size_bytes];
        memcpy(blob_buffer, payload, len);

        num_blob_segments = (blob_header.total_num_datagrams + 31) / 32;
        completion_bitmap = new uint32_t[num_blob_segments];
        memset(completion_bitmap, 0, sizeof(uint32_t) * num_blob_segments);

        if (blob_header.total_num_datagrams & 31)
        {
            // mark "extra" bits as already completed to simplify completion
            // checks
            for (uint32_t i = blob_header.total_num_datagrams & 31; i < 32; i++)
            {
                completion_bitmap[num_blob_segments - 1] |= (1UL << i);
            }
        }

        cur_blob_segment = 0;
        completion_bitmap[0] |= 1;
        scan_sequence_number = fc.scan_sequence_number;
    }
    else if (fc.scan_sequence_number == scan_sequence_number)
    {
        if (!blob_buffer)
        {
            printf("no allocated blob buffer\n");
            return;
        }
        else if (fc.blob_offset + len <= blob_header.total_blob_size_bytes)
        {
            memcpy(blob_buffer + fc.blob_offset, payload, len);
        }
        else
        {
            printf("blob offset %d, len %d >= %d\n",
                   fc.blob_offset, len, blob_header.total_blob_size_bytes);
        }

        if (!completion_bitmap)
        {
            printf("no allocated completion bitmap\n");
            return;
        }
        else if (fc.packet_index < blob_header.total_num_datagrams)
        {
            //printf("scan_sequence_number %d packet_index %d blob_offset %d\n", fc.scan_sequence_number, fc.packet_index, fc.blob_offset);

            completion_bitmap[fc.packet_index >> 5] |= (1UL << (fc.packet_index & 31));

            uint32_t new_blob_segment = fc.packet_index >> 5;
            if (new_blob_segment != cur_blob_segment)
            {
                send_ack(false, new_blob_segment);
                cur_blob_segment = new_blob_segment;
            }

            struct timeval tv;
            gettimeofday(&tv, NULL);
            rdc3_blob_last_received = uint64_t(1000 * 1000) * tv.tv_sec + tv.tv_usec;
        }
        else
        {
            printf("packet index invalid\n");
        }

        // completion check
        for (uint32_t i = 0; i < num_blob_segments; i++)
        {
            if (completion_bitmap[i] != 0xFFFFFFFFUL)
            {
                return;
            }
        }

        // all done!
        send_ack(true, 0);

        if (my_con.apply_rdc3_blob(scan_sequence_number, blob_buffer, blob_header.total_blob_size_bytes))
        {
            // a scan object has taken ownership of the buffer, we drop our
            // pointer to it so we do not free it
            blob_buffer = NULL;
        }

        clear();
    }
    else
    {
        if (extra_ack_sequence != fc.scan_sequence_number)
        {
            // - "Ok boomer, you can shut up now, I'm not listening"
            scan_sequence_number = fc.scan_sequence_number;
            extra_ack_sequence = scan_sequence_number;
            send_ack(true, 0);
            clear();
        }
    }
}


void FastCaptureHandler::poll(uint64_t cur_timestamp)
{
    if (rdc3_blob_last_received == 0)
    {
        ; // disabled
    }
    else if ((cur_timestamp - rdc3_blob_last_received) >= 10000) // 10.0ms without receiving a packet
    {
        //printf("fast capture timeout after %ld us\n", unsignedcur_timestamp - rdc3_blob_last_received);
        // reset cur_blob_segment, allow all dirty blocks to be sent
        cur_blob_segment = 0;
        send_ack(false, num_blob_segments);
        rdc3_blob_last_received = cur_timestamp;
    }
}


void FastCaptureHandler::send_ack(bool done, uint32_t new_blob_segment)
{
    char buf[1500];
    UhdpHeader* hdr = (UhdpHeader*)buf;
    hdr->message_type = UHDP_TYPE_FAST_CAPTURE_ACK;
    uint32_t* words = (uint32_t*)(hdr + 1);
    words[0] = scan_sequence_number;

    if (done)
    {
        hdr->total_length = sizeof(UhdpHeader) + 2 * sizeof(uint32_t);
        words[1] = 1; // ACK
        my_con.send_uhdp(hdr);
    }
    else
    {
        words[1] = 0; // NAK

        uint32_t send_blocks = 0;
        for (uint32_t seg = cur_blob_segment; seg < num_blob_segments; seg++)
        {
            if (seg == new_blob_segment)
            {
                break;
            }
            else if ((send_blocks == 0) && (completion_bitmap[seg] != 0xFFFFFFFFUL))
            {
                words[2] = seg;
                words[3 + send_blocks] = completion_bitmap[seg];
                send_blocks++;
            }
            else if (send_blocks > 0)
            {
                words[3 + send_blocks] = completion_bitmap[seg];
                send_blocks++;
            }
        }

        // if none dirty, exit without sending NAK
        if (send_blocks)
        {
            printf("Nak seq %d %d blocks\n", scan_sequence_number, send_blocks);
            hdr->total_length = sizeof(UhdpHeader) + (3 + send_blocks) * sizeof(uint32_t);
            my_con.send_uhdp(hdr);
        }
    }
}
