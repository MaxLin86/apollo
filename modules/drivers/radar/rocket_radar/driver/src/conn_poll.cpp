// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "modules/drivers/radar/rocket_radar/driver/src/srs-profiling.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/diag/api/diag_calibrate_structs.h"

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhstdlib.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhunistd.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhinet.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/state-manager.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "modules/drivers/radar/rocket_radar/driver/src/fastcapture.h"

/* these must be included in a particular order, let sleeping dogs lie */
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/discovery-diag.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-logmsg.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-antcfg.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-ppa.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//logaggr.h"

#include <assert.h>
#include <signal.h>
#include <stdio.h>

#ifdef _WIN32
#include <time.h>
#else
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#endif

void Connection_Impl::thread_main()
{
#if _WIN32
    FD_SET write_set;
    FD_SET read_set;
#else
    fd_set write_set;
    fd_set read_set;
#endif

    while (connection_active)
    {
#if _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#endif
        FD_ZERO(&write_set);
        FD_ZERO(&read_set);
        FD_SET(sockfd, &read_set);
        int nfds = sockfd + 1;

        // wait for a message, or 1ms
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        select(nfds, &read_set, &write_set, NULL, &tv);

        poll_socket();
    }
}

void Connection_Impl::poll_socket()
{
    if (mysensorgroup)
    {
        poll_sensor_group();
        return;
    }

    poll_mutex.acquire();

    poll_socket_internal();

    poll_retransmit_timers();

    poll_mutex.release();
}


void Connection_Impl::poll_retransmit_timers()
{
    if (!connection_active)
    {
        return;
    }
    if (ctrl_c_pressed && resend_count > 0)
    {
        abort_connection();
        return;
    }

    timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t cur_timestamp = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

    FastCaptureHandler* fc = static_cast<FastCaptureHandler*>(uhdp_table[UHDP_TYPE_FAST_CAPTURE]);
    fc->poll(cur_timestamp);

    // check for timeouts
    if (diag_last_resend)
    {
        uint64_t timeout = uint64_t(USEC_PER_SEC) * 5 * timeout_scale;
        if (cur_timestamp - diag_last_resend > timeout)
        {
            if (++resend_count < MAX_RESEND_COUNT)
            {
                send_uhdp((UhdpHeader*)diag_msgbuf);
                diag_last_resend = cur_timestamp;
                counters.diag_retransmissions++;
            }
            else
            {
                diag_is_hung = true;
                diag_last_resend = cur_timestamp;
                diag_comp_event.trigger();
            }
        }
    }
    if (core_dump_last_resend)
    {
        uint64_t timeout = uint64_t(USEC_PER_SEC) * 5 * timeout_scale;
        if (cur_timestamp - core_dump_last_resend > timeout)
        {
            if (++resend_count < MAX_RESEND_COUNT)
            {
                send_uhdp((UhdpHeader*)lld_core_dump_msg_buffer);
                core_dump_last_resend = cur_timestamp;
            }
            else
            {
                core_dump_last_resend = cur_timestamp;
                core_dump_ack_event.trigger();
            }
        }
    }
    if (peek_last_resend)
    {
        uint64_t timeout = uint64_t(USEC_PER_SEC) * 5 * timeout_scale;
        if (cur_timestamp - peek_last_resend > timeout)
        {
            if (++resend_count < MAX_RESEND_COUNT)
            {
                send_uhdp((UhdpHeader*)peek_buffer);
                peek_last_resend = cur_timestamp;
            }
            else
            {
                printf("Peek/Poke is not responding, aborting connection\n");
                abort_connection();
                return;
            }
        }
    }
    if (status_last_resend)
    {
        uint64_t timeout = uint64_t(USEC_PER_SEC) * 5 * timeout_scale;
        if (cur_timestamp - status_last_resend > timeout)
        {
            if (++resend_count < 3)
            {
                send_uhdp((UhdpHeader*)status_buffer);
                status_last_resend = cur_timestamp;
            }
            else
            {
                printf("Status Request is not responding, aborting connection\n");
                abort_connection();
                return;
            }
        }
    }
    if (control_last_resend)
    {
        uint64_t timeout = uint64_t(USEC_PER_SEC) * 5 * timeout_scale;
        if (cur_timestamp - control_last_resend > timeout)
        {
            if (++resend_count < MAX_RESEND_COUNT)
            {
                send_uhdp((UhdpHeader*)control_msg_buffer);
                control_last_resend = cur_timestamp;
            }
            else
            {
                printf("No response to scan or capture control message, aborting connection\n");
                abort_connection();
                return;
            }
        }
    }
    if (request_close_time)
    {
        // only request close once, timeout after two seconds.  This is the
        // stall you see in RCC when trying to close a dead connection
        if (cur_timestamp - request_close_time > uint64_t(USEC_PER_SEC) * 2 * timeout_scale)
        {
            printf("Radar is not responding to connection close requests, aborting connection\n");
            abort_connection();
            return;
        }
    }
}


void Connection_Impl::poll_socket_internal()
{
    if (!connection_active)
    {
        return;
    }

    // these pointers never change, so we can define them outside the loop
    char msgbuf[MAX_UDP_DATAGRAM];
    const UhdpHeader*    uhdp = (const UhdpHeader*)msgbuf;
    const char*       payload = (const char*)(uhdp + 1);
    const UhdpDataHeader* win = (const UhdpDataHeader*)payload;

    do
    {
        // process received datagrams until it returns empty
        int recvlen = recv(sockfd, msgbuf, MAX_UDP_DATAGRAM, 0);
        if (recvlen < 0)
        {
#if _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK || err == WSAETIMEDOUT)
            {
                // timeout
                break;
            }
#else
            if (errno == EWOULDBLOCK)
            {
                // timeout
                break;
            }
#endif
            else
            {
                perror("recvfrom() error: ");
                abort_connection();
                return;
            }
        }

        counters.total_packets_rcvd++;

        if (uhdp->version != connection_uhdp_version)
        {
            counters.wrong_uhdp_version++;
            continue;
        }

        if (uhdp->total_length != recvlen)
        {
            counters.wrong_payload_length++;
            continue;
        }

        recvlen -= sizeof(UhdpHeader);

        if (uhdp->message_type == UHDP_TYPE_TIME_ALIGN)
        {
            const UhdpTimeAlignment& ta = *reinterpret_cast<const UhdpTimeAlignment*>(payload);

            timeval tv;
            gettimeofday(&tv, NULL);
            uint64_t cur_timestamp = uint64_t(USEC_PER_SEC)* tv.tv_sec + tv.tv_usec;
            int64_t new_delta = cur_timestamp - ta.timestamp64;

            // if this delta is smaller than the previously known delta, assume
            // that we had slightly less latency in getting this packet and
            // record the new delta. otherwise, take the new delta every 100
            // scans in case there is slow drift.
            if ((new_delta < training_radar_host_delta) || (skew_update_counter == 0))
            {
                training_radar_host_delta = new_delta;

                skew_update_counter++;
            }
            else if (frame_interval_base_host_time.tv_sec && (skew_update_counter == 32))
            {
                skew_update_counter = 0;

                radar_host_delta = training_radar_host_delta;
                //printf("delta: %lld\n", radar_host_delta);
                send_frame_interval_base_time();
            }
            else
            {
                skew_update_counter++;
            }

            last_time_align = ta;
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_SCAN_INFORMATION)
        {
            const UhdpScanInformation& msg = *reinterpret_cast<const UhdpScanInformation *>(payload);

            // UhdpScanInformation.current_time   - radar 32bit timestamp when message was sent
            // UhdpScanInformation.scan_timestamp - radar 32bit timestamp when this scan ended
            int32_t usec_since_scan_end;
            if (msg.clock_tick_denominator >= 1)
            {
                // timestamps are in units of ticks, counter wraps at MAXUINT32
                uint32_t ticks = msg.current_time - msg.scan_timestamp;
                usec_since_scan_end = ticks * msg.clock_tick_numerator / msg.clock_tick_denominator;
            }
            else
            {
                // timestamps are in units of microseconds, wraps early
                if (msg.current_time < msg.scan_timestamp)
                {
                    // Wrap detected. The 32bit clock counter wraps around at (2^36) / 200 =
                    // 343597383 = 0x147AE147. Since the counter wraps early, we cannot
                    // rely on integer subtraction giving the correct answer
                    usec_since_scan_end = msg.current_time + (0x147AE147 - msg.scan_timestamp);
                }
                else
                {
                    usec_since_scan_end = msg.current_time - msg.scan_timestamp;
                }
            }

            timeval host_local_time;
            if (last_time_align.scan_sequence_number == msg.scan_sequence_number)
            {
                // leverage the time alignment message to map the scan end
                // directly into host time

                uint64_t radar_scan_end = last_time_align.timestamp64 - usec_since_scan_end;
                uint64_t host_scan_end = radar_scan_end + radar_host_delta;
                host_local_time.tv_sec = host_scan_end / USEC_PER_SEC;
                host_local_time.tv_usec = host_scan_end - host_local_time.tv_sec * USEC_PER_SEC;
            }
            else
            {
                // this should be a rare condition (packet drop), derive scan
                // end time by subtracting it from the current time (less
                // accurate, since it cannot account for network latencies)

                gettimeofday(&host_local_time, NULL);

                if (usec_since_scan_end > host_local_time.tv_usec)
                {
                    host_local_time.tv_sec--;
                    usec_since_scan_end -= host_local_time.tv_usec;
                    host_local_time.tv_usec = USEC_PER_SEC - usec_since_scan_end;
                }
                else
                {
                    host_local_time.tv_usec -= usec_since_scan_end;
                }
            }

            while (host_local_time.tv_usec > USEC_PER_SEC)
            {
                host_local_time.tv_sec++;
                host_local_time.tv_usec -= USEC_PER_SEC;
            }

            ScanObject_Impl* obj = new ScanObject_Impl();
            obj->handle_scan_info(&msg, recvlen, host_local_time, connection_uhdp_version);
            if (srs_version_str && module_name)
            {
                obj->srs_version_str = strdup(srs_version_str);
                obj->module_name = strdup(module_name);
                obj->module_type_name = strdup(module_type_name);
                obj->motherboard_type_name = strdup(motherboard_type_name);
                obj->antennaboard_type_name = strdup(antennaboard_type_name);
                obj->antenna_module_type_name = strdup(antenna_module_type_name);
            }
            else
            {
                obj->srs_version_str = strdup("N/A");
                obj->module_name = strdup("N/A");
                obj->module_type_name = strdup("N/A");
                obj->motherboard_type_name = strdup("N/A");
                obj->antennaboard_type_name = strdup("N/A");
                obj->antenna_module_type_name = strdup("N/A");
            }
            obj->rear_axle_distance = rear_axle_distance;
            obj->centerline_distance = centerline_distance;
            obj->mount_height = mount_height;
            obj->mount_azimuth = mount_azimuth;
            obj->mount_elevation = mount_elevation;

            // append scan to input list (raw radar output order)
            if (last_in_scan)
            {
                last_in_scan->next_scan_object = obj;
                last_in_scan = obj;
            }
            else
            {
                cur_in_scan = last_in_scan = obj;
            }

            char buf[sizeof(UhdpHeader) + sizeof(uint32_t)];
            UhdpHeader* hdr = (UhdpHeader*)buf;
            hdr->message_type = UHDP_TYPE_SCAN_READY;
            hdr->total_length = sizeof(UhdpHeader) + sizeof(uint32_t);
            *(uint32_t*)(hdr + 1) = msg.scan_sequence_number + 1;

            send_uhdp(hdr);
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_LOG_AGGR)
        {
            uhdp_table[uhdp->message_type]->handle_packet(payload, recvlen);

            UhdpHeader* msg = const_cast<UhdpHeader*>(uhdp);
            msg->message_type = UHDP_TYPE_LOG_AGGR_ACK;
            msg->total_length = sizeof(UhdpHeader) + sizeof(uint32_t);
            send_uhdp(msg);
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_CLOSE_ACK)
        {
            abort_connection();
            return;
        }
        else if (uhdp->message_type == UHDP_TYPE_DIAG_COMPLETE)
        {
            if (diag_last_resend)
            {
                UhdpDiagComplete* comp = (UhdpDiagComplete*)(uhdp + 1);
                if (comp->diag_sequence_number == diag_sequence_number)
                {
                    // mark diag as complete
                    diag_last_resend = 0;
                    // overwrite input resend buffer with the output message
                    memcpy(diag_msgbuf, uhdp, recvlen + sizeof(UhdpHeader));
                    // unlock the diag mutex, allowing user thread to resume
                    diag_comp_event.trigger();
                }
                else
                {
                    printf("RRA: Ignoring diag complete response for seq %X, expecting %X\n",
                           comp->diag_sequence_number, diag_sequence_number);
                }
            }
            else
            {
                printf("RRA: Ignoring redundant diag complete response\n");
            }
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_CONTROL_ACK)
        {
            if (control_last_resend)
            {
                control_last_resend = 0;
                if (recvlen == sizeof(uint32_t) + sizeof(uint64_t))
                {
                    timeval tv;
                    gettimeofday(&tv, NULL);
                    uint64_t cur_timestamp = uint64_t(USEC_PER_SEC)* tv.tv_sec + tv.tv_usec;
                    uint64_t timestamp64;
                    memcpy(&timestamp64, (char*)(uhdp + 1) + sizeof(uint32_t), sizeof(timestamp64));

                    int64_t new_delta = cur_timestamp - timestamp64;
                    if (new_delta < radar_host_delta)
                    {
                        radar_host_delta = new_delta;
                    }
                }
                // overwrite input resend buffer with the ACK message
                memcpy(control_msg_buffer, uhdp, recvlen + sizeof(UhdpHeader));
                // unlock the diag mutex, allowing user thread to resume
                control_ack_event.trigger();
            }
            else
            {
                printf("Control ACK received but none expected\n");
            }
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_STATE_PEEK ||
                 uhdp->message_type == UHDP_TYPE_STATE_POKE)
        {
            if (peek_last_resend)
            {
                UhdpPeekResp* resp = (UhdpPeekResp*)payload;

                if (resp->sequence_id == peek_sequence_number)
                {
                    peek_last_resend = 0;
                    memcpy(peek_buffer, uhdp, recvlen + sizeof(UhdpHeader));
                    peek_comp_event.trigger();
                }
            }
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_STATE_RESPONSE)
        {
            if (status_last_resend)
            {
                status_last_resend = 0;
                memcpy(status_buffer, uhdp, recvlen + sizeof(UhdpHeader));
                peek_comp_event.trigger();
            }
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_FLUSH)
        {
            // the radar claims no scans are queued for transmission, so we
            // clear the incoming and pending lists
            ScanObject_Impl* orig = cur_in_scan;

            if (cur_in_scan)
            {
                cur_in_scan->finish_uhdp(last_msg_type);
            }

            last_msg_type = UHDP_TYPE_HELLO;

            // all scan objects in the incoming linked list are complete
            while (cur_in_scan)
            {
                if (cur_in_scan != orig)
                {
                    if (cur_in_scan->scan_info.num_pulses != 8)
                    {
                        // do not count dummy scans
                        counters.dropped_scans++;
                    }
                }
                ScanObject_Impl* scan = cur_in_scan;
                cur_in_scan = cur_in_scan->next_scan_object;
                queue_completed_scan(scan);
            }
            last_in_scan = NULL;
            FastCaptureHandler* fc = static_cast<FastCaptureHandler*>(uhdp_table[UHDP_TYPE_FAST_CAPTURE]);
            fc->clear();
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_LLD_CORE_DUMP)
        {
            if (core_dump_last_resend)
            {
                UhdpLLDCoreDumpResponse* msg  = (UhdpLLDCoreDumpResponse*)(uhdp + 1);

                if (msg->hardware_unit_name[0] == '?')
                {
                    uint32_t namelen = recvlen - sizeof(*msg);
                    if (!hw_unit_names)
                    {
                        hw_unit_names = new char[namelen + 1];
                        memcpy(const_cast<char*>(hw_unit_names), (char*)(msg + 1), namelen);
                        const_cast<char*>(hw_unit_names)[namelen] = 0;
                        core_dump_last_resend = 0;
                        core_dump_ack_event.trigger();
                    }
                    else
                    {
                        counters.dropped_data_packets++;
                    }
                }
                else
                {
                    uint32_t tuple_length = recvlen - sizeof(*msg);
                    uint32_t tuple_count = tuple_length / (2 * sizeof(uint32_t));
                    uint32_t* tuples = (uint32_t*)(msg + 1);

                    if (msg->cur_tuple_pair == 0)
                    {
                        // re-allocate buffer on first message receive
                        delete [] lld_core_dump_buffer;
                        lld_core_dump_buffer = new uint32_t[2 * msg->total_tuple_pairs];
                        lld_core_dump_size = sizeof(uint32_t) * 2 * msg->total_tuple_pairs;

                        memcpy(lld_core_dump_buffer, tuples, tuple_length);
                        lld_core_dump_receive_count = tuple_count;
                    }
                    else if (lld_core_dump_size != msg->total_tuple_pairs * 2 * sizeof(uint32_t))
                    {
                        // first tuple message was dropped
                        counters.dropped_data_packets++;
                    }
                    else if (!lld_core_dump_buffer)
                    {
                        // first tuple message was dropped
                        counters.dropped_data_packets++;
                    }
                    else if (msg->cur_tuple_pair != lld_core_dump_receive_count)
                    {
                        // intermediate tuple message was dropped
                        counters.dropped_data_packets++;
                    }
                    else
                    {
                        memcpy(&lld_core_dump_buffer[2 * lld_core_dump_receive_count], tuples, tuple_length);
                        lld_core_dump_receive_count += tuple_count;
                    }

                    if (lld_core_dump_receive_count == msg->total_tuple_pairs)
                    {
                        core_dump_last_resend = 0;
                        core_dump_ack_event.trigger();
                    }
                }
            }
            continue;
        }
        else if ((uhdp->message_type < UHDP_TYPE_ADC) || (uhdp->message_type > UHDP_TYPE_FLUSH))
        {
            /* non-radar-data message type */
            uhdp_table[uhdp->message_type]->handle_packet(payload, recvlen);
            continue;
        }

        /**** Radar Data Packet, with Data Window Header ****/

        if (win->scan_sequence_num != last_scan_seq)
        {
            current_window = INITIAL_PACKET_WINDOW;
            cur_pkt_counter = 0;

            // new scan sequence number indicates previous scan is complete
            if (cur_in_scan)
            {
                cur_in_scan->finish_uhdp(last_msg_type);
            }

            last_msg_type = UHDP_TYPE_HELLO; // NOP

            // flush incoming list up to new sequence number; these scans were dropped
            while (cur_in_scan)
            {
                if (win->scan_sequence_num == cur_in_scan->scan_info.scan_sequence_number)
                {
                    break;
                }
                if (last_scan_seq != cur_in_scan->scan_info.scan_sequence_number)
                {
                    counters.dropped_scans++;
                }

                ScanObject_Impl* scan = cur_in_scan;
                cur_in_scan = cur_in_scan->next_scan_object;
                queue_completed_scan(scan);
            }

            // note cur_in_scan should be non-null 99.9% of the time, the only
            // time it should be null is if the scan information message was dropped
            if (!cur_in_scan)
            {
                last_in_scan = NULL;
            }
        }
        else if (uhdp->message_type != last_msg_type)
        {
            // new message type, finalize previous message type
            if (cur_in_scan)
            {
                if (win->packet_counter != cur_pkt_counter)
                {
                    cur_in_scan->abort_uhdp(last_msg_type);
                }
                else
                {
                    cur_in_scan->finish_uhdp(last_msg_type);
                }
            }
            last_msg_type = UHDP_TYPE_HELLO; // NOP
        }

        if (win->packet_counter != cur_pkt_counter)
        {
            /* a packet was dropped, abort this file */
            if (cur_in_scan)
            {
                cur_in_scan->abort_uhdp(uhdp->message_type);
            }
            counters.dropped_data_packets += win->packet_counter - cur_pkt_counter;
            cur_pkt_counter = win->packet_counter + 1;
            continue;
        }

        if (cur_pkt_counter + (additional_packet_window / 2) > current_window)
        {
            current_window += additional_packet_window;

            char buf[sizeof(UhdpHeader) + sizeof(UhdpDataWindow)];
            UhdpHeader* hdr = (UhdpHeader*)buf;
            hdr->message_type = UHDP_TYPE_DATA_WINDOW;
            hdr->total_length = sizeof(buf);

            UhdpDataWindow* winmsg = (UhdpDataWindow*)(hdr + 1);
            winmsg->scan_sequence_number = win->scan_sequence_num;
            winmsg->max_packet_counter = current_window;

            send_uhdp(hdr);
        }

        if (cur_in_scan)
        {
            cur_in_scan->handle_uhdp(uhdp->message_type, payload, recvlen);
        }
        else
        {
            counters.data_without_scan_info++;
        }

        cur_pkt_counter = win->packet_counter + 1;
        last_msg_type = uhdp->message_type;
        last_scan_seq = win->scan_sequence_num;
    }
    while (!mysensorgroup);
}


bool            Connection_Impl::scan_available()
{
    scanlist_mutex.acquire();
    bool ret = !!first_out_scan;
    scanlist_mutex.release();

    return ret;
}


ScanObject*     Connection_Impl::poll_completed_scan()
{
    if (scan_available())
    {
        scanlist_mutex.acquire();

        ScanObject_Impl* scan = first_out_scan;
        first_out_scan = first_out_scan->next_scan_object;
        if (!first_out_scan)
        {
            last_out_scan = NULL;
        }

        scanlist_mutex.release();

        // the user now owns this ScanObject; the Connection has no references to it
        return scan;
    }
    else
    {
        return NULL;
    }
}


void Connection_Impl::abort_connection()
{
    // this connection is closing, free all queued ScanObjects
    while (cur_in_scan)
    {
        ScanObject_Impl* cur = cur_in_scan;
        cur_in_scan = cur_in_scan->next_scan_object;
        delete cur;
    }
    last_in_scan = NULL;

    scanlist_mutex.acquire();
    while (first_out_scan)
    {
        ScanObject_Impl* cur = first_out_scan;
        first_out_scan = first_out_scan->next_scan_object;
        delete cur;
    }
    last_out_scan = NULL;
    scanlist_mutex.release();

    // unblock any threads waiting for a scan
    connection_active = false;
    scanlist_cond.set(0);

    // unblock any blocked threads
    diag_comp_event.trigger();
    peek_comp_event.trigger();
    control_ack_event.trigger();

    last_err = CON_SOCKET_FAILURE;
}
