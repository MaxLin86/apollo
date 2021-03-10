// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhstdlib.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhunistd.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhinet.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/state-manager.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "modules/drivers/radar/rocket_radar/driver/src/fastcapture.h"

/* these must be included in a particular order, let sleeping dogs lie */
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/discovery-diag.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-logmsg.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-antcfg.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-ppa.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//logaggr.h"

#include <signal.h>
#include <assert.h>

#ifdef _WIN32
#include <time.h>
#endif


enum {
    MIN_SUPPORTED_UHDP_VERSION = 28, // SRS 0.5.9
    MAX_SUPPORTED_UHDP_VERSION = 35, // SRS 0.7.6
};


static const char* uhdp_msg_type_names[] =
{
   "UHDP_TYPE_HELLO",
   "UHDP_TYPE_CAPTURE_CONTROL",
   "UHDP_TYPE_DISCOVERY",
   "UHDP_TYPE_SCAN_INFORMATION",
   "UHDP_TYPE_SCAN_READY",
   "UHDP_TYPE_PRINT",
   "UHDP_TYPE_SCAN_CONTROL",
   "UHDP_TYPE_PPA",
   "UHDP_TYPE_CONTROL_ACK",
   "UHDP_TYPE_CONN_CLOSE",
   "UHDP_TYPE_ADC",
   "UHDP_TYPE_RDC1",
   "UHDP_TYPE_RDC2",
   "UHDP_TYPE_RDC3",
   "UHDP_TYPE_STATIC_SLICE",
   "UHDP_TYPE_STATIC_SLICE_BIN",
   "UHDP_TYPE_HISTOGRAM",
   "UHDP_TYPE_REG_CORE_DUMP",
   "UHDP_TYPE_COVARIANCE",
   "UHDP_TYPE_SVD",
   "UHDP_TYPE_DETECTIONS",
   "UHDP_TYPE_CLUTTER_IMAGE",
   "UHDP_TYPE_MUSIC_SAMPLES",
   "UHDP_TYPE_ZERO_DOPPLER",
   "UHDP_TYPE_RANGE_BINS",
   "UHDP_TYPE_ANGLE_BINS",
   "UHDP_TYPE_ENV_SCAN_DATA",
   "UHDP_TYPE_POINT_CLOUD",
   "UHDP_TYPE_DPROBE_DC_DATA",
   "UHDP_TYPE_29",
   "UHDP_TYPE_30",
   "UHDP_TYPE_31",
   "UHDP_TYPE_32",
   "UHDP_TYPE_REQUEST_FLUSH",
   "UHDP_TYPE_CLOSE_ACK",
   "UHDP_TYPE_FLUSH",
   "UHDP_TYPE_DIAG_DISCOVERY",
   "UHDP_TYPE_DIAG_REQUEST",
   "UHDP_TYPE_DIAG_COMPLETE",
   "UHDP_TYPE_DIAG_FORCE_STOP",
   "UHDP_TYPE_LOG_PRODUCER_NAME",
   "UHDP_TYPE_ANTENNA_CONFIG",
   "UHDP_TYPE_LOG_MESSAGE",
   "UHDP_TYPE_LOG_TASK_NAME",
   "UHDP_TYPE_DATA_WINDOW",
   "UHDP_TYPE_STATE_QUERY",
   "UHDP_TYPE_STATE_RESPONSE",
   "UHDP_TYPE_STATE_PEEK",
   "UHDP_TYPE_STATE_POKE",
   "UHDP_TYPE_TIME_ALIGN",
   "UHDP_TYPE_LOG_AGGR",
   "UHDP_TYPE_LOG_AGGR_ACK",
   "UHDP_TYPE_TELEMETRY_DATA",
   "UHDP_TYPE_MUSIC_CONTROL",
   "UHDP_TYPE_THRESHOLD",
   "UHDP_TYPE_BULK_UPLOAD",
   "UHDP_TYPE_BULK_UPLOAD_ACK",
   "UHDP_TYPE_LOG_CONTROL",
   "UHDP_TYPE_LLD_CORE_DUMP",
   "UHDP_TYPE_DPROBE_CONFIG",
};

int Connection_Impl::ctrl_c_pressed /* = 0 */;

#if __APPLE__
sig_t        orignal_sigint_handler /* = NULL */;
#else
#if _WIN32
typedef void (*sighandler_t)(int);
#endif
sighandler_t orignal_sigint_handler /* = NULL */;
#endif

static void con_sigint_handler(int arg)
{
    if (++Connection_Impl::ctrl_c_pressed < 2)
    {
        /* clean shutdown path */
        printf("\nRRA: Caught CTRL+C, attempting a clean shutdown\n");

        /* pass the signal up to Python (if in the context of pyRRA) */
        orignal_sigint_handler(arg);
    }
    else
    {
        printf("RRA: Caught second CTRL+C, hard-shutdown\n");
        exit(1);
    }
}


Connection_Impl::~Connection_Impl()
{
    if (sockfd > 0)
    {
#ifdef _WIN32
        closesocket(sockfd);
        WSACleanup();
#else
        close(sockfd);
#endif
        sockfd = 0;
    }

    delete [] hw_unit_names;
    delete [] lld_core_dump_buffer;

    for (uint32_t i = 0; i < 256; i++)
    {
        delete uhdp_table[i];
    }

    free(const_cast<char*>(srs_version_str));
    free(const_cast<char*>(module_name));
    free(const_cast<char*>(module_type_name));
    free(const_cast<char*>(motherboard_type_name));
    free(const_cast<char*>(antennaboard_type_name));
    free(const_cast<char*>(antenna_module_type_name));
}


int Connection_Impl::create_socket()
{
#ifdef _WIN32
    WSADATA wsadata;
    // request socket version 2.2
    if (WSAStartup(MAKEWORD(2, 2), &wsadata) != 0)
    {
        printf("Windows socket err: %d\n", WSAGetLastError());
        return -1;
    }
#endif
    if (getenv("UHDP_HELLO_TIMEOUT"))
    {
        // If the user is setting UHDP_HELLO_TIMEOUT, they are almost certainly
        // dealing with a very slow simulation (likely a chip design simulator)
        // that is achingly slow. All timeouts should be scaled appropriately.
        timeout_scale = 100;
    }

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("Cannot create socket");
        return -1;
    }

    struct sockaddr_in myaddr;
    memset(&myaddr, 0, sizeof(myaddr));

    uint16_t listen_port = 0;
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(listen_port);

    if (bind(sockfd, (struct sockaddr*)&myaddr, sizeof(myaddr)) < 0)
    {
        perror("Unable to bind local socket\n");
        return -1;
    }

    if (!orignal_sigint_handler)
    {
        orignal_sigint_handler = signal(SIGINT, con_sigint_handler);
    }
    else
    {
        /* reset any previous cleanly handled ctrl+c event */
        ctrl_c_pressed = 0;
    }
    return 0;
}


void Connection_Impl::configure_socket_wait_time(int timeout_ms)
{
#if _WIN32
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms * 1000) - (tv.tv_sec * USEC_PER_SEC);
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
#endif
}


void Connection_Impl::send_uhdp(UhdpHeader* hdr)
{
    hdr->version = connection_uhdp_version;

    if (hdr->total_length)
    {
        sendto_mutex.acquire();
        int res = sendto(sockfd, (char*)hdr, hdr->total_length, 0, (struct sockaddr*)&radar_ip4addr, sizeof(radar_ip4addr));
        sendto_mutex.release();
        if (res < 0)
        {
            perror("sendto() error: ");
            abort_connection();
        }
    }
    else
    {
        counters.wrong_payload_length++;
        printf("Dropping outgoing UHDP message of type %d (%s), message length 0\n",
               hdr->message_type, uhdp_msg_type_names[hdr->message_type]);
    }
}


MakeConnectionError Connection_Impl::send_hello(uint32_t radar_ip, uint32_t timeout_sec, uint32_t uhdp_ver)
{
    uint16_t uhdp_port = UHDP_PORT;
    const char* portstr = getenv("SABINE_UHDP_PORT");
    if (portstr)
    {
        int envport = atoi(portstr);
        if (envport > 0 && envport < 0xFFFF)
        {
            uhdp_port = (uint16_t)envport;
        }
    }

    connection_uhdp_version = uhdp_ver;

    // setup radar UDP IPv4 address
    memset(&radar_ip4addr, 0, sizeof(radar_ip4addr));
    radar_ip4addr.sin_family = AF_INET;
    radar_ip4addr.sin_port = htons(uhdp_port);
    radar_ip4addr.sin_addr.s_addr = radar_ip;

    // learn local UDP listening port (since we bound to port 0, allowing the
    // O/S to pick an availabe port)
    struct sockaddr_in lcladdr;
    socklen_t addrlen = sizeof(lcladdr);
    memset(&lcladdr, 0, sizeof(lcladdr));
    getsockname(sockfd, (struct sockaddr*)&lcladdr, &addrlen);

    // construct HELLO message
    char msgbuf[MAX_UDP_DATAGRAM];
    memset(msgbuf, 0, sizeof(msgbuf));
    UhdpHeader*       uhdp = (UhdpHeader*)msgbuf;
    UhdpHelloMessage* msg  = (UhdpHelloMessage*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_HELLO;
    memset(msg, 0, sizeof(UhdpHelloMessage));
    msg->radar_ip_address  = ntohl(radar_ip);
    msg->logger_UDP_port   = ntohs(lcladdr.sin_port);
    char* ver_str = (char*)(msg + 1);
    int len;
    if (uhdp_ver >= 35)
    {
        len = snprintf(ver_str, 512, "%s", get_rra_version_string());
    }
    else
    {
        len = 0;
    }
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpHelloMessage) + len;
    send_uhdp(uhdp);

    // configure socket to block until a datagram or timeout
    configure_socket_wait_time(timeout_sec * 1000);

    // wait for HELLO ack (UHDP_TYPE_DISCOVERY)
    struct sockaddr_in remaddr;
    int recvlen = recvfrom(sockfd, msgbuf, MAX_UDP_DATAGRAM, 0, (struct sockaddr*)&remaddr, &addrlen);
    if (recvlen < 0)
    {
#if _WIN32
        if (WSAGetLastError() == WSAETIMEDOUT)
#else
        if (errno == EWOULDBLOCK)
#endif
        {
            return MCE_NO_RESPONSE;
        }
        else
        {
            perror("recvfrom() error: ");
            return MCE_SOCKET_FAILURE;
        }
    }

    timeval tv;
    gettimeofday(&tv, NULL);

    counters.total_packets_rcvd++;
    uint32_t num_log_message_enums = 0;

    //printf("received type %d (%s), message length %d\n", uhdp->message_type, uhdp_msg_type_names[uhdp->message_type], recvlen);

    if (UHDP_TYPE_DISCOVERY == uhdp->message_type)
    {
        UhdpDiscovery* disc = (UhdpDiscovery*)(uhdp + 1);
        assert(disc->discovery_state == 0);
        msgbuf[uhdp->total_length] = 0;
        char* disc_ver = (char*)(disc + 1);
        srs_version_str = strdup(disc_ver);

        // this is our first glance at the radar clock, it should be very close to
        // the time of connection, so we'll call it the connection epoch for
        // now. Later we might refine this estimation.
        uint64_t host_time = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;
        radar_host_delta = host_time - disc->timestamp;
        training_radar_host_delta = radar_host_delta;

        if (connection_uhdp_version >= 29)
        {
            disc_ver += strlen(disc_ver) + 1;
            module_name = strdup(disc_ver);
            disc_ver += strlen(disc_ver) + 1;
            module_type_name = strdup(disc_ver);
            disc_ver += strlen(disc_ver) + 1;
            motherboard_type_name = strdup(disc_ver);
            disc_ver += strlen(disc_ver) + 1;
            antennaboard_type_name = strdup(disc_ver);
            disc_ver += strlen(disc_ver) + 1;
            antenna_module_type_name = strdup(disc_ver);
            disc_ver += strlen(disc_ver) + 1;
        }
        else
        {
            module_name = strdup("Unknown");
            module_type_name = strdup("Uhknown");
            motherboard_type_name = strdup("Unknown");
            antennaboard_type_name = strdup("Unknown");
            antenna_module_type_name = strdup("Unknown");
        }
        if (connection_uhdp_version >= 30)
        {
            float geoms[5];
            memcpy(&geoms[0], disc_ver, sizeof(geoms)); // prevent alignment issues
            disc_ver += sizeof(geoms);
            rear_axle_distance = geoms[0];
            centerline_distance = geoms[1];
            mount_height = geoms[2];
            mount_azimuth = geoms[3];
            mount_elevation = geoms[4];
        }
        if (connection_uhdp_version >= 35)
        {
            num_log_message_enums = *(uint32_t*)disc_ver;
            disc_ver += sizeof(uint32_t);
        }
    }
    else if (UHDP_TYPE_PRINT == uhdp->message_type)
    {
        uhdp_table[uhdp->message_type]->handle_packet((char*)(uhdp + 1), recvlen);
        return MCE_UHDP_VERSION_MISMATCH;
    }
    else
    {
        // radar thinks we are already connected, but we are NOT!
        UhdpHeader hdr;
        hdr.message_type = UHDP_TYPE_CONN_CLOSE;
        hdr.total_length = sizeof(UhdpHeader);
        send_uhdp(&hdr);
        uh_usleep(20000);
        return MCE_INCOMPLETE_DISCOVERY;
    }

    if (radar_ip != remaddr.sin_addr.s_addr || htons(uhdp_port) != remaddr.sin_port)
    {
        char ipbuf[64];
        printf("Response came from unexpected IP %s or port %d\n",
                inet_ntop(AF_INET, &remaddr, ipbuf, sizeof(ipbuf)),
                ntohs(remaddr.sin_port));
        return MCE_INVALID_RADAR_IP;
    }

    struct DiscoveryAck
    {
        UhdpHeader hdr;
        uint32_t   discovery_msg_count;
    };

    DiscoveryAck ack;
    ack.hdr.version = connection_uhdp_version;
    ack.hdr.total_length = sizeof(DiscoveryAck);
    ack.hdr.message_type = UHDP_TYPE_DISCOVERY;
    ack.discovery_msg_count = 0;
    send_uhdp(&ack.hdr);

    assert(connection_uhdp_version >= 28);

    // Once we receive a HELLO ACK (Discovery-0), shorten the timeout to 500ms
    // and presume any recv timeout is caused by a missed packet and re-try the
    // connection
    configure_socket_wait_time(500);

    // Process all discovery messages before returning
    for(;;)
    {
        int recvlen = recv(sockfd, msgbuf, MAX_UDP_DATAGRAM, 0);
        if (recvlen < 0)
        {
            if (ctrl_c_pressed)
            {
                return MCE_SOCKET_FAILURE;
            }
#if _WIN32
            if (WSAGetLastError() == WSAETIMEDOUT)
#else
            if (errno == EWOULDBLOCK)
#endif
            {
                return MCE_INCOMPLETE_DISCOVERY;
            }
            else
            {
                perror("recvfrom() error: ");
                return MCE_SOCKET_FAILURE;
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

        //printf("received type %d (%s), message length %d\n", uhdp->message_type, uhdp_msg_type_names[uhdp->message_type], recvlen);

        if (UHDP_TYPE_DISCOVERY == uhdp->message_type)
        {
            UhdpDiscovery* disc = (UhdpDiscovery*)(uhdp + 1);
            if (disc->discovery_state == 1)
            {
                // Successful connection and discovery stage!
                break;
            }
            else
            {
                // the radar uhdp agent is in a wierd state, send a close
                // message and try to connect again later
                UhdpHeader hdr;
                hdr.message_type = UHDP_TYPE_CONN_CLOSE;
                hdr.total_length = sizeof(UhdpHeader);
                send_uhdp(&hdr);
                uh_usleep(20000);
                return MCE_INCOMPLETE_DISCOVERY;
            }
        }
        else
        {
            ack.discovery_msg_count++;
            send_uhdp(&ack.hdr);
            uhdp_table[uhdp->message_type]->handle_packet((char*)(uhdp + 1), recvlen);
        }
    }

    // in some circumstances, the radar might have more than one HELLO
    // queued up in its ethernet receive queue and it responds to each
    // of them in series. We want to ignore all of this extra discovery
    // info; it can only do harm to these classes
    uhdp_table[UHDP_TYPE_DISCOVERY]->disable();
    uhdp_table[UHDP_TYPE_LOG_PRODUCER_NAME]->disable();
    uhdp_table[UHDP_TYPE_LOG_MESSAGE]->disable();
    uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]->disable();
    uhdp_table[UHDP_TYPE_LOG_TASK_NAME]->disable();

    if (num_log_message_enums)
    {
        LogMsgDiscHandler* msghandler = static_cast<LogMsgDiscHandler*>(uhdp_table[UHDP_TYPE_LOG_MESSAGE]);
        if (msghandler->num_message_types != num_log_message_enums)
        {
            printf("Detected a dropped LOG message discovery packet, aborting this connection\n");
            UhdpHeader hdr;
            hdr.message_type = UHDP_TYPE_CONN_CLOSE;
            hdr.total_length = sizeof(UhdpHeader);
            send_uhdp(&hdr);
            uh_usleep(20000);
            return MCE_INCOMPLETE_DISCOVERY;
        }
    }

    connection_active = true;

    if (use_thread)
    {
        // configure socket to block until a datagram or a 100ms timeout
        // for polling thread efficiency
        configure_socket_wait_time(100);
        start();
    }
    else
    {
        // configure socket for short blocking wait when user is polling the
        // socket
        configure_socket_wait_time(1);
    }

    (void)query_radar_status_bitmask();

    return MCE_NO_ERROR;
}


MakeConnectionError last_err;


class Connection* make_connection(uint32_t radar_ip, uint32_t timeout_sec, UserLogAgent& log_agent, bool use_thread)
{
    srand(time(0)); // make rand() more random

    Connection_Impl* con = new Connection_Impl(log_agent, use_thread);

    if (con->create_socket() < 0)
    {
        last_err = MCE_SOCKET_FAILURE;
        delete con;
        return NULL;
    }

    uint32_t maxver = MAX_SUPPORTED_UHDP_VERSION;
    if (getenv("UHDP_OLD"))
    {
        // Radars with version 31..33 might not be able to negotiate a lower
        // UhDP version number, so we have this hack to start with version 31
        maxver = 31;
    }

    for (uint32_t ver = maxver; ver >= MIN_SUPPORTED_UHDP_VERSION; ver--)
    {
        last_err = con->send_hello(radar_ip, timeout_sec, ver);
        if (MCE_NO_ERROR == last_err)
        {
            return con;
        }
        else if (MCE_UHDP_VERSION_MISMATCH == last_err)
        {
            continue;
        }
        else
        {
            // misc other errors, must destroy this connection and try again
            delete con;
            return NULL;
        }
    }

    // No common UhDP versions supported
    printf("%s\n", UHPHandler::last_msg);
    delete con;
    return NULL;
}


Connection* make_good_connection(uint32_t radar_ip, UserLogAgent& log_agent, bool use_thread)
{
    Connection* con = NULL;
    const uint32_t MAX_RETRIES = 5;
    uint32_t retries = 0;

    while (!con)
    {
        con = make_connection(radar_ip, 2, log_agent, use_thread);

        if (!con)
        {
            if (MCE_INCOMPLETE_DISCOVERY == last_err || MCE_NO_RESPONSE == last_err)
            {
                if (++retries > MAX_RETRIES)
                {
                    break;
                }

                printf("Retrying connection...\n");
            }
        }
    }

    return con;
}


bool  Connection::connectionless_radar_telemetry_update(uint32_t radar_ip, const UhdpTelemetryData& tel)
{
    struct sockaddr_in radar_ip4addr;
    struct sockaddr_in host_ip4addr;

    memset(&host_ip4addr, 0, sizeof(host_ip4addr));
    memset(&radar_ip4addr, 0, sizeof(radar_ip4addr));

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        perror("Cannot create socket");
        return false;
    }

    const uint16_t listen_port = 0;
    host_ip4addr.sin_family = AF_INET;
    host_ip4addr.sin_addr.s_addr = htonl(INADDR_ANY);
    host_ip4addr.sin_port = htons(listen_port);

    if (bind(sockfd, (struct sockaddr*)&host_ip4addr, sizeof(host_ip4addr)) < 0)
    {
        perror("Unable to bind local socket\n");
        close(sockfd);
        return false;
    }

    // construct message
    char buf[sizeof(UhdpHeader) + sizeof(UhdpTelemetryData)];
    UhdpHeader* hdr = (UhdpHeader*)buf;

    hdr->message_type = UHDP_TYPE_TELEMETRY_ASYNC;
    hdr->total_length = sizeof(buf);
    hdr->version = MAX_SUPPORTED_UHDP_VERSION;
    memcpy(hdr + 1, &tel, sizeof(UhdpTelemetryData));

    // setup radar UDP IPv4 address
    radar_ip4addr.sin_family = AF_INET;
    radar_ip4addr.sin_port = htons(UHDP_PORT);
    radar_ip4addr.sin_addr.s_addr = radar_ip;

    int res = sendto(sockfd, (char*)hdr, hdr->total_length, 0, (struct sockaddr*)&radar_ip4addr, sizeof(radar_ip4addr));
    close(sockfd);

    return res == hdr->total_length;
}


MakeConnectionError get_last_connection_error()
{
    return last_err;
}
