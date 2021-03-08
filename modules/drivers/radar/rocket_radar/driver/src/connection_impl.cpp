#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/sra.h"
#include "modules/drivers/radar/rocket_radar/driver/src/srs-profiling.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/diag/api/diag_calibrate_structs.h"

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhstdlib.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhunistd.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhinet.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/state-manager.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "modules/drivers/radar/rocket_radar/driver/include/vehiclecan.h"

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

static const char* stars =
"******************************************************************************\n";
static const char* radar_hang_message =
"*** The radar has stopped responding to ethernet packets.  Attach the module's\n"
"*** serial cable to a PC and run uart_monitor.py to determine if the module\n"
"*** encountered an exception or if the module is entirely unresponsive. To file a\n"
"*** bug report, reboot and use backup.py to record this module's calibrations and\n"
"*** configurations, then reproduce the problem and keep full logs\n";

static const char* diag_hang_message =
"*** Diag %s::%s has hung, or is taking too long to complete.  Diags are not supposed\n"
"*** to take longer than one second, this one has taken more than 5 seconds.  We do\n"
"*** allow exceptions for this rule in a few situations, particularly for calibration\n"
"*** To extend the timeout one must set the environment variable UHDP_HELLO_TIMEOUT\n"
"*** in your script. For example:\n"
"***\n"
"*** import os; os.environ['UHDP_HELLO_TIMEOUT'] = '5'\n"
"***\n";

static bool ctrl_c_pressed /* = false */;
#if __APPLE__
sig_t        orignal_sigint_handler /* = NULL */;
#else
#if _WIN32
typedef void (*sighandler_t)(int);
#endif
sighandler_t orignal_sigint_handler /* = NULL */;
#endif

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

static void con_sigint_handler(int arg)
{
    static bool once /* = false */;

    if (!once)
    {
        /* clean shutdown path */
        printf("\nSRA: Caught CTRL+C, attempting a clean shutdown\n");
        ctrl_c_pressed = true;
        once = true;
        /* pass the signal up to Python (if in the context of pySRA) */
        orignal_sigint_handler(arg);
    }
    else
    {
        printf("SRA: Caught second CTRL+C, hard-shutdown\n");
        exit(1);
    }
}

struct NopHandler : public ProtHandlerBase
{
    NopHandler(Connection_Impl& impl, uint8_t type) : my_con(impl), my_type(type) {}

    void handle_packet(const char* payload, uint32_t len)
    {
        printf("NOP handler type %u, %x, len=%u\n", my_type, my_type, len);
        my_con.counters.wrong_message_type++;
    }

    Connection_Impl& my_con;
    uint32_t my_type;
};

struct UHPHandler : public ProtHandlerBase
{
    // modern SRS only sends this message in response to UHDP version mismatch,
    // so we treat it very specially
    void handle_packet(const char* payload, uint32_t len)
    {
        //uint32_t cpuid = *(uint32_t*)payload;
        payload += sizeof(uint32_t);
        len -= sizeof(uint32_t);
        char* msg = const_cast<char*>(payload);
        msg[len - 5] = 0;
        free(last_msg);
        last_msg = strdup(msg);
    }

    static char* last_msg;
};

char* UHPHandler::last_msg;

struct EnvControlDataHandler : public ProtHandlerBase
{
    EnvControlDataHandler(Connection_Impl& impl) : my_con(impl) {}

    virtual ~EnvControlDataHandler() {}

    virtual void handle_packet(const char* payload, uint32_t len)
    {
        if (len >= sizeof(UhdpEnvControl))
        {
            const UhdpEnvControl& msg = *(const UhdpEnvControl*)payload;
            payload += sizeof(UhdpEnvControl);
            len     -= sizeof(UhdpEnvControl);

            my_con.log_agent.received_control_message((UhdpControlMessageType)msg.message_type, payload, len);

            // send ACK
            char msgbuf[sizeof(UhdpHeader) + sizeof(UhdpEnvControl)];
            UhdpHeader*     uhdp = (UhdpHeader*)msgbuf;
            UhdpEnvControl* emsg  = (UhdpEnvControl*)(uhdp + 1);

            uhdp->message_type = UHDP_TYPE_ENV_CONTROL_ACK;
            uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpEnvControl);
            memcpy(emsg, &msg, sizeof(UhdpEnvControl));

            my_con.send_uhdp(uhdp);
        }
    }

    Connection_Impl& my_con;
};

struct EnvControlAckHandler : public ProtHandlerBase
{
    EnvControlAckHandler(Connection_Impl& impl) : my_con(impl) {}

    virtual ~EnvControlAckHandler() {}

    virtual void handle_packet(const char* payload, uint32_t len)
    {
        if (len == sizeof(UhdpEnvControl))
        {
            const UhdpEnvControl& msg = *(const UhdpEnvControl*)payload;
            my_con.env_control_ack_received(msg.sequence_number);
        }
    }

    Connection_Impl& my_con;
};


struct PPAHandler : public ProtHandlerBase
{
    PPAHandler(Connection_Impl& impl) : my_con(impl) {}

    virtual ~PPAHandler() {}

    virtual void handle_packet(const char* payload, uint32_t len)
    {
        if (len < sizeof(uint32_t))
        {
            return;
        }

        uint32_t cpu_id = *reinterpret_cast<const uint32_t*>(payload);
        payload += sizeof(cpu_id);
        len -= sizeof(cpu_id);

        uint32_t count = len / sizeof(LogEvent);
        if (len != count * sizeof(LogEvent))
        {
            printf("PPA message has unaligned length %d != %u\n", len, (uint32_t)(count * sizeof(LogEvent)));
        }
        else if (!my_con.profile_agent)
        {
            //printf("Discarding %d PPA events, no profile agent\n", count);
        }
        else
        {
            my_con.profile_agent->received_ppa_events(cpu_id, reinterpret_cast<const LogEvent*>(payload), count);
        }
    }

    Connection_Impl& my_con;
};

class ControlMessage
{
public:

    ControlMessage(UhdpControlMessageType t, const char* msg, uint32_t len, uint16_t next_sequence)
        : next(NULL)
        , sequence(next_sequence)
    {
        UhdpHeader*     uhdp = (UhdpHeader*)msgbuf;
        UhdpEnvControl* ctrl = (UhdpEnvControl*)(uhdp + 1);

        uhdp->message_type = UHDP_TYPE_ENV_CONTROL_DATA;
        uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpEnvControl) + len;
        ctrl->message_type = (uint16_t)t;
        ctrl->sequence_number = sequence;
        memcpy(ctrl + 1, msg, len);
    }

    enum { MAX_SIZE = MAX_UDP_DATAGRAM - sizeof(UhdpHeader) - sizeof(UhdpEnvControl) };

    UhdpHeader* get_message() { return (UhdpHeader*)msgbuf; }

    char msgbuf[MAX_UDP_DATAGRAM];
    ControlMessage* next;
    uint16_t sequence;
};


void Connection_Impl::allocate_protocols()
{
    memset(uhdp_table, 0, sizeof(uhdp_table));

    LogMsgDiscHandler* msg_disc = new LogMsgDiscHandler();

    uhdp_table[UHDP_TYPE_PRINT]             = new UHPHandler();
    uhdp_table[UHDP_TYPE_PPA]               = new PPAHandler(*this);
    uhdp_table[UHDP_TYPE_LOG_AGGR]          = new LogAggregateHandler(log_agent, *logcontrol, *msg_disc);
    uhdp_table[UHDP_TYPE_LOG_PRODUCER_NAME] = dynamic_cast<ProtHandlerBase*>(logcontrol);
    uhdp_table[UHDP_TYPE_LOG_MESSAGE]       = msg_disc;
    uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]    = new DiagDiscHandler();
    uhdp_table[UHDP_TYPE_ANTENNA_CONFIG]    = new AntennaConfigDiscHandler();
    uhdp_table[UHDP_TYPE_LOG_TASK_NAME]     = new PPADiscHandler();
    uhdp_table[UHDP_TYPE_ENV_CONTROL_DATA]  = new EnvControlDataHandler(*this);
    uhdp_table[UHDP_TYPE_ENV_CONTROL_ACK]   = new EnvControlAckHandler(*this);

    for (uint32_t i = 0; i < 256; i++)
    {
        if (!uhdp_table[i])
        {
            uhdp_table[i] = new NopHandler(*this, i);
        }
    }

    if (!orignal_sigint_handler)
    {
        orignal_sigint_handler = signal(SIGINT, con_sigint_handler);
    }
    else
    {
        /* reset any previous cleanly handled ctrl+c event */
        ctrl_c_pressed = false;
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
    }

    delete vehicle;
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


uint32_t        Connection_Impl::get_num_antenna_configs() const
{
    AntennaConfigDiscHandler* ant = dynamic_cast<AntennaConfigDiscHandler*>(uhdp_table[UHDP_TYPE_ANTENNA_CONFIG]);
    return ant->get_num_antenna_configs();
}

const char*     Connection_Impl::get_antenna_config_name(uint32_t antcfg_id) const
{
    AntennaConfigDiscHandler* ant = dynamic_cast<AntennaConfigDiscHandler*>(uhdp_table[UHDP_TYPE_ANTENNA_CONFIG]);
    return ant->get_antenna_config_name(antcfg_id);
}

uint32_t        Connection_Impl::get_basic_antenna_config_id(BasicAntennaConfigEnum bac) const
{
    AntennaConfigDiscHandler* ant = dynamic_cast<AntennaConfigDiscHandler*>(uhdp_table[UHDP_TYPE_ANTENNA_CONFIG]);
    return ant->get_basic_antenna_config_id(bac);
}

void Connection_Impl::register_vehicle_CAN_device(VehicleCAN& v)
{
    delete vehicle;
    vehicle = &v;
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
        }
    }
    else
    {
        counters.wrong_payload_length++;
        printf("Dropping outgoing UHDP message of type %d (%s), message length 0\n", hdr->message_type, uhdp_msg_type_names[hdr->message_type]);
    }
}


void Connection_Impl::diag_force_stop()
{
    UhdpHeader hdr;
    hdr.message_type = UHDP_TYPE_DIAG_FORCE_STOP;
    hdr.total_length = sizeof(UhdpHeader);
    send_uhdp(&hdr);
}


int Connection_Impl::send_hello(uint32_t radar_ip, uint32_t timeout_sec, uint32_t uhdp_ver)
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

    //printf("Attempting to connect with uhdp_ver %d\n", uhdp_ver);
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
    UhdpHeader*       uhdp = (UhdpHeader*)msgbuf;
    UhdpHelloMessage* msg  = (UhdpHelloMessage*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_HELLO;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpHelloMessage);
    memset(msg, 0, sizeof(UhdpHelloMessage));
    msg->logger_ip_address = 0;
    msg->radar_ip_address  = ntohl(radar_ip);
    msg->logger_UDP_port   = ntohs(lcladdr.sin_port);

    //printf("Sending HELLO\n");
    send_uhdp(uhdp);

    timeval tv;

    // configure recvfrom() to block until a datagram or timeout
#if _WIN32
    int timeout_ms = timeout_sec * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
#endif

    struct sockaddr_in remaddr;
    int recvlen = recvfrom(sockfd, msgbuf, MAX_UDP_DATAGRAM, 0,
                           (struct sockaddr*)&remaddr, &addrlen);
    if (recvlen < 0)
    {
#if _WIN32
        if (WSAGetLastError() == WSAETIMEDOUT)
        {
            // timeout
            return - 1;
        }
#else
        if (errno == EWOULDBLOCK)
        {
            // timeout
            return -1;
        }
#endif
        else
        {
            perror("recvfrom() error: ");
            return -1;
        }
    }

    gettimeofday(&tv, NULL);

    counters.total_packets_rcvd++;

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
        uint64_t host_time = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                             (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;
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
            rear_axle_distance = geoms[0];
            centerline_distance = geoms[1];
            mount_height = geoms[2];
            mount_azimuth = geoms[3];
            mount_elevation = geoms[4];
        }
    }
    else if (UHDP_TYPE_PRINT == uhdp->message_type)
    {
        UHPHandler uhp;
        uhp.handle_packet((char*)(uhdp + 1), recvlen);
        return -2;
    }
    else
    {
        printf("Unexpected message type %d\n", uhdp->message_type);
        return -1;
    }

    if (radar_ip != remaddr.sin_addr.s_addr || htons(uhdp_port) != remaddr.sin_port)
    {
        char ipbuf[64];
        printf("Response came from unexpected IP %s or port %d\n",
                inet_ntop(AF_INET, &remaddr, ipbuf, sizeof(ipbuf)),
                ntohs(remaddr.sin_port));
        return -1;
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

    if (connection_uhdp_version >= 28)
    {
        send_uhdp(&ack.hdr);
    }

    // Process all discovery messages before returning
    for(;;)
    {
        int recvlen = recvfrom(sockfd, msgbuf, MAX_UDP_DATAGRAM, 0,
                               (struct sockaddr*)&remaddr, &addrlen);
        if (recvlen < 0)
        {
            if (ctrl_c_pressed)
            {
                return -1;
            }
#if _WIN32
            if (WSAGetLastError() == WSAETIMEDOUT)
            {
                // timeout
                continue;
            }
#else
            if (errno == EWOULDBLOCK)
            {
                // timeout
                continue;
            }
#endif
            else
            {
                perror("recvfrom() error: ");
                return -1;
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

        if (UHDP_TYPE_DISCOVERY == uhdp->message_type)
        {
            UhdpDiscovery* disc = (UhdpDiscovery*)(uhdp + 1);

            if (disc->discovery_state == 0)
            {
                // We received a second (or third) discovery start message. This
                // means the radar was slow and the previous discovery start was
                // a response to a previous HELLO request.
                printf("Ignoring duplicate DISCOVERY start message\n");

                // reset the discovery message count, and re-acknowledge the
                // HELLO
                ack.discovery_msg_count = 0;
                if (connection_uhdp_version >= 28)
                {
                    send_uhdp(&ack.hdr);
                }
                continue;
            }

#if _WIN32
            // configure recvfrom() to have no blocking
            ULONG NonBlock = 1;
            bool ok = true;
            if (ioctlsocket(sockfd, FIONBIO, &NonBlock) == SOCKET_ERROR)
            {
                printf("ioctlsocket() failed with error %d\n", WSAGetLastError());
                ok = false;
            }
            // configure large send/receive buffers (Windows defaults to 8K, W10 64K)
            int buffer_size = 128 * 1024;
            ok &= setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (const char*)&buffer_size, sizeof(buffer_size)) == 0;
            ok &= setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (const char*)&buffer_size, sizeof(buffer_size)) == 0;
            assert(ok);
#else
            if (use_thread)
            {
                // configure recvfrom() to block until a datagram or a short 100us timeout
                tv.tv_sec = 0;
                tv.tv_usec = 100;
                setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
            }
            else
            {
                tv.tv_sec = 0;
                tv.tv_usec = 1;
                setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
                fcntl(sockfd, O_NONBLOCK, 0);
            }
#endif
            connection_active = true;

            // in some circumstances, the radar might have more than one HELLO
            // queued up in its ethernet receive queue and it responds to each
            // of them in series. We want to ignore all of this extra discovery
            // info; it can only do harm to these classes
            uhdp_table[UHDP_TYPE_DISCOVERY]->disable();
            uhdp_table[UHDP_TYPE_LOG_PRODUCER_NAME]->disable();
            uhdp_table[UHDP_TYPE_LOG_MESSAGE]->disable();
            uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]->disable();
            uhdp_table[UHDP_TYPE_LOG_TASK_NAME]->disable();
            return 0;
        }
        else
        {
            if (connection_uhdp_version >= 28)
            {
                ack.discovery_msg_count++;
                send_uhdp(&ack.hdr);
            }

            //printf("received type %d (%s), message length %d\n", uhdp->message_type, uhdp_msg_type_names[uhdp->message_type], recvlen);
            uhdp_table[uhdp->message_type]->handle_packet((char*)(uhdp + 1), recvlen);
        }
    }

    return 0;
}

bool            Connection_Impl::send_control_message(UhdpControlMessageType t, const char* msg, uint32_t len)
{
    if (len > ControlMessage::MAX_SIZE)
        return false;

    ControlMessage* cm = new ControlMessage(t, msg, len, next_cm_sequence++);

    if (!pending_cm)
    {
        send_uhdp(cm->get_message());
        pending_cm = cm;
    }
    else if (last_cm)
    {
        last_cm->next = cm;
        last_cm = cm;
    }
    else
    {
        first_cm = last_cm = cm;
    }

    // TODO: handle retransmissions in the network thread

    return true;
}

void            Connection_Impl::env_control_ack_received(uint16_t sequence_number)
{
    if (pending_cm && pending_cm->sequence == sequence_number)
    {
        delete pending_cm;
        pending_cm = NULL;
    }

    if (first_cm && !pending_cm)
    {
        send_uhdp(first_cm->get_message());
        pending_cm = first_cm;
        first_cm = first_cm->next;
        if (!first_cm)
        {
            last_cm = NULL;
        }
    }
}

void            Connection_Impl::start_radar_profiling(UserProfileAgent& agent)
{
    if (profile_agent)
    {
        return;
    }

    profile_agent = &agent;

    PPADiscHandler* ppa_disc = dynamic_cast<PPADiscHandler*>(uhdp_table[UHDP_TYPE_LOG_TASK_NAME]);

    agent.set_num_producers(logcontrol->num_producers);

    for (uint32_t prodid = 0; prodid < logcontrol->num_producers; ++prodid)
    {
        auto producer = logcontrol->producers[prodid];
        agent.set_producer(prodid, producer.prod_name, producer.num_subunits);

        for (uint32_t subid = 0; subid < producer.num_subunits; subid++)
        {
            agent.set_producer_subunit(prodid, subid, logcontrol->get_subunit_name(prodid, subid));
        }
    }

    agent.set_task_names(ppa_disc->num_task_types, ppa_disc->task_name);

    run_diag_by_name("control", "enable-ppa", NULL, 0, NULL, 0);
}

void            Connection_Impl::stop_radar_profiling()
{
    if (profile_agent)
    {
        profile_agent = NULL;

        if (connection_active)
        {
            run_diag_by_name("control", "disable-ppa", NULL, 0, NULL, 0);
        }
    }
}

uint32_t        Connection_Impl::get_num_diags() const
{
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);
    return dd->get_num_diags();
}

const char*     Connection_Impl::get_diag_name(uint32_t d) const
{
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);
    return dd->get_diag_name(d);
}

uint32_t        Connection_Impl::get_num_tests_in_diag(uint32_t d) const
{
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);
    return dd->get_num_tests_in_diag(d);
}

const char*     Connection_Impl::get_diag_test_name(uint32_t d, uint32_t test_id) const
{
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);
    return dd->get_diag_test_name(d, test_id);
}

uint32_t        Connection_Impl::run_diag_by_id(uint32_t diag_id,
                                                uint32_t test_id,
                                                const void* inputs,
                                                size_t input_bytesize,
                                                void* outputs,
                                                size_t output_bytesize)
{
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);
    if (diag_id >= dd->get_num_diags())
    {
        last_err = CON_DIAG_ID_OUT_OF_RANGE;
        return 0;
    }
    if (test_id >= dd->get_num_tests_in_diag(diag_id))
    {
        last_err = CON_DIAG_TEST_ID_OUT_OF_RANGE;
        return 0;
    }
    if (diag_last_resend)
    {
        last_err = CON_DIAG_ALREADY_RUNNING;
        return 0;
    }
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    UhdpHeader*      uhdp = (UhdpHeader*)diag_msgbuf;
    UhdpDiagRequest* msg  = (UhdpDiagRequest*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_DIAG_REQUEST;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpDiagRequest) + input_bytesize;

    if (uhdp->total_length >= sizeof(diag_msgbuf))
    {
        last_err = CON_DIAG_INPUT_TOO_LARGE;
        return 0;
    }

    msg->diag_sequence_number = diag_sequence_number;
    msg->class_id = diag_id;
    msg->test_id = test_id;
    msg->reserved = 0;

    if (inputs && input_bytesize > 0)
    {
        memcpy(msg + 1, inputs, input_bytesize);
    }

    send_uhdp(uhdp);

    resend_count = 0;
    diag_is_hung = false;

    timeval tv;
    gettimeofday(&tv, NULL);
    diag_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                       (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        diag_comp_event.wait();
    }
    else
    {
        while (connection_active && diag_last_resend)
        {
            poll_socket();
        }
    }

    if (diag_is_hung)
    {
        diag_is_hung = false;

        (void)query_radar_status_bitmask();

        if (last_err == CON_SOCKET_FAILURE)
        {
            printf("%s%s%s", stars, radar_hang_message, stars);
            return 0;
        }
        else
        {
            printf("%s", stars);
            printf(diag_hang_message, get_diag_name(diag_id), get_diag_test_name(diag_id, test_id));
            printf("%s", stars);
            last_err = CON_DIAG_HANG;
            return 0;
        }
    }

    if (!connection_active || diag_last_resend)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    // diag_msgbuf now contains the diag response
    UhdpDiagComplete* comp = (UhdpDiagComplete*)(uhdp + 1);
    size_t output_size = uhdp->total_length - sizeof(UhdpHeader) - sizeof(UhdpDiagComplete);
    if ((output_size > output_bytesize) || (output_size && !outputs))
    {
        last_err = CON_DIAG_OUTPUT_TOO_LARGE;
    }
    else if (comp->diag_sequence_number != diag_sequence_number)
    {
        last_err = CON_DIAG_ALREADY_RUNNING;
    }
    else
    {
        last_diag_return_code = (DiagReturnCodeEnum)comp->return_code;
        last_diag_error_num   = comp->error_num;

        if (last_diag_return_code == DIAG_RET_SUCCESS)
        {
            last_err = CON_NO_ERROR;
        }
        else
        {
            last_err = CON_DIAG_FAILURE;
        }

        memcpy(outputs, comp + 1, output_size);
    }

    diag_sequence_number++;

    return output_size;
}


uint32_t        Connection_Impl::peek_register(uint32_t reg_addr, uint32_t read_mask)
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    UhdpHeader*  uhdp = (UhdpHeader*)peek_buffer;
    UhdpPeekReq* msg  = (UhdpPeekReq*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_STATE_PEEK;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpPeekReq);
    msg->sequence_id   = peek_sequence_number;
    msg->type = 1; // register peek mode
    msg->read_dev_ptr = reg_addr;
    msg->reg_mask = read_mask;

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    peek_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                       (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(uhdp);

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        peek_comp_event.wait();
    }
    else
    {
        while (connection_active && peek_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    peek_sequence_number++;

    UhdpPeekResp* resp = (UhdpPeekResp*)(uhdp + 1);
    if (resp->type != 1 || resp->status || resp->read_dev_ptr != reg_addr)
    {
        last_err = CON_PEEK_INVALID_ADDRESS;
        return 0;
    }

    last_err = CON_NO_ERROR;
    return *(uint32_t*)(resp + 1);
}


uint32_t        Connection_Impl::query_radar_status_bitmask()
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    UhdpHeader*  uhdp = (UhdpHeader*)status_buffer;
    uhdp->message_type = UHDP_TYPE_STATE_QUERY;
    uhdp->total_length = sizeof(UhdpHeader);

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    status_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                         (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(uhdp);

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        peek_comp_event.wait();
    }
    else
    {
        while (connection_active && status_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    uint32_t* resp = (uint32_t*)(uhdp + 1);
    last_err = CON_NO_ERROR;

    if (sabine_type == ST_UNKNOWN)
    {
        sabine_type = (*resp & STATE_ATTR_SABINE_B) ? ST_SABINE_B : ST_SABINE_A;
    }

    return *resp;
}

bool            Connection_Impl::is_sabine_b()
{
    if (sabine_type == ST_UNKNOWN)
    {
        (void)query_radar_status_bitmask();
    }

    return sabine_type == ST_SABINE_B;
}


bool            Connection_Impl::peek_memory(uint32_t dev_addr, uint32_t byte_count, char* output)
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return false;
    }

    UhdpHeader*  uhdp = (UhdpHeader*)peek_buffer;
    UhdpPeekReq* msg  = (UhdpPeekReq*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_STATE_PEEK;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpPeekReq);
    msg->sequence_id   = peek_sequence_number;
    msg->type = 0; // memory peek mode
    msg->read_dev_ptr = dev_addr;
    msg->byte_count = byte_count;

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    peek_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                       (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(uhdp);

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        peek_comp_event.wait();
    }
    else
    {
        while (connection_active && peek_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    peek_sequence_number++;

    UhdpPeekResp* resp = (UhdpPeekResp*)(uhdp + 1);
    if (resp->type != 0 || resp->status == 2 || resp->read_dev_ptr != dev_addr)
    {
        last_err = CON_PEEK_INVALID_ADDRESS;
        return false;
    }
    if (resp->status == 1)
    {
        last_err = CON_PEEK_INVALID_SIZE;
        return false;
    }

    last_err = CON_NO_ERROR;
    memcpy(output, resp + 1, byte_count);
    return true;
}


bool            Connection_Impl::poke_memory(uint32_t dev_addr, uint32_t bytecount, const char* input)
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return false;
    }

    UhdpHeader*  uhdp = (UhdpHeader*)peek_buffer;
    UhdpPokeReq* msg = (UhdpPokeReq*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_STATE_POKE;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpPeekReq);
    msg->sequence_id   = peek_sequence_number;
    msg->type = 0; // memory poke mode
    msg->write_dev_ptr = dev_addr;

    char* payload = (char*)(msg + 1);
    if (bytecount > sizeof(peek_buffer) - uhdp->total_length)
    {
        last_err = CON_POKE_INVALID_SIZE;
        return false;
    }

    memcpy(payload, input, bytecount);
    uhdp->total_length += bytecount;

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    peek_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                       (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(uhdp);

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        peek_comp_event.wait();
    }
    else
    {
        while (connection_active && peek_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }

    peek_sequence_number++;

    UhdpPokeResp* resp = (UhdpPokeResp*)(uhdp + 1);
    if (resp->type != 0 || resp->status)
    {
        last_err = CON_PEEK_INVALID_ADDRESS;
        return false;
    }

    last_err = CON_NO_ERROR;
    return true;
}


bool            Connection_Impl::poke_register(uint32_t reg_addr, uint32_t write_mask, uint32_t value)
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return false;
    }

    UhdpHeader*  uhdp = (UhdpHeader*)peek_buffer;
    UhdpPokeReq* msg  = (UhdpPokeReq*)(uhdp + 1);
    RegisterInfo* info = (RegisterInfo*)(msg + 1);

    uhdp->message_type = UHDP_TYPE_STATE_POKE;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpPokeReq) + sizeof(RegisterInfo);
    msg->sequence_id   = peek_sequence_number;
    msg->type = 1; // register poke mode
    msg->write_dev_ptr = reg_addr;
    info->mask = write_mask;
    info->value = value;

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    peek_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                       (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(uhdp);

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        peek_comp_event.wait();
    }
    else
    {
        while (connection_active && peek_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return false;
    }

    peek_sequence_number++;

    UhdpPokeResp* resp = (UhdpPokeResp*)(uhdp + 1);
    if (resp->type != 1 || resp->status)
    {
        last_err = CON_POKE_INVALID_ADDRESS;
        return false;
    }

    last_err = CON_NO_ERROR;
    return true;
}


int32_t         Connection_Impl::get_last_diag_error(DiagReturnCodeEnum& code) const
{
    code = last_diag_return_code;
    return last_diag_error_num;
}


uint32_t        Connection_Impl::run_diag_by_name(const char* diag_name,
                                                  const char* test_name,
                                                  const void* inputs,
                                                  size_t input_bytesize,
                                                  void* outputs,
                                                  size_t output_bytesize)
{
    uint32_t diag_id, test_id;
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);

    last_err = CON_NO_ERROR;
    for (diag_id = 0; diag_id < dd->get_num_diags(); diag_id++)
    {
        if (!strcmp(diag_name, dd->get_diag_name(diag_id)))
        {
            for (test_id = 0; test_id < dd->get_num_tests_in_diag(diag_id); test_id++)
            {
                if (!strcmp(test_name, dd->get_diag_test_name(diag_id, test_id)))
                {
                    return run_diag_by_id(diag_id, test_id, inputs, input_bytesize, outputs, output_bytesize);
                }
            }

            last_err = CON_DIAG_UNKNOWN_TEST_NAME;
            return 0;
        }
    }

    last_err = CON_DIAG_UNKNOWN_NAME;
    return 0;
}

uint8_t         Connection_Impl::get_diag_test_api_version(const char* diag_name, const char* test_name) const
{
    DiagDiscHandler* dd = dynamic_cast<DiagDiscHandler*>(uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]);
    last_err = CON_NO_ERROR;
    for (uint32_t diag_id = 0; diag_id < dd->get_num_diags(); diag_id++)
    {
        if (!strcmp(diag_name, dd->get_diag_name(diag_id)))
        {
            for (uint32_t test_id = 0; test_id < dd->get_num_tests_in_diag(diag_id); test_id++)
            {
                if (!strcmp(test_name, dd->get_diag_test_name(diag_id, test_id)))
                {
                    return dd->get_diag_test_api_version(diag_id, test_id);
                }
            }

            last_err = CON_DIAG_UNKNOWN_TEST_NAME;
            return 0xFF;
        }
    }

    last_err = CON_DIAG_UNKNOWN_NAME;
    return 0xFF;
}

void            Connection_Impl::configure_radar_telemetry(const UhdpTelemetryData& tel)
{
    char buf[sizeof(UhdpHeader) + sizeof(UhdpTelemetryData)];
    UhdpHeader* hdr = (UhdpHeader*)buf;

    hdr->message_type = UHDP_TYPE_TELEMETRY_DATA;
    hdr->total_length = sizeof(buf);
    memcpy(hdr + 1, &tel, sizeof(UhdpTelemetryData));

    send_uhdp(hdr);
}


bool            Connection_Impl::configure_data_capture(const UhdpCaptureControl& cap_ctrl)
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return false;
    }

    StaticAssert<sizeof(UhdpHeader) + sizeof(UhdpCaptureControl) < sizeof(control_msg_buffer)>::istrue();
    UhdpHeader* hdr = (UhdpHeader*)control_msg_buffer;
    UhdpCaptureControl* cap = (UhdpCaptureControl*)(hdr + 1);

    hdr->message_type = UHDP_TYPE_CAPTURE_CONTROL;
    hdr->total_length = sizeof(UhdpHeader) + sizeof(UhdpCaptureControl);
    memcpy(hdr + 1, &cap_ctrl, sizeof(UhdpCaptureControl));

    cap->command_sequence_number = command_sequence_number;
    if (cap->enable_mask & DL_POINT_CLOUD) cap->enable_mask |= DL_CI;

    resend_count = 0;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    control_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                          (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(hdr);

    if (use_thread)
    {
        control_ack_event.wait();
    }
    else
    {
        while (connection_active && control_last_resend)
        {
            poll_socket();
        }
    }

    // not currently looking at the ACK for capture control

    command_sequence_number++;

    return true;
}


int            Connection_Impl::send_scan_control(const RDC_ScanControl& scan_ctrl, const RDC_ScanDescriptor* desc)
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return 0;
    }
    if (scan_ctrl.scan_loop_count > uint32_t(RDC_MAX_SCAN_LOOP))
    {
        last_err = CON_INPUT_OUT_OF_RANGE;
        return 0;
    }

    UhdpHeader* hdr = (UhdpHeader*)control_msg_buffer;
    uint32_t* cmd_seq_ptr = (uint32_t*)(hdr + 1);
    RDC_ScanControl* sc = (RDC_ScanControl*)(cmd_seq_ptr + 1);

    hdr->message_type = UHDP_TYPE_SCAN_CONTROL;
    hdr->total_length = sizeof(UhdpHeader) + sizeof(uint32_t) + sizeof(RDC_ScanControl);
    hdr->total_length += sizeof(RDC_ScanDescriptor) * scan_ctrl.scan_loop_count;

    if (hdr->total_length > sizeof(control_msg_buffer))
    {
        last_err = CON_INPUT_OUT_OF_RANGE;
        return 0;
    }

    *cmd_seq_ptr = command_sequence_number;

    memcpy(sc, &scan_ctrl, sizeof(RDC_ScanControl));

    if (scan_ctrl.scan_loop_count != 0U)
    {
        memcpy(sc + 1, desc, sizeof(RDC_ScanDescriptor) * scan_ctrl.scan_loop_count);
    }

    resend_count = 0;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    control_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                          (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    send_uhdp(hdr);

    if (use_thread)
    {
        control_ack_event.wait();
    }
    else
    {
        while (connection_active && control_last_resend)
        {
            poll_socket();
        }
    }

    uint32_t* payload = (uint32_t*)(hdr + 1);

    if (payload[0] == command_sequence_number)
    {
        command_sequence_number++;

        if (hdr->total_length >= sizeof(UhdpHeader) + sizeof(uint32_t) * 2)
        {
            // radar returned a count of scans enqueued
            return (int32_t)payload[1];
        }
    }
    else
    {
        printf("Control ACK command sequence number mismatch\n");
        // user could call this function again
        return 0;
    }

    return 0;
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


void            Connection_Impl::configure_detection_thresholds(const RDC_ThresholdControl* thresh_ctrl, uint32_t count)
{
    char buf[sizeof(UhdpHeader) + sizeof(RDC_ThresholdControl) * RDC_MAX_SCAN_LOOP];
    UhdpHeader* hdr = (UhdpHeader*)buf;

    if (count == 0 || count > RDC_MAX_SCAN_LOOP)
    {
        last_err = CON_INPUT_OUT_OF_RANGE;
    }
    else
    {
        hdr->message_type = UHDP_TYPE_THRESHOLD;
        hdr->total_length = sizeof(UhdpHeader) + (sizeof(RDC_ThresholdControl) * count);
        memcpy(hdr + 1, thresh_ctrl, sizeof(RDC_ThresholdControl) * count);

        send_uhdp(hdr);

        // the radar will not respond

#if POINT_CLOUD_HOST_CALCULATION
        PointCloud_Impl::configure_thresholds(*thresh_ctrl);
#endif
    }
}


void            Connection_Impl::request_MUSIC(RDC_music_request* regions, uint32_t count)
{
    uint32_t total_size = sizeof(UhdpHeader) + sizeof(RDC_music_request) * count;
    if (total_size > MAX_UDP_DATAGRAM)
    {
        last_err = CON_INPUT_OUT_OF_RANGE;
        return;
    }

    char *buf = new char[total_size];
    UhdpHeader* hdr = (UhdpHeader*)buf;

    hdr->message_type = UHDP_TYPE_MUSIC_CONTROL;
    hdr->total_length = total_size;
    if (count)
    {
        memcpy(hdr + 1, regions, sizeof(RDC_music_request) * count);
    }

    send_uhdp(hdr);

    delete [] buf;
}


void            Connection_Impl::register_dprobe_request(
        const RDC_DProbe_corr_config& corr,
        const RDC_DProbe_rms_config& rms,
        const RDC_DProbe_iq_config& iq,
        uint8_t rx_select)
{
    char buf[sizeof(UhdpHeader) + sizeof(RDC_DProbe_corr_config) + sizeof(RDC_DProbe_rms_config) + sizeof(RDC_DProbe_iq_config) + sizeof(uint8_t)];
    UhdpHeader* hdr = (UhdpHeader*)buf;
    hdr->message_type = UHDP_TYPE_DPROBE_CONFIG;
    hdr->total_length = sizeof(buf);

    RDC_DProbe_corr_config* corrptr = (RDC_DProbe_corr_config*)(hdr + 1);
    RDC_DProbe_rms_config* rmsptr   = (RDC_DProbe_rms_config*)(corrptr + 1);
    RDC_DProbe_iq_config* iqptr     = (RDC_DProbe_iq_config*)(rmsptr + 1);
    uint8_t* select_rx              = (uint8_t*) (iqptr + 1);

    *corrptr = corr;
    *rmsptr = rms;
    *iqptr = iq;
    *select_rx = rx_select;

    send_uhdp(hdr);
}

void            Connection_Impl::clear_dprobe_requests()
{
    UhdpHeader hdr;
    hdr.message_type = UHDP_TYPE_DPROBE_CONFIG;
    hdr.total_length = sizeof(hdr);
    send_uhdp(&hdr);
}

void            Connection_Impl::register_dcmeasurement_requests(
        const RDC_DCMeasure_config& dc_measure)
{
    char buf[sizeof(UhdpHeader) + sizeof(RDC_DCMeasure_config)];
    UhdpHeader* hdr = (UhdpHeader*)buf;
    hdr->message_type = UHDP_TYPE_DC_MEAS_CONFIG;
    hdr->total_length = sizeof(buf);

    RDC_DCMeasure_config* dcmeasptr = (RDC_DCMeasure_config*)(hdr + 1);

    *dcmeasptr = dc_measure;

    send_uhdp(hdr);
}


const char*     Connection_Impl::get_requestable_hardware_unit_names()
{
    if (hw_unit_names)
    {
        return hw_unit_names;
    }

    UhdpHeader*             uhdp = (UhdpHeader*)lld_core_dump_msg_buffer;
    UhdpLLDCoreDumpRequest* msg  = (UhdpLLDCoreDumpRequest*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_LLD_CORE_DUMP;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpLLDCoreDumpRequest);

    memset(msg->hardware_unit_name, 0, sizeof(msg->hardware_unit_name));
    strcpy(msg->hardware_unit_name, "?");

    send_uhdp(uhdp);

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    core_dump_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                            (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        core_dump_ack_event.wait();
    }
    else
    {
        while (connection_active && core_dump_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active || core_dump_last_resend)
    {
        last_err = CON_SOCKET_FAILURE;
        return NULL;
    }

    return hw_unit_names;
}


const uint32_t* Connection_Impl::get_hardware_unit_core_dump(const char* hw_unit_name, INT preload, uint32_t& num_addr_value_tuples)
{
    UhdpHeader*             uhdp = (UhdpHeader*)lld_core_dump_msg_buffer;
    UhdpLLDCoreDumpRequest* msg  = (UhdpLLDCoreDumpRequest*)(uhdp + 1);

    uhdp->message_type = UHDP_TYPE_LLD_CORE_DUMP;
    uhdp->total_length = sizeof(UhdpHeader) + sizeof(UhdpLLDCoreDumpRequest);

    memset(msg->hardware_unit_name, 0, sizeof(msg->hardware_unit_name));
    strncpy(msg->hardware_unit_name, hw_unit_name, sizeof(msg->hardware_unit_name)-1);
    msg->preload = preload;

    send_uhdp(uhdp);

    resend_count = 0;

    timeval tv;
    gettimeofday(&tv, NULL);
    core_dump_last_resend = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                            (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    if (use_thread)
    {
        // blocking wait for network thread to unlock
        core_dump_ack_event.wait();
    }
    else
    {
        while (connection_active && core_dump_last_resend)
        {
            poll_socket();
        }
    }

    if (!connection_active || core_dump_last_resend)
    {
        last_err = CON_SOCKET_FAILURE;
        num_addr_value_tuples = 0U;
        return NULL;
    }

    num_addr_value_tuples = lld_core_dump_receive_count;
    return lld_core_dump_buffer;
}


Scanning*       Connection_Impl::setup_scanning(const RDC_ScanControl& scanctrl, const RDC_ScanDescriptor* desc, const UhdpCaptureControl& cc)
{
    if (myscanning)
    {
        last_err = CON_ALREADY_SCANNING;
        return NULL;
    }

    UhdpCaptureControl capctrl;
    capctrl = cc;
    capctrl.capture_mode = DL_NORMAL;
    if (cc.enable_mask & DL_POINT_CLOUD) capctrl.enable_mask |= DL_CI;
    configure_data_capture(capctrl); // blocking wait for ACK

    RDC_ScanControl ctrl;
    ctrl = scanctrl;
    ctrl.scan_count = -1;

    if (send_scan_control(ctrl, desc) != 0) // blocking wait for ACK
    {
        myscanning = new Scanning_Impl(this);
        return myscanning;
    }
    else
    {
        return NULL;
    }
}


void            Connection_Impl::release_scanning(Scanning_Impl& s)
{
    UhdpCaptureControl capctrl;
    capctrl.set_defaults();
    capctrl.capture_mode = DL_IDLE;
    configure_data_capture(capctrl);

    RDC_ScanControl scanctrl;
    scanctrl.defaults();
    scanctrl.scan_count = 0;
    scanctrl.scan_loop_count = 0;
    send_scan_control(scanctrl, NULL);

    assert(myscanning == &s);
    delete &s;
    myscanning = NULL;
}


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
        int nfds = 1;

        if (vehicle)
        {
            FD_SET(vehicle->get_file_handle(), &read_set);
            nfds++;
        }

        // wait for a message, or 1ms
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        select(nfds + 1, &read_set, &write_set, NULL, &tv);

        poll_socket();
    }
}

void Connection_Impl::poll_socket()
{
    poll_mutex.acquire();

    poll_socket_internal();

    if (vehicle && vehicle->poll())
    {
        UhdpTelemetryData tel;
        memset(&tel, 0, sizeof(tel));
        float temp_y = deg2rad(vehicle->yaw_rate) * vehicle->axle_dist;
        float temp_x = sqrtf(fabs((vehicle->vel_kmph_x * vehicle->vel_kmph_x) -
                                  (vehicle->vel_kmph_y * vehicle->vel_kmph_y)));
        tel.ego_velocity.x      = -temp_x / 3.6f;
        tel.ego_velocity.y      = -temp_y / 3.6f;
        tel.ego_velocity.z      = -vehicle->vel_kmph_z / 3.6f;
        tel.ego_acceleration.x  = -vehicle->acc_mpsps_x;
        tel.ego_acceleration.y  = -vehicle->acc_mpsps_y;
        tel.ego_acceleration.z  = -vehicle->acc_mpsps_z;
        tel.yaw_rate            = vehicle->yaw_rate;
        tel.braking_pressure    = vehicle->brake_pressure;
        tel.steering_turn_angle = vehicle->steering_angle;

        timeval cur_time;
        gettimeofday(&cur_time, NULL);

        uint64_t age_us = (cur_time.tv_sec - vehicle->update_time.tv_sec) * 1000 * 1000;
        if (cur_time.tv_usec >= vehicle->update_time.tv_usec)
            age_us += cur_time.tv_usec - vehicle->update_time.tv_usec;
        else
            age_us -= vehicle->update_time.tv_usec - cur_time.tv_usec;

        tel.telemetry_age_us = age_us;
        configure_radar_telemetry(tel);
    }

    poll_mutex.release();
}

void Connection_Impl::queue_completed_scan(ScanObject_Impl* scan)
{
    // this scan has completed transmission, or is never going to receive scan data
    uint32_t expected = scanlist_cond.get();
    uint32_t seq = scan->scan_info.scan_sequence_number;

    scan->next_scan_object = NULL;

    scanlist_mutex.acquire();
    if (last_out_scan)
    {
        last_out_scan->next_scan_object = scan;
    }
    else
    {
        first_out_scan = scan;
    }
    last_out_scan = scan;
    scanlist_mutex.release();

    scanlist_cond.set(seq + 1);
    counters.missing_scans += seq - expected;
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
}


void Connection_Impl::poll_socket_internal()
{
    // How many packets do we give the radar at a time?
    struct sockaddr_in remaddr;
    socklen_t addrlen = sizeof(remaddr);

    // these pointers never change, so we can define them outside the loop
    char msgbuf[MAX_UDP_DATAGRAM];
    const UhdpHeader* uhdp = (const UhdpHeader*)msgbuf;
    const char*       payload = (const char*)(uhdp + 1);
    const UhdpDataHeader* win = (const UhdpDataHeader*)payload;

    timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t cur_timestamp = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                             (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;

    // check for timeouts
    if (diag_last_resend)
    {
        uint64_t timeout = (3 * UH_CLOCK_TICS_PER_SECOND / 2) * timeout_scale * (resend_count + 1);
        if (ctrl_c_pressed && resend_count > 0)
        {
            abort_connection();
            return;
        }
        else if (cur_timestamp - diag_last_resend > timeout)
        {
            if (++resend_count < MAX_RESEND_COUNT)
            {
                send_uhdp((UhdpHeader*)diag_msgbuf);
                diag_last_resend = cur_timestamp;
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
        uint64_t timeout = (3 * UH_CLOCK_TICS_PER_SECOND / 2) * timeout_scale * (resend_count + 1);
        if (ctrl_c_pressed && resend_count > 0)
        {
            abort_connection();
            return;
        }
        else if (cur_timestamp - core_dump_last_resend > timeout)
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
        uint64_t timeout = (3 * UH_CLOCK_TICS_PER_SECOND / 2) * timeout_scale * (resend_count + 1);
        if (ctrl_c_pressed && resend_count > 0)
        {
            abort_connection();
            return;
        }
        else if (cur_timestamp - peek_last_resend > timeout)
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
        uint64_t timeout = UH_CLOCK_TICS_PER_SECOND * timeout_scale;
        if (ctrl_c_pressed && resend_count > 0)
        {
            abort_connection();
            return;
        }
        else if (cur_timestamp - status_last_resend > timeout)
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
        uint64_t timeout = (UH_CLOCK_TICS_PER_SECOND * 2) * timeout_scale * (resend_count + 1);
        if (ctrl_c_pressed && resend_count > 0)
        {
            abort_connection();
            return;
        }
        else if (cur_timestamp - control_last_resend > timeout)
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
        // stall you see in SCC when trying to close a dead connection
        if (ctrl_c_pressed && resend_count > 0)
        {
            abort_connection();
            return;
        }
        else if (cur_timestamp - request_close_time > UH_CLOCK_TICS_PER_SECOND * 2 * timeout_scale)
        {
            printf("Radar is not responding to connection close requests, aborting connection\n");
            abort_connection();
            return;
        }
    }

    // process received datagrams until it returns empty
    for (;;)
    {
        int recvlen = recvfrom(sockfd, msgbuf, MAX_UDP_DATAGRAM, 0,
                               (struct sockaddr*)&remaddr, &addrlen);
        if (recvlen < 0)
        {
#if _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK || err == WSAETIMEDOUT)
            {
                // timeout
                return;
            }
#else
            if (errno == EWOULDBLOCK)
            {
                // timeout
                return;
            }
#endif
            else
            {
                perror("recvfrom() error: ");
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

            gettimeofday(&tv, NULL);
            uint64_t cur_timestamp = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                                     (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;
            int64_t new_delta = cur_timestamp - ta.timestamp64;

            // if this delta is smaller than the previously known delta, assume
            // that we had slightly less latency in getting this packet and
            // record the new delta. otherwise, take the new delta every 100
            // scans in case there is slow drift.
            if (new_delta < training_radar_host_delta ||
                (scanlist_cond.get() % 100) == 0)
            {
                training_radar_host_delta = new_delta;
            }
            else if (scan_interval_base_host_time.tv_sec &&
                    ((scanlist_cond.get() % 100) == 90))
            {
                radar_host_delta = training_radar_host_delta;
                //printf("delta: %lld\n", radar_host_delta);
                send_scan_interval_base_time();
            }

            last_time_align = ta;
            continue;
        }
        else if (uhdp->message_type == UHDP_TYPE_SCAN_INFORMATION)
        {
            const UhdpScanInformation& msg = *reinterpret_cast<const UhdpScanInformation *>(payload);

            // UhdpScanInformation.current_time - radar timestamp when message was sent
            // UhdpScanInformation.scan_timestamp - radar timestamp when this scan ended
            // both timestamps are in units of 1 microsecond.
            //
            // derive microseconds since scan end timestamp by subtracting them
            timeval host_local_time;
            int32_t usec_since_scan_end;
            if (msg.current_time < msg.scan_timestamp)
            {
                if (is_sabine_b())
                {
                    // Wrap detected. The 32bit clock counter wraps around at (2^36) / 200 =
                    // 343597383 = 0x147AE147. Since the counter wraps early, we cannot
                    // rely on integer subtraction giving the correct answer
                    usec_since_scan_end = msg.current_time + (0x147AE147 - msg.scan_timestamp);
                }
                else
                {
                    // Wrap detected. The 32bit clock counter wraps around at (2^36) / 125 =
                    // 549755813.888 ~= 0x20C49BA6. Since the counter wraps early, we cannot
                    // rely on integer subtraction giving the correct answer
                    usec_since_scan_end = msg.current_time + (0x20C49BA6 - msg.scan_timestamp);
                }
            }
            else
            {
                usec_since_scan_end = msg.current_time - msg.scan_timestamp;
            }

            if (last_time_align.scan_sequence_number == msg.scan_sequence_number)
            {
                // leverage the time alignment message to map the scan end
                // directly into host time

                uint64_t radar_scan_end = last_time_align.timestamp64 - usec_since_scan_end;
                uint64_t host_scan_end = radar_scan_end + radar_host_delta;
                host_local_time.tv_sec = host_scan_end / (1000 * 1000);
                host_local_time.tv_usec = host_scan_end - host_local_time.tv_sec * (1000 * 1000);
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
                    host_local_time.tv_usec = 1000 * 1000 - usec_since_scan_end;
                }
                else
                {
                    host_local_time.tv_usec -= usec_since_scan_end;
                }
            }

            while (host_local_time.tv_usec > 1000 * 1000)
            {
                host_local_time.tv_sec++;
                host_local_time.tv_usec -= 1000 * 1000;
            }

            ScanObject_Impl* obj = new ScanObject_Impl();
            obj->handle_scan_info(&msg, recvlen, host_local_time, connection_uhdp_version);
            obj->srs_version_str = strdup(srs_version_str);
            obj->module_name = strdup(module_name);
            obj->module_type_name = strdup(module_type_name);
            obj->motherboard_type_name = strdup(motherboard_type_name);
            obj->antennaboard_type_name = strdup(antennaboard_type_name);
            obj->antenna_module_type_name = strdup(antenna_module_type_name);
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
                    printf("SRA: Ignoring diag complete response for seq %X, expecting %X\n",
                           comp->diag_sequence_number, diag_sequence_number);
                }
            }
            else
            {
                printf("SRA: Ignoring redundant diag complete response\n");
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
                    gettimeofday(&tv, NULL);
                    uint64_t cur_timestamp = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                                             (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;
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
}


void            Connection_Impl::synchronize_scan_interval(timeval host_time)
{
    scan_interval_base_host_time = host_time;

    send_scan_interval_base_time();
}


void            Connection_Impl::send_scan_interval_base_time()
{
    char buf[sizeof(UhdpHeader) + sizeof(UhdpTimeAlignment)];
    UhdpHeader* hdr = (UhdpHeader*)buf;
    hdr->message_type = UHDP_TYPE_TIME_ALIGN;
    hdr->total_length = sizeof(buf);

    // map host time to radar time
    uint64_t t = (uint64_t)UH_CLOCK_TICS_PER_SECOND * scan_interval_base_host_time.tv_sec +
                 (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * scan_interval_base_host_time.tv_usec;
    UhdpTimeAlignment* atime = (UhdpTimeAlignment*)(hdr + 1);
    atime->timestamp64 = t - radar_host_delta;
    atime->timestamp32 = 0;
    atime->scan_sequence_number = -1;

    send_uhdp(hdr);
}


ScanObject_Impl* Connection_Impl::scan_wait()
{
    if (!connection_active)
    {
        last_err = CON_SOCKET_FAILURE;
        return NULL;
    }

    uint32_t cache_next_scan_seq = scanlist_cond.get();

    // wait for the next scan to complete, or for connection to close
    for (;;)
    {
        ScanObject* scan = Connection_Impl::poll_completed_scan();
        if (scan)
        {
            return dynamic_cast<ScanObject_Impl*>(scan);
        }
        else if (!connection_active)
        {
            last_err = CON_SOCKET_FAILURE;
            return NULL;
        }
        else if (use_thread)
        {
            // safe blocking wait for notification from network thread
            cache_next_scan_seq = scanlist_cond.waitForChange(cache_next_scan_seq);
        }
        else
        {
            poll_socket();
        }
    }
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
        printf("Cannot create socket, err %d\n", sockfd);
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
        printf("Unable to bind local socket\n");
        return -1;
    }

    return 0;
}

void Connection_Impl::close_and_release()
{
    released = true;

    if (myscanning)
    {
        release_scanning(*myscanning);
    }

    if (connection_active)
    {
        // request the connection to be closed
        UhdpHeader hdr;
        hdr.message_type = UHDP_TYPE_CONN_CLOSE;
        hdr.total_length = sizeof(UhdpHeader);
        send_uhdp(&hdr);

        struct timeval tv;
        gettimeofday(&tv, NULL);
        request_close_time = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                             (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;
    }

    if (use_thread)
    {
        // main loop waits for CLOSE_ACK, then exits. blocking wait
        stop();
    }

    delete this;
}

void Connection_Impl::close_and_reboot()
{
    released = true;

    if (myscanning)
    {
        release_scanning(*myscanning);
    }

    if (connection_active)
    {
        run_diag_by_name("sabine", "sabine_reset", NULL, 0, NULL, 0);

        // request the connection to be closed
        UhdpHeader hdr;
        hdr.message_type = UHDP_TYPE_CONN_CLOSE;
        hdr.total_length = sizeof(UhdpHeader);
        send_uhdp(&hdr);

        struct timeval tv;
        gettimeofday(&tv, NULL);
        request_close_time = (uint64_t)UH_CLOCK_TICS_PER_SECOND * tv.tv_sec +
                             (uint64_t)UH_CLOCK_TICS_PER_MICRO_SECONDS * tv.tv_usec;
    }

    if (use_thread)
    {
        // main loop waits for CLOSE_ACK, then exits. blocking wait
        stop();
    }

    delete this;
}

MakeConnectionError last_err;


enum {
    MIN_SUPPORTED_UHDP_VERSION = 24,
    MAX_SUPPORTED_UHDP_VERSION = 32,
    REBOOT_RETRIES = 3,
    CONNECTION_RETRIES = 5,
    BOOT_CAL_TIMEOUT_RETRIES = 20,
};


class Connection*   make_connection(uint32_t radar_ip, uint32_t timeout_sec, UserLogAgent& log_agent, bool use_thread)
{
    srand(time(0)); // make rand() more random

    Connection_Impl* con = new Connection_Impl(log_agent, use_thread);

    if (con->create_socket() < 0)
    {
        last_err = MCE_SOCKET_FAILURE;
        delete con;
        return NULL;
    }

    // TODO: last_err = MCE_INVALID_RADAR_IP

    uint32_t ver = MAX_SUPPORTED_UHDP_VERSION;
    if (getenv("UHDP_OLD"))
    {
        // Radars with version 31..33 might not be able to negotiate a lower
        // UhDP version number, so we have this hack to start with version 31
        ver = 31;
    }
    for (; ver >= MIN_SUPPORTED_UHDP_VERSION; ver--)
    {
        int ret = con->send_hello(radar_ip, timeout_sec, ver);
        if (ret == 0)
        {
            last_err = MCE_NO_ERROR;
            break;
        }
        if (ret == -2)
        {
            last_err = MCE_UHDP_VERSION_MISMATCH;
        }
        else
        {
            last_err = MCE_NO_RESPONSE;
            delete con;
            return NULL;
        }
    }

    if (last_err == MCE_UHDP_VERSION_MISMATCH)
    {
        printf("%s\n", UHPHandler::last_msg);
        delete con;
        return NULL;
    }

    if (use_thread)
    {
        con->start();
    }

    last_err = MCE_NO_ERROR;
    return con;
}

namespace {

bool reboot(Connection& con)
{
    con.run_diag("sabine_reset", "reset");
    DiagReturnCodeEnum code;
    con.get_last_diag_error(code);
    con.close_and_release();
    uh_usleep(3 * 1000 * 1000); // wait 3 seconds for watchdog timeout
    return code == DIAG_RET_SUCCESS;
}

bool run_warmup_scans(Connection& con)
{
    RDC_ScanControl ctrl;
    ctrl.defaults();
    ctrl.scan_count = 1000;

    RDC_ScanDescriptor desc;
    desc.set_defaults(VP1a);

    UhdpCaptureControl capctrl;
    capctrl.set_defaults();

    Scanning* s = con.setup_scanning(ctrl, &desc, capctrl);
    if (s)
    {
        int32_t i;
        for (i = 0; i < ctrl.scan_count; i++)
        {
            ScanObject* scan = s->wait_for_scan();
            if (scan)
            {
                scan->release();
            }
            else
            {
                break;
            }
        }
        s->release();
        return i == ctrl.scan_count;
    }
    else
    {
        return false;
    }
}

bool run_vrx_align_field_mode(Connection& con)
{
    Calibrate_cal_vrx_align_control_input ctrl;
    ctrl.rb_halfwidth = 10;
    ctrl.flip_threshold = 60;
    ctrl.rx_flip_time = 250;
    ctrl.tx_extra_threshold = 50;
    ctrl.tx_flip_time = 250;
    ctrl.tx_nflip = 3;
    ctrl.max_iterations = 50;
    ctrl.verbose = false;
    ctrl.factory_mode = false;
    ctrl.umsk_mode = 1;

    // temporarily increase allowed diag time for field mode
    uint32_t save_timeout = static_cast<Connection_Impl&>(con).timeout_scale;
    static_cast<Connection_Impl&>(con).timeout_scale = 100;

    DiagReturnCodeEnum code;
    con.run_diag("calibrate", "calibrate_vrx_align_control", ctrl);
    con.get_last_diag_error(code);
    if (code == DIAG_RET_SUCCESS)
    {
        con.run_diag("calibrate", "calibrate_vrx_align_run");
        con.run_diag("calibrate", "calibrate_vrx_align_release");
    }
    else
    {
        con.run_diag("calibrate", "abort");
    }

    static_cast<Connection_Impl&>(con).timeout_scale = save_timeout;

    uint32_t state_mask = con.query_radar_status_bitmask();
    return !!(state_mask & STATE_ATTR_VRX_ALIGN);
}

}


Connection* make_good_connection(uint32_t radar_ip, UserLogAgent& log_agent, bool use_thread)
{
    for (uint32_t i = 0; i < REBOOT_RETRIES; i++)
    {
        Connection* con = NULL;
        for (uint32_t x = 0; x < CONNECTION_RETRIES; x++)
        {
            con = make_connection(radar_ip, 2, log_agent, use_thread);
            if (con)
            {
                break;
            }
            printf("No response in 1 second, retry %d of %d...\n", x, CONNECTION_RETRIES);
        }
        if (!con)
        {
            last_err = MCE_NO_RESPONSE;
            return NULL;
        }
        if (con->is_sabine_b())
        {
            return con;
        }

        uint32_t status = con->query_radar_status_bitmask();
        for (uint32_t x = 0; x < BOOT_CAL_TIMEOUT_RETRIES; x++)
        {
            if (status & STATE_ATTR_OBJECT_INIT)
            {
                break;
            }

            uh_usleep(1000 * 1000); // wait for radar to complete boot-up
            status = con->query_radar_status_bitmask();
        }

        if (!(status & STATE_ATTR_ADI_SYNC))
        {
            printf("\n\nADI sync failure, the radar must be rebooted\n\n\n");
            if (reboot(*con))
            {
                continue;
            }
            else
            {
                last_err = MCE_BAD_STATE;
                return NULL;
            }
        }
        if (!(status & STATE_ATTR_VRX_ALIGN))
        {
            printf("\n\nBoot Vrx Alignment (field mode) failed, running warmup scans...\n\n\n");
            if (!run_warmup_scans(*con))
            {
                con->close_and_release();
                return NULL;
            }
            printf("\n\nRetrying field mode after warmup scans...\n\n\n");
            if (run_vrx_align_field_mode(*con))
            {
                break;
            }
            else if (reboot(*con))
            {
                continue;
            }
            else
            {
                last_err = MCE_BAD_STATE;
                return NULL;
            }
        }

        uh_usleep(100 * 1000); // wait boot logs to quiesce
        last_err = MCE_NO_ERROR;
        return con;
    }

    printf("Radar is not able to complete boot calibrations successfully, scanning not possible\n");
    last_err = MCE_BAD_STATE;
    return NULL;
}


MakeConnectionError get_last_connection_error()
{
    return last_err;
}

const char* UserLogAgent::cpu_names[] = { "SCP", "DSP1", "DSP2", "CCP", "HSM", NULL };
