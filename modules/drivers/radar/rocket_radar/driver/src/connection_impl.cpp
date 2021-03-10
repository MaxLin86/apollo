#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "modules/drivers/radar/rocket_radar/driver/src/srs-profiling.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanning_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
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
#include "modules/drivers/radar/rocket_radar/driver/include/vehiclecan.h"

/* these must be included in a particular order, let sleeping dogs lie */
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/discovery-diag.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-logmsg.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-antcfg.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//discovery-ppa.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp//logaggr.h"

#include <assert.h>
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
"*** to take longer than one second, this diag has not completed in many seconds.  We do\n"
"*** allow exceptions for this rule in a few situations, particularly for calibration\n"
"*** To extend the diag timeout one must set the environment variable UHDP_HELLO_TIMEOUT\n"
"*** prior to connecting to the radar. For example:\n"
"***\n"
"*** import os; os.environ['UHDP_HELLO_TIMEOUT'] = '1'\n"
"***\n";

struct NopHandler : public ProtHandlerBase
{
    NopHandler(Connection_Impl& impl, uint8_t type) : my_con(impl), my_type(type) {}

    void handle_packet(const char* payload, uint32_t len)
    {
        //printf("NOP handler type %u, %x, len=%u\n", my_type, my_type, len);
        my_con.counters.wrong_message_type++;
    }

    Connection_Impl& my_con;
    uint32_t my_type;
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


void            Connection_Impl::allocate_protocols()
{
    memset(uhdp_table, 0, sizeof(uhdp_table));

    LogMsgDiscHandler* msg_disc = new LogMsgDiscHandler();

    uhdp_table[UHDP_TYPE_PRINT]             = new UHPHandler();
    uhdp_table[UHDP_TYPE_PPA]               = new PPAHandler(*this);
    uhdp_table[UHDP_TYPE_LOG_AGGR]          = new LogAggregateHandler(log_agent, *logcontrol, *msg_disc);
    uhdp_table[UHDP_TYPE_LOG_PRODUCER_NAME] = static_cast<ProtHandlerBase*>(logcontrol);
    uhdp_table[UHDP_TYPE_LOG_MESSAGE]       = msg_disc;
    uhdp_table[UHDP_TYPE_DIAG_DISCOVERY]    = new DiagDiscHandler();
    uhdp_table[UHDP_TYPE_ANTENNA_CONFIG]    = new AntennaConfigDiscHandler();
    uhdp_table[UHDP_TYPE_LOG_TASK_NAME]     = new PPADiscHandler();
    uhdp_table[UHDP_TYPE_ENV_CONTROL_DATA]  = new EnvControlDataHandler(*this);
    uhdp_table[UHDP_TYPE_ENV_CONTROL_ACK]   = new EnvControlAckHandler(*this);
    uhdp_table[UHDP_TYPE_FAST_CAPTURE]      = new FastCaptureHandler(*this);

    for (uint32_t i = 0; i < 256; i++)
    {
        if (!uhdp_table[i])
        {
            uhdp_table[i] = new NopHandler(*this, i);
        }
    }
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

bool            Connection_Impl::apply_rdc3_blob(uint32_t scan_sequence_number, char* blob, uint32_t size_bytes)
{
    while (cur_in_scan)
    {
        if (scan_sequence_number == cur_in_scan->scan_info.scan_sequence_number)
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
    if (cur_in_scan)
    {
        cur_in_scan->attach_blob(blob, size_bytes);
        return true;
    }
    else
    {
        last_in_scan = NULL;
        return false;
    }
}

void            Connection_Impl::send_rdc1_data_ready_message()
{
    UhdpHeader hdr;
    hdr.message_type = UHDP_TYPE_RDC1_DATA_READY;
    hdr.total_length = sizeof(UhdpHeader);
    send_uhdp(&hdr);
}


void            Connection_Impl::diag_force_stop()
{
    UhdpHeader hdr;
    hdr.message_type = UHDP_TYPE_DIAG_FORCE_STOP;
    hdr.total_length = sizeof(UhdpHeader);
    send_uhdp(&hdr);
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
    if (input_bytesize > 1024)
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
    diag_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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

        if (outputs && output_bytesize)
        {
            // go ahead and copy partial output
            memcpy(outputs, comp + 1, output_bytesize);
        }
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
    peek_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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

const char*     Connection_Impl::get_radar_host_name()
{
    static ModuleFlashData data;
    data.set_defaults();

    run_diag_by_name("tfs", "get_module_cfg", NULL, 0, &data, sizeof(data));

    return &data.radar_hostname[0];
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
    status_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    assert(sabine_type != ST_UNKNOWN); // should always be known
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
    peek_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    peek_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    peek_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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

    resend_count = 0;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    control_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    control_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    core_dump_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    core_dump_last_resend = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;

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
    if (connection_active)
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
    }

    assert(myscanning == &s);
    delete &s;
    myscanning = NULL;
}


void            Connection_Impl::queue_completed_scan(ScanObject_Impl* scan)
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


void            Connection_Impl::synchronize_frame_interval(timeval host_time)
{
    frame_interval_base_host_time = host_time;

    send_frame_interval_base_time();
}


void            Connection_Impl::send_frame_interval_base_time()
{
    char buf[sizeof(UhdpHeader) + sizeof(UhdpTimeAlignment)];
    UhdpHeader* hdr = (UhdpHeader*)buf;
    hdr->message_type = UHDP_TYPE_TIME_ALIGN;
    hdr->total_length = sizeof(buf);

    // map host time to radar time
    uint64_t t = uint64_t(USEC_PER_SEC) * frame_interval_base_host_time.tv_sec + frame_interval_base_host_time.tv_usec;
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


void            Connection_Impl::close_and_release()
{
    if (mysensorgroup)
    {
        printf("Connections in a SensorGroup cannot be closed ad-hoc, ignoring close_and_release()\n");
        return;
    }

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
        request_close_time = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;
    }

    if (use_thread)
    {
        // when UHDP_TYPE_CLOSE_ACK is received, abort_connection() gets called
        stop();
    }

    delete this;
}

void            Connection_Impl::close_and_reboot()
{
    if (mysensorgroup)
    {
        printf("Connections in a SensorGroup cannot be closed ad-hoc, ignoring close_and_reboot()\n");
        return;
    }

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
        request_close_time = uint64_t(USEC_PER_SEC) * tv.tv_sec + tv.tv_usec;
    }

    if (use_thread)
    {
        // when UHDP_TYPE_CLOSE_ACK is received, abort_connection() gets called
        stop();
    }

    delete this;
}

const char* UserLogAgent::cpu_names[] = { "SCP", "DSP1", "DSP2", "CCP", "HSM", NULL };
