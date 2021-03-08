#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/ip-protocols.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/sra.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/connection.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/threading.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/logcontrol_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhstdlib.h"

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhnder-helpers.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/uhdp/prothandlerbase.h"

class ScanObject_Impl;
class Scanning_Impl;
class ControlMessage;
class UserProfileAgent;
struct ProtHandlerBase;

class Connection_Impl : public Connection, public Thread
{
public:

    enum { MAX_RESEND_COUNT = 5 };

    enum SabineType
    {
        ST_UNKNOWN,
        ST_SABINE_A,
        ST_SABINE_B,
    };

    Connection_Impl(UserLogAgent& a, bool _use_thread)
        : last_err(CON_NO_ERROR)
        , sockfd(-1)
        , log_agent(a)
        , released(false)
        , use_thread(_use_thread)
        , connection_active(false)
        , diag_is_hung(false)
        , command_sequence_number(0)
        , diag_sequence_number(rand())
        , peek_sequence_number(0)
        , resend_count(0)
        , peek_last_resend(0)
        , status_last_resend(0)
        , diag_last_resend(0)
        , request_close_time(0)
        , control_last_resend(0)
        , core_dump_last_resend(0)
        , training_radar_host_delta(0)
        , radar_host_delta(0)
        , last_diag_return_code(DIAG_RET_SUCCESS)
        , last_diag_error_num(0)
        , myscanning(NULL)
        , scanlist_cond(scanlist_mutex)
        , cur_in_scan(NULL)
        , last_in_scan(NULL)
        , first_pending_scan(NULL)
        , last_pending_scan(NULL)
        , first_out_scan(NULL)
        , last_out_scan(NULL)
        , first_cm(NULL)
        , last_cm(NULL)
        , pending_cm(NULL)
        , vehicle(NULL)
        , profile_agent(NULL)
        , next_cm_sequence(0)
        , current_window(INITIAL_PACKET_WINDOW)
        , cur_pkt_counter(0)
        , last_scan_seq(0)
        , last_msg_type(UHDP_TYPE_HELLO)
        , timeout_scale(1)
        , rear_axle_distance(3.91f)
        , centerline_distance(0)
        , mount_height(0.7f)
        , mount_azimuth(0)
        , mount_elevation(0)
        , sabine_type(ST_UNKNOWN)
        , srs_version_str(NULL)
        , module_name(NULL)
        , module_type_name(NULL)
        , motherboard_type_name(NULL)
        , antennaboard_type_name(NULL)
        , antenna_module_type_name(NULL)
        , hw_unit_names(NULL)
        , lld_core_dump_buffer(NULL)
        , lld_core_dump_size(0)
        , lld_core_dump_receive_count(0)
    {
        memset(&counters, 0, sizeof(counters));
        memset(&scanctrl, 0, sizeof(scanctrl));
        scan_interval_base_host_time.tv_sec = 0;
        scan_interval_base_host_time.tv_usec = 0;
        last_time_align.timestamp32 = 0;
        last_time_align.timestamp64 = 0;
        last_time_align.scan_sequence_number = -1;
        logcontrol = new LogControl_Impl(*this);
        allocate_protocols();
    }

    virtual ~Connection_Impl();

    virtual int             get_socket_descriptor() const { return sockfd; }

    virtual void            poll_socket();

    virtual void            register_vehicle_CAN_device(VehicleCAN& v);

    virtual void            synchronize_scan_interval(timeval host_time);

    virtual uint32_t        get_num_diags() const;

    virtual const char*     get_diag_name(uint32_t i) const;

    virtual uint32_t        get_num_tests_in_diag(uint32_t d) const;

    virtual const char*     get_diag_test_name(uint32_t d, uint32_t test_id) const;

    virtual uint32_t        run_diag_by_id(uint32_t diag_id, uint32_t test_id, const void* inputs, size_t input_bytesize, void* outputs, size_t output_bytesize);

    virtual uint32_t        run_diag_by_name(const char* diag_name, const char* test_name, const void* inputs, size_t input_bytesize, void* outputs, size_t output_bytesize);

    virtual uint8_t         get_diag_test_api_version(const char* diag_name, const char* test_name) const;

    virtual int32_t         get_last_diag_error(DiagReturnCodeEnum& code) const;

    virtual void            diag_force_stop();

    virtual LogControl*     get_log_control() { return logcontrol; }

    virtual bool            configure_data_capture(const UhdpCaptureControl& cap_ctrl);

    virtual int             send_scan_control(const RDC_ScanControl& scan_ctrl, const RDC_ScanDescriptor* desc);

    virtual ScanObject*     poll_completed_scan();

            bool            scan_available();

    virtual void            configure_detection_thresholds(const UhdpThresholdControl& thresh_ctrl);

    virtual void            configure_adaptive_thresholds(const RDC_AdaptiveThreshold& static_thresh, const RDC_AdaptiveThreshold& moving_thresh, bool twoD);

    virtual void            configure_radar_telemetry(const UhdpTelemetryData& tel);

    virtual const char*     get_radar_software_version() const { return srs_version_str; }

    virtual void            request_MUSIC(RDC_music_request* regions, uint32_t count);

    virtual uint32_t        query_radar_status_bitmask();

    virtual const char*     get_module_name() const { return module_name; }

    virtual const char*     get_module_type_name() const { return module_type_name; }

    virtual const char*     get_motherboard_type_name() const { return motherboard_type_name; }

    virtual const char*     get_antennaboard_type_name() const { return antennaboard_type_name; }

    virtual const char*     get_antenna_module_type_name() const { return antenna_module_type_name; }

    virtual bool            is_sabine_b();

    virtual uint8_t         uhdp_protocol_version() { return uint8_t(connection_uhdp_version); }

    virtual uint32_t        peek_register(uint32_t reg_addr, uint32_t read_mask);

    virtual bool            poke_register(uint32_t reg_addr, uint32_t write_mask, uint32_t value);

    virtual bool            peek_memory(uint32_t dev_addr, uint32_t bytecount, char* output);

    virtual bool            poke_memory(uint32_t dev_addr, uint32_t bytecount, const char* input);

    virtual Scanning*       setup_scanning(const RDC_ScanControl&, const RDC_ScanDescriptor*, const UhdpCaptureControl&);

    virtual bool            tftp_upload(const char* local_fname, const char* radar_fname);

    virtual bool            tftp_download(const char* local_fname, const char* radar_fname);

    virtual void            close_and_release();

    virtual void            close_and_reboot();

    virtual const Counters& get_connection_counters() const { return counters; }

    virtual bool            send_control_message(UhdpControlMessageType t, const char* msg, uint32_t len);

    virtual Err             get_last_error() const { return last_err; }

    virtual void            thread_main();

    virtual uint32_t        get_num_antenna_configs() const;

    virtual const char*     get_antenna_config_name(uint32_t antcfg_id) const;

    virtual uint32_t        get_basic_antenna_config_id(BasicAntennaConfigEnum bac) const;

    virtual const char*     get_requestable_hardware_unit_names();

    virtual const uint32_t* get_hardware_unit_core_dump(const char* hw_unit_name, uint32_t& num_addr_value_tuples);

    virtual void            get_mounting_distances(float& axle, float& centerline, float& height) const
    {
        axle = rear_axle_distance; centerline = centerline_distance; height = mount_height;
    }

    virtual void            get_mounting_angles(float& azimuth, float& elevation) const
    {
        azimuth = mount_azimuth;
        elevation = mount_elevation;
    }

    void                    start_radar_profiling(UserProfileAgent& agent);

    void                    stop_radar_profiling();

    void                    poll_socket_internal();

    int                     create_socket();

    void                    abort_connection();

    void                    release_scanning(Scanning_Impl&);

    void                    send_uhdp(struct UhdpHeader* hdr);

    void                    send_scan_interval_base_time();

    void                    allocate_protocols();

    ScanObject_Impl*        scan_wait();

    void                    queue_completed_scan(ScanObject_Impl*, bool force=false);

    int                     send_hello(uint32_t radar_ip, uint32_t local_ip, uint32_t timeout_sec, uint32_t uhdp_ver);

    void                    env_control_ack_received(uint16_t sequence_number);

    mutable Err         last_err;

    int                 sockfd;

    UserLogAgent&       log_agent;

    bool                released;

    bool                use_thread;

    bool                connection_active;

    bool                diag_is_hung;

    ProtHandlerBase*    uhdp_table[256];

    char                peek_buffer[1024 + 64];

    char                status_buffer[1024];

    char                control_msg_buffer[1024];

    char                lld_core_dump_msg_buffer[128];

    char                diag_msgbuf[MAX_UDP_DATAGRAM]; // for resends & outputs

    struct sockaddr_in  radar_ip4addr;

    timeval             scan_interval_base_host_time;

    uint32_t            connection_uhdp_version;

    uint32_t            command_sequence_number;

    uint32_t            diag_sequence_number;

    uint32_t            peek_sequence_number;

    uint32_t            resend_count;

    uint64_t            peek_last_resend;

    uint64_t            status_last_resend;

    uint64_t            diag_last_resend;

    uint64_t            request_close_time;

    uint64_t            control_last_resend;

    uint64_t            core_dump_last_resend;

    int64_t             training_radar_host_delta; /* host clock - radar clock */

    int64_t             radar_host_delta; /* host clock - radar clock */

    DiagReturnCodeEnum  last_diag_return_code;

    int32_t             last_diag_error_num;

    Scanning_Impl*      myscanning;

    LogControl_Impl*    logcontrol;

    Counters            counters;

    RDC_ScanControl     scanctrl;

    UhdpTimeAlignment   last_time_align;

    Event               diag_comp_event;

    Event               peek_comp_event;

    Event               control_ack_event;

    Event               core_dump_ack_event;

    Mutex               poll_mutex;

    Mutex               scanlist_mutex;

    Mutex               sendto_mutex;

    ConditionVar        scanlist_cond;

    /* scans coming from radar, cur_in_scan points to scan currently being
     * transmitted, followed by scans which have received ScanInformation.
     * scans in this list are in radar output order */
    ScanObject_Impl*    cur_in_scan;
    ScanObject_Impl*    last_in_scan;

    /* scans which cannot go into the output list because one or more scans are
     * missing. Scans are inserted into this list in sequence order */
    ScanObject_Impl*    first_pending_scan;
    ScanObject_Impl*    last_pending_scan;

    /* scans waiting to be retrieved by the user, scans in this list are in
     * sorted order. All scans in this list are free to be taken */
    ScanObject_Impl*    first_out_scan;
    ScanObject_Impl*    last_out_scan;

    ControlMessage*     first_cm;

    ControlMessage*     last_cm;

    ControlMessage*     pending_cm;

    VehicleCAN*         vehicle;

    UserProfileAgent*   profile_agent;

    uint16_t            next_cm_sequence;

    uint32_t            current_window;

    uint32_t            cur_pkt_counter;

    uint32_t            last_scan_seq;

    uint32_t            last_msg_type;

    uint32_t            timeout_scale;

    float               rear_axle_distance;

    float               centerline_distance;

    float               mount_height;

    float               mount_azimuth;

    float               mount_elevation;

    SabineType          sabine_type;

    const char*         srs_version_str;

    const char*         module_name;

    const char*         module_type_name;

    const char*         motherboard_type_name;

    const char*         antennaboard_type_name;

    const char*         antenna_module_type_name;

    const char*         hw_unit_names;

    uint32_t*           lld_core_dump_buffer;

    uint32_t            lld_core_dump_size;

    uint32_t            lld_core_dump_receive_count;
};
