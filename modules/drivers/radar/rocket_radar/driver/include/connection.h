// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-scanctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc-threshctrl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/diag-desc.h"
#include <assert.h>

#ifdef _WIN32
#include "Windows.h"
#endif

class Scanning;
class ScanObject;
class LogControl;

SRS_DECLARE_NAMESPACE()
enum UhdpControlMessageType;
SRS_CLOSE_NAMESPACE()

class Connection
{
public:

    virtual ~Connection() {}

    //! returns the operating system descriptor handle for the network socket
    //! which is connected to the radar, for use in select() or poll() system
    //! calls
    virtual int         get_socket_descriptor() const = 0;

    //! manually process UhDP packets that have arrived from the Uhnder radar
    //! and process timers.  Calling this function is only necessary if the
    //! Connection was created with use_thread=false.  When use_thread is false,
    //! all UserLogAgent callbacks will originate from the context of this
    //! function call. All blocking API calls like running diags or waiting for
    //! scan completions will internally call poll_socket() if use_thread is
    //! false. This function is mutex protected so only one thread may enter the
    //! function at a time.
    virtual void        poll_socket() = 0;

    //! Only applicable when the connected radar is running scans with a defined
    //! frame interval. This re-synchronizes the radar’s frame interval base time
    //! to the specified host time.
    virtual void        synchronize_frame_interval(timeval host_time) = 0;

    //! Returns the number of unique antenna configurations supported by the
    //! connected radar (and its radar software). Might return zero if the radar
    //! software version is too old. When this is the case, it will not be able
    //! to run scans for this version of the Radar Remote API
    virtual uint32_t    get_num_antenna_configs() const = 0;

    //! Returns the string name of a radar antenna configuration, if antcfg_id
    //! is less than the count returned by get_num_antenna_configs().
    virtual const char* get_antenna_config_name(uint32_t antcfg_id) const = 0;

    //! Returns the antenna configuration ID which supports the abstract basic
    //! antenna configuration type. All antenna modules should support each of
    //! these basic antenna configuration types.
    virtual uint32_t    get_basic_antenna_config_id(BasicAntennaConfigEnum bac) const = 0;

    // Radar diag introspection routines

    //! returns the number of diags which are supported by your radar. Each diag
    //! instance will support a number of named tests. Each test will define its
    //! own input and output structures.
    virtual uint32_t    get_num_diags() const = 0;

    //! returns the name of a particular diag instance. Each diag instance will
    //! support a number of named tests. Each test will define its own input and
    //! output structures.
    virtual const char* get_diag_name(uint32_t d) const = 0;

    //! returns the number of tests supported by a particular diag instance
    virtual uint32_t    get_num_tests_in_diag(uint32_t d) const = 0;

    //! returns the name of a test supported by a particular diag instance
    virtual const char* get_diag_test_name(uint32_t d, uint32_t test_id) const = 0;

    //! Execute a radar diag specified by ID. User must provide inputs and
    //! storage for outputs.  Returns bytesize of output data. This function
    //! blocks until the diag completes. If the connection has no poll thread,
    //! it calls poll_socket() internally until the diag completes.
    //!
    //! @param[in] diag_id              index of diag
    //! @param[in] test_id              index of test of diag
    //! @param[in] inputs               pointer to diag input struct, can be NULL
    //! @param[in] input_bytesize       size of diag test input struct
    //! @param[in] outputs              user-allocated output buffer, can be NULL
    //! @param[in] outputs_bytesize     size of user-allocated output buffer
    virtual uint32_t    run_diag_by_id(uint32_t diag_id, uint32_t test_id, const void* inputs,
                                       size_t input_bytesize, void* outputs, size_t output_bytesize) = 0;

    //! Execute a radar diag specified by name. User must provide inputs and
    //! storage for outputs.  Returns bytesize of output data. This function
    //! calls run_diag_by_id() which will block for diag completion.
    //!
    //! @param[in] diag_name            name of a diag
    //! @param[in] test_name            name of a test of named diag
    //! @param[in] inputs               pointer to diag input struct, can be NULL
    //! @param[in] input_bytesize       size of diag test input struct
    //! @param[in] outputs              user-allocated output buffer, can be NULL
    //! @param[in] outputs_bytesize     size of user-allocated output buffer
    virtual uint32_t    run_diag_by_name(const char* diag_name, const char* test_name, const void* inputs,
                                         size_t input_bytesize, void* outputs, size_t output_bytesize) = 0;

    bool run_diag(const char* diag_name, const char* test_name)
    {
        return run_diag_by_name(diag_name, test_name, NULL, 0, NULL, 0) == 0;
    }

    template<class Input>
    bool run_diag(const char* diag_name, const char* test_name, const Input& inp)
    {
        return run_diag_by_name(diag_name, test_name, &inp, sizeof(Input), NULL, 0) == 0;
    }

    template<class Input, class Output>
    bool run_diag(const char* diag_name, const char* test_name, const Input& inp, Output& out)
    {
        return run_diag_by_name(diag_name, test_name, &inp, sizeof(Input),
                                &out, sizeof(Output)) == sizeof(Output);
    }

    //! Determine the version number of the interface of a radar diag specified by name
    //! Returns API version of the diag test, 0xFF on error
    //
    //! @param[in] diag_name            name of a diag
    //! @param[in] test_name            name of a test of named diag
    virtual uint8_t     get_diag_test_api_version(const char* diag_name, const char* test_name) const = 0;

    //! If code is set to any value other than DIAG_RET_SUCCESS, the returned
    //! signed integer is the diag error_num value
    virtual int32_t     get_last_diag_error(DiagReturnCodeEnum& code) const = 0;

    //! Send a Force Stop message to the radar; which should only be necessary
    //! with broken diag tests
    virtual void        diag_force_stop() = 0;

    //! Returns an object which can configure log levels on the radar
    virtual LogControl* get_log_control() = 0;

    //! Configure which radar data should be returned from the radar, and their
    //! output frequency. This should be called before scanning, if you wish to
    //! get any scan data. It can be called at any time to change the scan
    //! behavior. Returns false if configuration failed to apply.
    virtual bool        configure_data_capture(const UhdpCaptureControl& cap_ctrl) = 0;

    //! Control radar scanning without the use of a Scanning object. This is an
    //! optional method for running scans that does not involve the creation of
    //! a Scanning object. When scanning is started with this method, the user
    //! must call poll_completed_scan() periodically to collect the returned
    //! scan data. There is no blocking scan wait mechanism in this method.
    //! Returns number of scans enqueued on the radar
    virtual int         send_scan_control(const RDC_ScanControl& scan_ctrl, const RDC_ScanDescriptor* desc) = 0;

    //! Poll for completed scans not associated with a Scanning object, in other
    //! words scans that were issued via a diagnostic test or via RDC_ScanControl
    //! messaging. This function will always return NULL if a Scanning object
    //! has been allocated for this Connection and has not been released
    virtual ScanObject* poll_completed_scan() = 0;

    //! Configure the detection, clutter image, and point cloud thresholds.
    //! This API can be called at any time to change the detection thresholds,
    //! even while scanning
    virtual void        configure_detection_thresholds(const RDC_ThresholdControl* thresh_ctrl, uint32_t count=1) = 0;

    //! Notify the radar of its current telemetry, i.e. the velocity of the
    //! world relative to the radar, etc..  This can be called at any time, as
    //! often as necessary.
    virtual void        configure_radar_telemetry(const UhdpTelemetryData& tel) = 0;

    //! Request that the radar perform MUSIC spectrum analysis for a given set
    //! of target regions. There are several prerequisites for MUSIC to
    //! function. The scan channelizer ratio must be above 1 and divide the
    //! number of pulses evenly into a supported FFT size. MUSIC output must be
    //! enabled for capture. The radar will analyse these regions every scan
    //! until you specify a new set of regions. Each request overrides all
    //! previous requests. It is valid for count to be 0, this stops MUSIC.
    virtual void        request_MUSIC(RDC_music_request* regions, uint32_t count) = 0;

    //! Request digital probes to be performed every scan, and the results
    //! returned in the scan objects.
    virtual void        register_dprobe_request(const RDC_DProbe_corr_config& corr,
                                                const RDC_DProbe_rms_config& rms,
                                                const RDC_DProbe_iq_config& iq,
                                                uint8_t rx_select) = 0;

    virtual void        clear_dprobe_requests() = 0;

    virtual void        register_dcmeasurement_requests(const RDC_DCMeasure_config& dc_measure) = 0;

    //! Requests a 32bit bitmask from the radar which indicates its current
    //! working status, including the results of a few key boot calibrations
    //! If any of these calibrations failed, it is recommended to reboot the
    //! radar again.
    virtual uint32_t    query_radar_status_bitmask() = 0;

    //! Specify the number of data packets the radar can send to the host
    //! without receiving acknowledgement from the host. If this number is
    //! too small, the average latency of each packet will be high and this
    //! will limit the amount of data you can capture from each scan without
    //! dropping scans. If it is too large, you might encounter packet drops
    //! in networking switches or in O/S networking stacks. The default value
    //! is generally preferred (16 on Linux, 8 on Windows). Specify <=1 to reset
    //! back to the default packet window value
    virtual void        specify_data_packet_window(uint8_t packet_window) = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the module. This string is user defined and arbitrary
    virtual const char* get_radar_host_name() = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the module (board) name, eg: "MOD-222-Judo"
    virtual const char* get_module_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the module type, eg: "Judo", "Wingchun", "Kenpo",
    //! or "Kung-fu"
    virtual const char* get_module_type_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the motherboard type, eg: "PIB2", "PIB48", or
    //! "Laufenburg"
    virtual const char* get_motherboard_type_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the antenna daughterboard type, eg: "ADB2",
    //! "Laufenburg", or "Dhaka"
    virtual const char* get_antennaboard_type_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the antenna module type, eg: "ADB2",
    //! "Rhine", "Turag", or "Blanco"
    virtual const char* get_antenna_module_type_name() const = 0;

    //! Returns true if the connected radar has a Sabine B SOC
    virtual bool        is_sabine_b() = 0;

    //! Returns the UhDP protocol version that was negotiated at connect time
    virtual uint8_t     uhdp_protocol_version() = 0;

    //! Blocking function which sends a peek request message to the radar and
    //! waits for a response. Retransmissions are performed if necessary.
    virtual uint32_t    peek_register(uint32_t reg_addr, uint32_t read_mask) = 0;

    //! Blocking function which sends a poke request message to the radar and
    //! waits for a response. Retransmissions are performed if necessary.
    virtual bool        poke_register(uint32_t reg_addr, uint32_t write_mask, uint32_t value) = 0;

    //! Blocking function which sends a read request message to the radar and
    //! waits for a response. Retransmissions are performed if necessary.
    virtual bool        peek_memory(uint32_t dev_addr, uint32_t bytecount, char* output) = 0;

    //! Blocking function which sends a write request message to the radar and
    //! waits for a response. Retransmissions are performed if necessary.
    virtual bool        poke_memory(uint32_t dev_addr, uint32_t bytecount, const char* input) = 0;

    //! Configure radar scans using a custom scan descriptor.
    virtual Scanning*   setup_scanning(const RDC_ScanControl&, const RDC_ScanDescriptor*, const UhdpCaptureControl&) = 0;

    //! Send an environment (user) defined control message to the radar. Each
    //! message must fit within a single ethernet packet. The connection object
    //! manages acknowledgement and flow control.
    virtual bool        send_control_message(UhdpControlMessageType t, const char* msg, uint32_t len) = 0;

    //! transmit a file on the local host to the radar's DDR backed trivial file system
    virtual bool        tftp_upload(const char* local_fname, const char* radar_fname) = 0;

    //! transmit a string buffer the radar's DDR backed trivial file system
    virtual bool        tftp_string_upload(const char* buffer, size_t size_bytes, const char* radar_fname) = 0;

    //! transmit a file from radar's DDR backed trivial file system to the local host
    virtual bool        tftp_download(const char* local_fname, const char* radar_fname) = 0;

    //! send a message to the radar signaling that RDC1 replay uploads are
    //! complete (this is part of the RDC1 replay process)
    virtual void        send_rdc1_data_ready_message() = 0;

    //! return the software version string reported by the radar
    virtual const char* get_radar_software_version() const = 0;

    //! Mark a connection as disabled, meaning that it can no longer be used to
    //! communicate with a radar, except to cleanly close the connection. This
    //! is intended for use in reference counting languages like Python where
    //! a wrapped Connection instance might survive longer than intended, but
    //! it is invalid because the sensor has been rebooted
    virtual void        mark_connection_rebooted() = 0;

    //! Return true if mark_connection_rebooted() has been called on this
    //! Connection instance, indicating that the Connection is broken and
    //! the radar will no longer respond to it
    virtual bool        is_connection_rebooted() const = 0;

    //! Return false if connection has been disabled by mark_connection_rebooted
    //! or if the radar has failed to respond to any request after an appropriate
    //! number of retransmissions
    virtual bool        is_connection_valid() const = 0;

    //! Stop all scans, flush radar data, close radar connection and release
    //! all objects. When this function returns, the caller must discard all
    //! references to Connection and Scanning. ScanObject data is not released,
    //! the user maintains ownership.
    virtual void        close_and_release() = 0;

    //! Same as close_and_release() except the radar is also rebooted after
    //! the connection is closed. It will be several seconds before the radar
    //! will respond to new connection attempts.
    virtual void        close_and_reboot() = 0;

    //! Request a comma separated list of names of hardware units which can be
    //! queried for register core dumps. The response can be NULL if there was
    //! an error.  Output will be deterministic for a given radar product.
    //! The returned buffer is valid until this Connection is released.
    //! example output: "adi,acu_rx,acu_tx,rsu1,rsu2,ciu,feu,rau,nsu,seu,mce"
    virtual const char* get_requestable_hardware_unit_names() = 0;

    //! Request a register core dump from a specific hardware unit, hw_unit_name
    //! must match a name returned by get_requestable_hardware_unit_names(). The
    //! returned buffer will have 2 x num_addr_value_tuples uint32_t values.  The
    //! first value (and all even indexed values) is a 32bit hardware address. The next
    //! value (and all odd indexed values) is the 32bit value read from that
    //! address.  The returned memory buffer is valid until the Connection is
    //! released or until a subsequent call to this function. If the hardware
    //! unit has preload buffers, the preload argument selects which one is
    //! returned. the cache argument determines if the cached register values are
    //! returned or the raw hardware register values
    virtual const uint32_t* get_hardware_unit_core_dump(const char* hw_unit_name, INT preload, uint32_t& num_addr_value_tuples) = 0;

    //! get the mounting geometry distances of this radar (as configured in its modulecfg file)
    virtual void            get_mounting_distances(float& rear_axle_dist, float& centerline_dist, float& height) const = 0;

    //! get the mounting geometry angles of this radar (as configured in its modulecfg file)
    //! Positive mount angle implies the radar is pointing to the right of the vehicle boresight,
    //! i.e.: mounted on the right front corner of the vehicle).
    virtual void            get_mounting_angles(float& azimuth_deg, float& elevation_deg) const = 0;

    //! Network protocol packet and error counters maintained by connection
    struct Counters
    {
        uint32_t wrong_uhdp_version;   //!< number of UhDP messages received with incorrect version
        uint32_t wrong_payload_length; //!< number of UhDP messages which had incorrect total length
        uint32_t wrong_message_type;   //!< number of UhDP messages which had an unsupported message type
        uint32_t total_packets_rcvd;   //!< total number of UhDP messages received
        uint32_t dropped_data_packets; //!< total number detected scan data packets dropped
        uint32_t dropped_scans;        //!< total number scans not output by radar (dropped due to back-pressure)
        uint32_t missing_scans;        //!< total number scans without scan info
        uint32_t data_without_scan_info; //!< scan data packets received for scans without scan info
        uint32_t tftp_retransmissions; //!< number of times a TFTP operation had to be repeated (packet drop)
        uint32_t diag_retransmissions; //!< number of times a diag message had to be repeated (packet drop)

        //! convenience function to write the counters to a FILE handle
        void dump(FILE* fp) const
        {
            if (wrong_uhdp_version)   { fprintf(fp, "wrong_uhdp_version: %u\n",   wrong_uhdp_version); }
            if (wrong_payload_length) { fprintf(fp, "wrong_payload_length: %u\n", wrong_payload_length); }
            if (wrong_message_type)   { fprintf(fp, "wrong_message_type: %u\n",   wrong_message_type); }
            if (total_packets_rcvd)   { fprintf(fp, "total_packets_rcvd: %u\n",   total_packets_rcvd); }
            if (dropped_data_packets) { fprintf(fp, "dropped_data_packets: %u\n", dropped_data_packets); }
            if (dropped_scans)        { fprintf(fp, "dropped_scans: %u\n",        dropped_scans); }
            if (missing_scans)        { fprintf(fp, "missing_scans: %u\n",        missing_scans); }
            if (data_without_scan_info) { fprintf(fp, "data_without_scan_info: %u\n", data_without_scan_info); }
            if (tftp_retransmissions) { fprintf(fp, "tftp_retransmissions: %u\n", tftp_retransmissions); }
            if (diag_retransmissions) { fprintf(fp, "diag_retransmissions: %u\n", diag_retransmissions); }
        }
    };

    //! returns a constant reference to the connection's status counters. These
    //! may be updated by a background thread at any time.
    virtual const Counters& get_connection_counters() const = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        CON_NO_ERROR,
        CON_INPUT_OUT_OF_RANGE,
        CON_DIAG_ALREADY_RUNNING,
        CON_DIAG_INPUT_TOO_LARGE,
        CON_DIAG_OUTPUT_TOO_LARGE,
        CON_DIAG_UNKNOWN_NAME,
        CON_DIAG_ID_OUT_OF_RANGE,
        CON_DIAG_UNKNOWN_TEST_NAME,
        CON_DIAG_TEST_ID_OUT_OF_RANGE,
        CON_DIAG_FAILURE,
        CON_DIAG_HANG,
        CON_SCAN_DESCRIPTOR_INVALID,
        CON_ALREADY_SCANNING,
        CON_PEEK_INVALID_ADDRESS,
        CON_POKE_INVALID_ADDRESS,
        CON_PEEK_INVALID_SIZE,
        CON_POKE_INVALID_SIZE,
        CON_UNABLE_TO_OPEN_FILE,
        CON_UNABLE_TO_READ_FILE,
        CON_SOCKET_FAILURE,
        CON_TFTP_INPUT_TOO_LARGE,
        CON_TFTP_OPEN_ERROR,
        CON_TFTP_SEND_ERROR,
    };

    //! returns the last error encountered by this instance
    virtual Err         get_last_error() const = 0;

    // Same functionality as configure_radar_telemetry(), but it does not
    // require a connection to the sensor. For use in situations where the
    // inertial navigation system is a completely different process (or on a
    // completely different processor) than the one managing the radar
    // connection and radar data outputs. Returns true if temporary socket was
    // created and UhDP message was sent.
    //
    // Note that the phased_array_azimuth field of UhdpTelemetryData has no
    // effect when the telemetry is transmitted without a connection in this
    // way.
    static bool  connectionless_radar_telemetry_update(uint32_t radar_ip, const UhdpTelemetryData& tel);
};
