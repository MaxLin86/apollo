#ifndef SRS_HDR_LOGGING_H
#define SRS_HDR_LOGGING_H 1
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
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"

SRS_DECLARE_NAMESPACE()


class LoadedIOVec;

/*! Abstract API of Logging Agent */
class LogAgent
{
public:

    virtual ~LogAgent() {}

    /*! a CPU has generated a printf message */
    virtual void agent_print(ICCQTargetEnum cpu, const CHAR* uhpmsg, uint32_t slen) = 0;

    /*! a CPU has generated a log message event */
    virtual void agent_log(ICCQTargetEnum cpu, const LogEvent& ev) = 0;

    /*! the SCP has generated a profiling event */
    virtual void agent_ppa(const LogEvent& ev) = 0;

#if __SCP__
    /*! returns an IOVEC that can be used by a remote CPU for logging and profiling. */
    virtual LoadedIOVec* allocate_iovec(uint32_t& buffer_size) = 0;

    /*! a remote CPU has filled an IOVEC buffer with profile events, agent is
     * responsible for releasing the IOVEC (or sending it to ethernet) */
    virtual void remote_ppa(ICCQTargetEnum cpu, LoadedIOVec& iovec) = 0;

    /*! called when flushing of log messages from remote CPUs has completed */
    virtual void flush_complete() = 0;
#endif
};

/*! producers of log messages (those enumerated in LogProducerEnum) may derive
 * from this struct or declare an instance of it and register this method for
 * mapping a subunit ID into a name. The most notable example of this is that
 * the DIAG layer can use diagnostic classes as logging subunits and return the
 * diag class names from this method */
struct LogProducer
{
    virtual ~LogProducer() {}

    virtual const CHAR* get_subunit_name(uint32_t subunit) const = 0;
    virtual uint32_t get_num_subunits() const = 0;
};

/*! The SystemLogger is a mechanism for the radar software to communicate
 * status and progress to the user via UHDP (embedded) or simple printf (PC) */
class SystemLogger
{
public:

    static SystemLogger& instance();

    virtual ~SystemLogger() {}

    virtual void init() = 0;

    /*! called by UHDP or other environment logging agent when they are capable of
     * handling messages emitted by the engine */
    virtual void set_log_agent(LogAgent& agent) = 0;

    virtual void clear_log_agent() = 0;

    virtual LogAgent* get_log_agent() = 0;

    /*! enable/disable the use of the UART for console logging by the SCP and HSM */
    virtual void uart_control(bool enable) = 0;

    virtual bool uart_enabled() const = 0;

    /*! control emission of PPA profiler events by SCP and remote CPUs */
    virtual void ppa_capture_control(bool enable) = 0;

    virtual bool uhprintf_enabled() const = 0;

    /*! control emission of UHPRINTF by SCP and remote CPUs */
    virtual void uhprintf_control(bool enable) = 0;

    virtual bool ppa_capture_enabled() const = 0;

    /* SCP logging APIs */
    virtual void emit_pedantic(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_debug(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_verbose(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_info(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_warn(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_error(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_always(LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1 = 0, uint32_t data2 = 0) = 0;
    virtual void emit_fatal(const CHAR* sourcefile, INT lineno, LogProducerEnum p, uint32_t subunit, LogMessageEnum e, uint32_t data1, uint32_t data2) = 0;

    /* SCP profiling APIs */
    virtual void profile_start(LogProducerEnum p, LogTaskEnum taskID, uint32_t data1, uint32_t data2, uint32_t timestamp = 0) = 0;
    virtual void profile_stop(LogProducerEnum p, LogTaskEnum taskID, uint32_t data1, uint32_t data2, uint32_t timestamp = 0) = 0;

#if __SCP__

#if ENABLE_UHPRINTF
    virtual void log_to_string(CHAR buf[MAX_LOG_LENGTH], const LogEvent&ev) const = 0;
#endif

    /*! Each LogProducerEnum may register a LogProducer API which will provide
     * the names of the logging subunits, if that producer requires subunits */
    virtual void register_log_producer(LogProducerEnum p, const LogProducer& lp) = 0;

    /*! Set the log level of a given subunit of a log producer. Messages below
     * this log level will be silently discarded */
    virtual void set_filter_log_level(LogProducerEnum p, uint32_t subunit, LogLevelEnum l) = 0;

    /*! The currently configured log levels are discoverable, allowing debug
     * code to be bypassed if debug output is disabled, for instance */
    virtual LogLevelEnum get_filter_log_level(LogProducerEnum p, uint32_t subunit) const = 0;

    /*! Request all remote CPUs to flush any queued print or log messages, when
     * the flush has completed, log_agent->flush_complete() will be called */
    virtual void flush_buffered() = 0;

    virtual LogMessageEnum get_last_log_message() const = 0;

    bool   unit_test_mode;

    // for use by uhdp logging agent
    virtual const CHAR* get_producer_name(LogProducerEnum p, uint32_t* numsubunits) const = 0;
    virtual const CHAR* get_subunit_name(LogProducerEnum p, uint32_t subunit_id) const = 0;
    virtual const CHAR* get_log_level_name(LogLevelEnum l) const = 0;
    virtual const CHAR* get_message_string(LogMessageEnum e) const = 0;
    virtual const CHAR* get_task_name(LogTaskEnum t) const = 0;

#endif // if __SCP__
};

/*! ProfileScope is used to profile the elapsed time of a section of code. Each
 * log producer defines their list of profile tasks. You simply declare an
 * instance of this class on your stack, providing the log producer ID and the
 * task type enumeration. Its constructor records the start time, and when the
 * object goes out of scope the destructor notes the stop time and passes all of
 * this info to the system logger. The user can optionally specify integer(s)
 * which describes the work performed by the block. For example:
 *
 * {
 *     ProfileScope e(LOG_OS, PROC_ISR_EVENTS, num_isr_events);
 *     for (INT i = 0; i < num_isr_events; i++)
 *         isr[i]->trigger();
 * }
 */
#if DISABLE_PROFILING

class ProfileScope
{
    ProfileScope(LogProducerEnum p, LogTaskEnum tid) {}

    void set_data(uint32_t a, uint32_t b = 0, uint32_t c = 0, uint32_t d = 0) {}

    ~ProfileScope() {}
};

#else // if DISABLE_PROFILING

class ProfileScope
{
public:

    ProfileScope(LogProducerEnum p, LogTaskEnum tid)
    {
        prod = p;
        taskID = tid;
        data1 = data2 = data3 = data4 = 0;
        start_timestamp = uh_read_clock();
    }


    void set_data(uint32_t d1, uint32_t d2 = 0, uint32_t d3 = 0, uint32_t d4 = 0)
    {
        data1 = d1;
        data2 = d2;
        data3 = d3;
        data4 = d4;
    }


    ~ProfileScope()
    {
        if (SystemLogger::instance().ppa_capture_enabled())
        {
            uint32_t end_timestamp = uh_read_clock();
            SystemLogger::instance().profile_start(prod, taskID, data1, data2, start_timestamp);
            SystemLogger::instance().profile_stop(prod, taskID, data3, data4, end_timestamp);
        }
    }


private:

    uint32_t        data1, data2, data3, data4;
    uint32_t        start_timestamp;
    LogTaskEnum     taskID;
    LogProducerEnum prod;
};

#endif // if DISABLE_PROFILING


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_LOGGING_H
