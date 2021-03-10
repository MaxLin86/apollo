// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhinet.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/control-message-def.h"

#ifdef _WIN32
#include <Winsock2.h>
#include <time.h>
char* strptime(const char *buf, const char *fmt, struct tm *tm);
#if defined(_MSC_VER)
int gettimeofday(struct timeval * tp, struct timezone * tzp);
int poll(struct pollfd *pfd, int nfds, int timeout);
#else
#include <sys/time.h>
int inet_pton(int af, const char *src, void *dst);
const char *inet_ntop(int af, const void *src, char *dst, socklen_t size);
#endif
#else
#include <sys/time.h>
#include <arpa/inet.h>
#endif

class UserLogAgent
{
public:

    virtual ~UserLogAgent() {}

    //! an array of string names which can be indexed by an IPCTargetEnum
    static const char* cpu_names[];

    //! user must define this function which will be called by the Connection
    //! object (usually from the context of a network polling thread)
    virtual void radar_print_message(ICCQTargetEnum cpu, const char* message) = 0;

    //! user must define this function which will be called by the Connection
    //! object (usually from the context of a network polling thread)
    virtual void radar_log_message(ICCQTargetEnum cpu, LogLevelEnum level, const char* message) = 0;

    //! Handle control message arriving from environment (user) code running
    //! on the radar. It is safe to leave unimplemented.
    virtual void received_control_message(UhdpControlMessageType t, const char* message, uint32_t msg_len)
    {
        (void)t;
        (void)message;
        (void)msg_len;
    }
};

//! an enumeration of the errors which might occur during make_connection()
enum MakeConnectionError
{
    MCE_NO_ERROR,
    MCE_INVALID_RADAR_IP,
    MCE_NO_RESPONSE,
    MCE_BAD_STATE,
    MCE_UHDP_VERSION_MISMATCH,
    MCE_CONNECTION_ALREADY_ESTABLISHED,
    MCE_INCOMPLETE_DISCOVERY,
    MCE_SOCKET_FAILURE,
};

// cdecl to facilitate portable shared library use
extern "C"
{

//! returns NULL if no response is received from the radar in timeout_sec seconds.
//! if use_thread is false, caller is responsible for calling
//! Connection::poll_socket() periodically to handle network traffic. The socket
//! file descriptor can be queried by Connection::get_socket_descriptor(), so it
//! may be monitored by select() or poll() system calls.
//!
//! @param[in] radar_ip             IPv4 address of radar, in network byte order
//! @param[in] timeout_sec          Number of seconds to wait for a response
//! @param[in] log_agent            User-implemented radar logging agent
//! @param[in] use_thread           Connection creates thread to poll network
class Connection* make_connection(uint32_t radar_ip, uint32_t timeout_sec,
                                  UserLogAgent& log_agent, bool use_thread=true);

//! this function behaves the same as make_connection() with two exceptions:
//!   1. it will retry the connection requests to the radar, up to 10 seconds
//!   2. upon connecting to the radar, it ensures the radar is in a state to
//!      run valid scans. The radar is rebooted, if necessary.
//! If this function returns a Connection object, the caller knows that the
//! radar is capable of immediately running scans.
//!
//! @param[in] radar_ip             IPv4 address of radar, in network byte order
//! @param[in] log_agent            User-implemented radar logging agent
//! @param[in] use_thread           Connection creates thread to poll network
class Connection* make_good_connection(uint32_t radar_ip, UserLogAgent& log_agent, bool use_thread=true);

//! returns a string describing the revision of the sabine-remote-api in the
//! format: $LATESTTAG+$LATESTTAGDISTANCE-$HG_NODE_HASH
const char* get_rra_version_string();

//! returns a string describing the revision of the sabine-radar-sw that RRA
//! was compiled against (this can be different from the SRS version present on
//! the radar). The revision format: $LATESTTAG+$LATESTTAGDISTANCE-$HG_NODE_HASH
//! If revids is not NULL, it must point to user-provided storage of two
//! uint32_t for storing the first 16bytes of $HG_NODE_HASH as integers
const char* get_compiled_srs_version_string(uint32_t* revids);

//! returns a string describing the time stamp of the system-radar-sw build
const char* get_compiled_srs_timestamp_string();

//! returns a string describing the path of SRS repo used for system-radar-sw build
const char* get_compiled_srs_path_string();

//! returns the last error encountered by the most recent make_connection() call
MakeConnectionError get_last_connection_error();

}
