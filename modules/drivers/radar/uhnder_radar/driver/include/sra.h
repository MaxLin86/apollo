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
#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhinet.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/control-message-def.h"

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
    virtual void received_control_message(UhdpControlMessageType t, const char* message, uint32_t msg_len) {}
};

//! an enumeration of the errors which might occur during make_connection()
enum MakeConnectionError
{
    MCE_NO_ERROR,
    MCE_INVALID_RADAR_IP,
    MCE_INVALID_LOCAL_IP,
    MCE_NO_RESPONSE,
    MCE_BAD_STATE,
    MCE_UHDP_VERSION_MISMATCH,
    MCE_CONNECTION_ALREADY_ESTABLISHED,
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
//! @param[in] local_ip             IPv4 address of host, in network byte order
//! @param[in] timeout_sec          Number of seconds to wait for a response
//! @param[in] log_agent            User-implemented radar logging agent
//! @param[in] use_thread           Connection creates thread to poll network
class Connection* make_connection(uint32_t radar_ip, uint32_t local_ip, uint32_t timeout_sec,
                                  UserLogAgent& log_agent, bool use_thread=true);

//! this function behaves the same as make_connection() with two exceptions:
//!   1. it will retry the connection requests to the radar, up to 10 seconds
//!   2. upon connecting to the radar, it ensures the radar is in a state to
//!      run valid scans. The radar is rebooted, if necessary.
//! If this function returns a Connection object, the caller knows that the
//! radar is capable of immediately running scans.
//!
//! @param[in] radar_ip             IPv4 address of radar, in network byte order
//! @param[in] local_ip             IPv4 address of host, in network byte order
//! @param[in] log_agent            User-implemented radar logging agent
//! @param[in] use_thread           Connection creates thread to poll network
class Connection* make_good_connection(uint32_t radar_ip, uint32_t local_ip,
                                       UserLogAgent& log_agent, bool use_thread=true);

//! returns a string describing the revision of the sabine-remote-api in the
//! format: $LATESTTAG+$LATESTTAGDISTANCE-$HG_NODE_HASH
const char* get_sra_version_string();

//! returns a string describing the revision of the sabine-radar-sw that SRA
//! was compiled against (this can be different from the SRS version present on
//! the radar). The revision format: $LATESTTAG+$LATESTTAGDISTANCE-$HG_NODE_HASH
//! If revids is not NULL, it must point to user-provided storage of two
//! uint32_t for storing the first 16bytes of $HG_NODE_HASH as integers
const char* get_compiled_srs_version_string(uint32_t* revids);

//! returns the last error encountered by the most recent make_connection() call
MakeConnectionError get_last_connection_error();

}
