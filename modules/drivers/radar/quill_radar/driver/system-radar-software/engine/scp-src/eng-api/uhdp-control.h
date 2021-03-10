#ifndef SRS_HDR_UHDP_CONTROL_H
#define SRS_HDR_UHDP_CONTROL_H 1
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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/control-message-def.h"

SRS_DECLARE_NAMESPACE()

#if WITH_UHDP

class UhdpControlMessageHandler
{
public:
    UHVIRTDESTRUCT(UhdpControlMessageHandler)
    /*! Will be called by the UhDP protocol agent when an environment control
     * messages of the registered UhdpControlMessageType is received by the
     * UhDP protocol agent. The message buffer is owned by the protocol agent
     * will be reused immediately after this function returns. The caller must
     * copy any message data it wishes to keep */
    virtual void control_message_received(const CHAR* msg, uint32_t msg_length) = 0;

    /*! Will be called by the UhDP protocol agent when a data logger connects to
     * the radar's UhDP port. */
    virtual void connection_established() = 0;

    /*! Will be called by the UhDP protocol agent when a data logger disconnects
     * from the radar */
    virtual void connection_closed() = 0;
};

//! Interface by which environment code can trasmit or receive control messages
// over the UhDP network protocol.
class UhdpControl
{
public:

    static UhdpControl& instance();

    UhdpControl() {}
    UHVIRTDESTRUCT(UhdpControl)

    /*! Returns true if a data logger application is connected to the radar */
    virtual bool        is_connected() const { return false; }

    /*! Returns the UHDP version which was negotiated for this connection (only
     * applicable when is_connected() returns true) */
    virtual uint32_t    get_uhdp_version() const { return 0; }

    /*! Returns the maximum length, in bytes, of any message written into an
     * environment datagram buffer */
    virtual uint32_t    get_env_datagram_buffer_max_length() const { return 0; }

    /*! Returns a pointer to a network buffer.  Caller must write a control
     * message into the buffer and then call send_env_datagram_buffer(). This
     * function may return NULL if there are no network buffers available (which
     * should be a temporary situation) */
    virtual void*       get_env_datagram_buffer() { return NULL; }

    /*! Completes the process of sending an environment datagram buffer
     * (enqueues the message for transmission). The message type enumerations
     * are defined in the environment header coredefs/control-message-def.h.
     * msg_length must be the number of bytes written into the buffer */
    virtual bool        send_env_datagram_buffer(UhdpControlMessageType t, uint32_t msg_length)
    {
        (void)t;
        (void)msg_length;
        return false;
    }

    /*! Registers a callback function which will be called when environment
     * control messages of the specified type are received by the UhDP protocol
     * agent. ctrl.connection_established() will be called if a data logger is
     * already connected to the radar */
    virtual void        register_control_message_handler(UhdpControlMessageType t, UhdpControlMessageHandler& ctrl)
    {
        (void)t;
        (void)ctrl;
    }
};

#endif


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHDP_CONTROL_H
