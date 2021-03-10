// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHDP_CONTROL_H
#define SRS_HDR_UHDP_CONTROL_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "control-message-def.h"

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
