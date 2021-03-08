#ifndef SRS_HDR_ENG_ICCQ_H
#define SRS_HDR_ENG_ICCQ_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhnder-helpers.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/iccq-memory.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"

SRS_DECLARE_NAMESPACE()


struct ICCQMessage
{
    enum { ICCQ_PAYLOAD_BYTES = 60 };

    uint8_t dest_cpu;    /* ICCQTargetEnum  */
    uint8_t dest_port;   /* ICCQServiceEnum */
    uint8_t source_cpu;  /* ICCQTargetEnum  */
    uint8_t source_port; /* ICCQServiceEnum */
    int8_t  iccq_payload[ICCQ_PAYLOAD_BYTES];
};

struct IccqMessageHandler
{
    virtual ~IccqMessageHandler() {}
    virtual void iccq_receive(ICCQMessage& msg) = 0;
};

class InterCoreCommQueue
{
public:

    static InterCoreCommQueue& instance();

    InterCoreCommQueue();

    void init();

    /*! called from main loop, returns true if any message was received */
    bool poll();

    const CHAR* get_cpu_name(const ICCQTargetEnum cpu) const;

    /*! can return false if resource allocation or back-pressure prevents the
     * message from being queued */
    bool send_iccq_message(ICCQTargetEnum        tocpu,
                           const ICCQServiceEnum toport,
                           const int8_t* const   iccq_payload,
                           const size_t          payload_size,
                           const bool            high_priority = false,
                           ICCQServiceEnum       fromport = ICCQ_SERVICE_DUP);

    /*! identical to post_event except event is posted to a remote CPU via ICCQ */
    bool post_remote_event(const ICCQTargetEnum cpu, const EventEnum type, int8_t* const data, const uint32_t deadline = 0U);

    void register_iccq(IccqMessageHandler& h, const ICCQServiceEnum s);

    static ICCQTargetEnum get_queue_origination(const ICCQQueueEnum q);

    static ICCQTargetEnum get_queue_destination(const ICCQQueueEnum q);

    ICCQTargetEnum get_local_cpu() const;

private:

    class CommandQueueBase* cq;

    bool service_incoming(const ICCQQueueEnum q);

    ICCQTargetEnum localcpu;

    IccqMessageHandler* service[NUM_ICCQ_SERVICES];
};


class CommandQueueBase
{
public:

    virtual ~CommandQueueBase() {}

    virtual void setup_queues() = 0; // generally performed by HSM

    virtual bool write_message(const ICCQQueueEnum toq,
                               const ICCQTargetEnum tocpu, const ICCQServiceEnum toport,
                               const ICCQTargetEnum fromcpu, const ICCQServiceEnum fromport,
                               const int8_t* const iccq_payload, const size_t payload_size) = 0;

    virtual ICCQMessage* check_for_message(const ICCQQueueEnum q) = 0;

    virtual void advance_read_ptr(const ICCQQueueEnum q) = 0;

    void memory_barrier() const;

    enum { RING_SIZE = 16 };
};

CommandQueueBase* get_hardware_iccq_command_queue(void);
CommandQueueBase* get_simulated_hardware_iccq_command_queue(void);
CommandQueueBase* get_software_iccq_command_queue(void);

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ENG_ICCQ_H
