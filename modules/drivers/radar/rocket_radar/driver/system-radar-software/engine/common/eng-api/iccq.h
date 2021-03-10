// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_ENG_ICCQ_H
#define SRS_HDR_ENG_ICCQ_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhnder-helpers.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/iccq-memory.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"

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

    UHVIRTDESTRUCT(InterCoreCommQueue)
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
