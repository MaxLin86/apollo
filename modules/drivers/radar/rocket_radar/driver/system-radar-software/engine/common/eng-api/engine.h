// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_ENGINE_H
#define SRS_HDR_ENGINE_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "event-handler.h"
#if __SCP__ && !SRS_BOOT_IMAGE
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#endif

SRS_DECLARE_NAMESPACE()

enum EngineLogSubunits
{
    ENG_EVENT_HANDLER,
    ENG_STATE_MANAGER,
    ENG_LOGGER,
    ENG_RPC,
    ENG_IOVEC,
    ENG_DEVMM,
    ENG_TFS,
    ENG_TFTP,
    NUM_ENG_SUBUNITS
};

class Engine
#if __SCP__ && !SRS_BOOT_IMAGE
    : public LogProducer
#endif
{
public:

    UHVIRTDESTRUCT(Engine)
    static Engine& instance();

    void init();

    void shutdown() { /* TBD */ }

    void mainloop_poll();

#if __SCP__ && !SRS_BOOT_IMAGE
    static const CHAR* subunit_names[NUM_ENG_SUBUNITS];

    virtual const CHAR* get_subunit_name(uint32_t subunit) const;

    virtual uint32_t get_num_subunits() const { return NUM_ENG_SUBUNITS; }

#if WITH_UHDP
    // if the radar receives an ICMP destination unreachable message, this
    // function should be called with the IP address and UDP port of the
    // returned message header (after the ICMP header) in network order These
    // should be the IP and port of the UDP packet that triggered the ICMP
    // response
    void icmp_destination_unreachable(uint32_t dest_ip, uint16_t dest_port);
#endif

#else
    uint32_t data; // this class needs at least one member to declare as a singleton
#endif
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ENGINE_H
