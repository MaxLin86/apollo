// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHDP_AGENT_H
#define SRS_HDR_UHDP_AGENT_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

#if WITH_UHDP

struct UhdpCaptureControl;
struct RDC_AprobeRequest;
struct RDC_ThresholdControl;

class UhdpAgent
{
public:

    UHVIRTDESTRUCT(UhdpAgent)
    static UhdpAgent& instance();

    virtual void init_logger() {}

    virtual void init_net() {}

    virtual void init_radar_data() {}

    virtual void icmp_destination_unreachable(uint32_t dest_ip, uint16_t dest_port) {}

    virtual void disconnect_peer() {}

    virtual void set_capture_control(const UhdpCaptureControl& capctrl) {}

    virtual void get_capture_control(UhdpCaptureControl& capctrl) const {}

    virtual void get_threshold_control(uint32_t sc_index, RDC_ThresholdControl& threshctrl) const {}

    virtual bool scan_aprobes_clear(int16_t index) { return false; };

    virtual void scan_aprobes_list() {};

    virtual bool scan_aprobes_add(const RDC_AprobeRequest& ap) { return false; };
};

#endif

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHDP_AGENT_H
