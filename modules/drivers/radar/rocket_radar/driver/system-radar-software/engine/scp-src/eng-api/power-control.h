// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_POWER_CONTROL_H
#define SRS_HDR_POWER_CONTROL_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "moduleconfig.h"

SRS_DECLARE_NAMESPACE()


enum SCPPowerState       // Power states of the SCP processor(s)
{
    SCP_POWER_CLK_LO,
    SCP_POWER_NOMINAL,
};

enum NOCPowerState       // Power states of the NOC (Network on Chip)
{
    NOC_POWER_CLK_LO,
    NOC_POWER_NOMINAL,
};


class PowerControl
{
public:

    static PowerControl&    instance();

    void                    init();

    void                    scp_power_control(SCPPowerState pwr_state);

    void                    noc_power_control(NOCPowerState pwr_state);

private:

    SCPPowerState               scp_power_state;            // Power state of the SCP processor(s)
    NOCPowerState               noc_power_state;            // Power state of the NOC

    bool                        scp_noc_power_control_mode; // If enabled, allow switching to lower power states
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_POWER_CONTROL_H
