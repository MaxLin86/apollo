// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_REMOTELOG_ICCQ_H
#define SRS_HDR_REMOTELOG_ICCQ_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()


#if SRS_HAVE_DDR
enum { NUM_REMOTE_BUFFERS = 6 };

#else
enum { NUM_REMOTE_BUFFERS = 2 };

#endif

/* ICCQ messages exchanged between remote CPUs and SCC Logging Agent */
struct IccqMessageToRemote
{
    uint32_t buffer_size;
    uint32_t num_buffers;
    uint8_t* buffer[NUM_REMOTE_BUFFERS];
};

struct IccqMessageFromRemote
{
    uint32_t buffer_length;
    uint8_t* buffer;
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_REMOTELOG_ICCQ_H
