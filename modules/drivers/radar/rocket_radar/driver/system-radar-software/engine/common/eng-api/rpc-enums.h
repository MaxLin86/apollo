// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RPC_ENUMS_H
#define SRS_HDR_RPC_ENUMS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

enum RPCFunctionEnum
{
#define DEFINE_RPC(name) name,
#if SRS_BOOT_IMAGE_1
#include "boot-rpcdef.inc"
#else
#include "engine-rpcdef.inc"
#include "environment-rpcdef.inc"
#endif
#undef DEFINE_RPC
    NUM_RPC_FUNCTIONS
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RPC_ENUMS_H
