// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHSTACKTRACE_H
#define SRS_HDR_UHSTACKTRACE_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

#if SRS_PLATFORM_POSIX && defined(__GNUC__)

#include <execinfo.h>
#include <cxxabi.h>
#define UH_STACK_TRACE_AVAILABLE 1

#else

#define UH_STACK_TRACE_AVAILABLE 0

#endif


#endif // SRS_HDR_UHSTACKTRACE_H
