#ifndef SRS_HDR_UHSTACKTRACE_H
#define SRS_HDR_UHSTACKTRACE_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

#if SRS_PLATFORM_POSIX && defined(__GNUC__)

#include <execinfo.h>
#include <cxxabi.h>
#define UH_STACK_TRACE_AVAILABLE 1

#else

#define UH_STACK_TRACE_AVAILABLE 0

#endif


#endif // SRS_HDR_UHSTACKTRACE_H
