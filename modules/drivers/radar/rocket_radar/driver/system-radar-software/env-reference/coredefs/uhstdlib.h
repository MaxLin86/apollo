// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHSTDLIB_H
#define SRS_HDR_UHSTDLIB_H 1

#if SRS_PLATFORM_SOC
#include <stdlib.h>
#else
#include <cstdlib>
#endif

#define uh_strtoul ::strtoul
#define uh_getenv  ::getenv
#define uh_rand    ::rand
#define uh_atoi    ::atoi

#endif // SRS_HDR_UHSTDLIB_H
