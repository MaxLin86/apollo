// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHUNISTD_H
#define SRS_HDR_UHUNISTD_H 1

#if SRS_PLATFORM_WINDOWS

#include <fcntl.h>
#include <io.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <process.h>    // for _getpid
#include <Windows.h>

#define uh_getpid()    _getpid()
#define uh_usleep(x)    ::Sleep(((x) + 999) / 1000)

#elif SRS_PLATFORM_SOC

#define uh_usleep(count) uh_wait_us(count)
SRS_DECLARE_NAMESPACE()
void uh_wait_us(const uint32_t t); // cru_pin.cpp
SRS_CLOSE_NAMESPACE()

#else

#include <unistd.h>
#include <pthread.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/stat.h>

#define uh_usleep(count) ::usleep(count)
#define uh_getpid()      getpid()


#endif

#endif // SRS_HDR_UHUNISTD_H
