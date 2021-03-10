// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHNDER_COMMON_H
#define SRS_HDR_UHNDER_COMMON_H 1

#if defined(__cplusplus)
#if defined(__SCP__)
#if defined(SRS_BOOT_IMAGE)
#define SRS_DECLARE_NAMESPACE() namespace SRS_B1_SCP {
#else
#define SRS_DECLARE_NAMESPACE() namespace SRS_SCP {
#endif
#define SRS_CLOSE_NAMESPACE()   } // end namespace SRS_SCP
#elif defined(__HSM__)
#define SRS_DECLARE_NAMESPACE() namespace SRS_HSM {
#define SRS_CLOSE_NAMESPACE()   } // end namespace SRS_HSM
#elif defined(__DSP__)
#define SRS_DECLARE_NAMESPACE() namespace SRS_DSP {
#define SRS_CLOSE_NAMESPACE()   } // end namespace SRS_DSP
#elif defined(__CCP__)
#define SRS_DECLARE_NAMESPACE() namespace SRS_CCP {
#define SRS_CLOSE_NAMESPACE()   } // end namespace SRS_CCP
#elif defined(__ADTF__)
#define SRS_DECLARE_NAMESPACE() namespace SRS_ADTF {
#define SRS_CLOSE_NAMESPACE()   } // end namespace SRS_ADTF
#else // C++ external use
#define SRS_DECLARE_NAMESPACE()
#define SRS_CLOSE_NAMESPACE()
#endif // if defined(__SCP__)
#else // !defined(__cplusplus)
#define SRS_DECLARE_NAMESPACE()
#define SRS_CLOSE_NAMESPACE()
#endif // if defined(__cplusplus)

#include "engine-defs.h"
#include "environment-defs.h"
#if __BARE_METAL__

#if defined(__ghs__)
#define SRS_PLATFORM_GHS        1
#elif defined(__GCCARM__)
#define SRS_PLATFORM_GCCARM     1
#elif defined(__ARMCOMPILER_VERSION) && __ARMCOMPILER_VERSION >= 6110004
#define SRS_PLATFORM_ARMCC6     1
#elif defined(__ARMCC_VERSION)
#define SRS_PLATFORM_ARMCC      1
#elif defined(__XTENSA__)
#define SRS_PLATFORM_XTENSA     1
#elif defined(__arm__) && defined(__linux__)
#define SRS_PLATFORM_PCLINT     1
#else
#error "Unknown bare metal compiler, update coredefs/uhnder-common.h"
#endif

#else // if __BARE_METAL__

#if defined(_WIN32)

#if defined(_MSC_VER)
#define SRS_PLATFORM_MSVC       1
#else // if defined(_MSC_VER)
#define SRS_PLATFORM_MINGW      1
#endif // if defined(_MSC_VER)

#else // if defined(_WIN32)

#if defined(__APPLE__)
#define SRS_PLATFORM_MAC        1
#else // if defined(__APPLE__)
#define SRS_PLATFORM_LINUX      1
#endif // if defined(__APPLE__)

#endif // if defined(_WIN32)

#endif // if __BARE_METAL__

#ifndef SRS_PLATFORM_GHS
#define SRS_PLATFORM_GHS 0
#endif
#ifndef SRS_PLATFORM_GCCARM
#define SRS_PLATFORM_GCCARM 0
#endif
#ifndef SRS_PLATFORM_ARMCC
#define SRS_PLATFORM_ARMCC 0
#endif
#ifndef SRS_PLATFORM_ARMCC6
#define SRS_PLATFORM_ARMCC6 0
#endif
#ifndef SRS_PLATFORM_XTENSA
#define SRS_PLATFORM_XTENSA 0
#endif
#ifndef SRS_PLATFORM_PCLINT
#define SRS_PLATFORM_PCLINT 0
#endif
#ifndef SRS_PLATFORM_MSVC
#define SRS_PLATFORM_MSVC 0
#endif
#ifndef SRS_PLATFORM_MINGW
#define SRS_PLATFORM_MINGW 0
#endif
#ifndef SRS_PLATFORM_MAC
#define SRS_PLATFORM_MAC 0
#endif
#ifndef SRS_PLATFORM_LINUX
#define SRS_PLATFORM_LINUX 0
#endif

#if __BARE_METAL__
#define SRS_PLATFORM_SOC 1
#else
#define SRS_PLATFORM_SOC 0
#endif

#if SRS_PLATFORM_MSVC || SRS_PLATFORM_MINGW
#define SRS_PLATFORM_WINDOWS 1
#else
#define SRS_PLATFORM_WINDOWS 0
#endif

#if SRS_PLATFORM_MAC || SRS_PLATFORM_LINUX
#define SRS_PLATFORM_POSIX 1
#else
#define SRS_PLATFORM_POSIX 0
#endif

#include "uhstdint.h"

#if defined(__cplusplus)
#include "uhstring.h"
#include "uhintrin.h"
#include "uhattrib.h"
#include "uhutils.h"
#endif

#endif // SRS_HDR_UHNDER_COMMON_H
