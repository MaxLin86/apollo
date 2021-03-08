#ifndef SRS_HDR_UHNDER_COMMON_H
#define SRS_HDR_UHNDER_COMMON_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2017 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE

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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/engine-defs.h"
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
#error "Unknown bare metal compiler, update coredefs/modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
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
