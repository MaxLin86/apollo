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
#pragma once
#ifndef COREDEFS_ENGINE_DEFS
#define COREDEFS_ENGINE_DEFS 1

/* this file should only contain C macro #define statements. It is included by
 * linker scripts and other things that cannot parse anything else */

// Ensure these compiler symbols are defined, for automotive safety spec
#if !defined(__SCP__)
#define __SCP__ 0
#endif
#if !defined(__HSM__)
#define __HSM__ 0
#endif
#if !defined(__DSP__)
#define __DSP__ 0
#endif
#if !defined(__CCP__)
#define __CCP__ 0
#endif
#if !defined(__BARE_METAL__)
#define __BARE_METAL__ 0
#endif
#if !defined(__EN_RHAL_HOOKS__)
#define __EN_RHAL_HOOKS__ 0
#endif
#if !defined(GTEST_ENABLED)
#define GTEST_ENABLED 0
#endif
#if !defined(SRS_BOOT_IMAGE)
#define SRS_BOOT_IMAGE 0
#endif
#if !defined(SRS_BOOT_IMAGE_0)
#define SRS_BOOT_IMAGE_0 0
#endif
#if !defined(SRS_BOOT_IMAGE_1)
#define SRS_BOOT_IMAGE_1 0
#endif

#if SRS_BOOT_IMAGE
#define ENABLE_UHPRINTF       0
#else
#define ENABLE_UHPRINTF       1
#endif

#define WITH_DIAGS            1    //!< Enables base diag manager
#define WITH_ENV_DIAGS        1    //!< Enables environment defined diags ; env-uhnder/scp-src/diag
#define WITH_ENGINE_DIAGS     1    //!< Enables engine (non-cal) diags    ; engine/scp-src/diag
#define WITH_CAL_DIAGS        1    //!< Enables engine calibration diags  ; engine/scp-src/diag
#define WITH_UHDP             1    //!< Enables UhDP protocol stack       ; engine/scp-src/uhdp

#define DC_MEASURE_ADDITIONAL_SCAN_TIME (10*1024) //!< Additional time required for DC measure scan
                                                  //!< This will change based on compiler optmization settings

#if !defined(SRS_HAVE_DDR)
#define SRS_HAVE_DDR          1
#endif

#define __DDR_933__           1
#define __DCU_933__           0

#if SRS_HAVE_DDR
#define SYS_SRAM_UNCACHED_SIZE  (64 * 1024)
#define TFS_SIZE_MB           12   //!< size in MB of trivial filesystem (in DDR)
#define TFS_MAX_NUM_FILES     64   //!< maximum number of files supported by TFS
#else
// NOTE: NO-DDR only supported for Sabine B
#define SYS_SRAM_UNCACHED_SIZE  (512 * 1024)
#define TFS_SIZE_MB           1   //!< size in MB of trivial filesystem (in DDR)
#define TFS_SIZE_KB           512   //!< size in KB of trivial filesystem (in cached SRAM)
#define TFS_MAX_NUM_FILES     10  //!< maximum number of files supported by TFS
#endif

#define ENABLE_TFTPD          1    //!< disable TFTPD at compile time
#define LIMIT_LOGGING         0    //!< disable non-critical logs at compile time
#define DISABLE_PROFILING     0    //!< disable profiling scope events at compile time

//! the engine will register these UDP ports with the protocol stack at init
#define UHDP_PORT             9933 //!< Uhnder internal datagram protocol for control/capture
#define TFTP_PORT             9934 //!< TFTPD server port, non-standard port for simulations
#define TFTP_PORT_IETF        69   //!< TFTPD server port, standard specified by the IETF

#endif // ifndef COREDEFS_ENGINE_DEFS
