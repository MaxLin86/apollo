// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
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

#ifndef WITH_DIAGS
#define WITH_DIAGS            1    //!< Enables base diag manager
#endif

#ifndef WITH_ENV_DIAGS
#define WITH_ENV_DIAGS        1    //!< Enables environment defined diags ; env-reference/scp-src/diag
#endif

#ifndef WITH_ENGINE_DIAGS
#define WITH_ENGINE_DIAGS     1    //!< Enables engine (non-cal) diags    ; engine/scp-src/diag
#endif

#ifndef WITH_CAL_DIAGS
#define WITH_CAL_DIAGS        1    //!< Enables environment calibration diags  ; env-reference/scp-src/diag
#endif

#ifndef WITH_RRA_DIAGS
#define WITH_RRA_DIAGS        1    //!< Enables common diags used by Python RRA scripts
#endif

#ifndef WITH_MISC_DIAGS
#define WITH_MISC_DIAGS       1
#endif

#ifndef WITH_UNITTEST_DIAGS
#define WITH_UNITTEST_DIAGS   0
#endif

#ifndef WITH_SLT_DIAGS
#define WITH_SLT_DIAGS        1    //!< Enables SLT diags
#endif

#define WITH_UHDP             1    //!< Enables UhDP protocol stack       ; engine/scp-src/uhdp
#define WITH_HW_ERRATA        1    //!< Enables compilation of hardware errata table

#define DC_MEASURE_ADDITIONAL_SCAN_TIME (10*1024) //!< Additional time required for DC measure scan
                                                  //!< This will change based on compiler optmization settings

#if !defined(ALLOC_SDPD_IN_DDR)
#define ALLOC_SDPD_IN_DDR          0
#endif

#if !defined(SRS_HAVE_DDR)
#define SRS_HAVE_DDR          1
#endif

#define __DDR_933__           1
#define __DCU_933__           0

#if SRS_HAVE_DDR

#define SYS_SRAM_UNCACHED_SIZE  (128 * 1024)
#define TFS_SIZE_MB           64   //!< size in MB of trivial filesystem (in DDR)
#define TFS_MAX_NUM_FILES     128  //!< maximum number of files supported by TFS

#else

// NOTE: NO-DDR only supported for Sabine B
#define SYS_SRAM_UNCACHED_SIZE  (512 * 1024)
#define TFS_SIZE_KB           256 //!< size in KB of trivial filesystem (in cached SRAM)
#define TFS_MAX_NUM_FILES     32  //!< maximum number of files supported by TFS

#endif

#define ENABLE_TFTPD          1             //!< enable/disable TFTPD at compile time
#define ENABLE_SEQ_LOGGER     SRS_HAVE_DDR  //!< enable/disable sequential debug logger at compile time
#define LIMIT_LOGGING         0             //!< enable/disable non-critical logs at compile time
#define DISABLE_PROFILING     0             //!< enable/disable profiling scope events at compile time

//! the engine will register these UDP ports with the protocol stack at init
#define UHDP_PORT             9933 //!< Uhnder internal datagram protocol for control/capture
#define TFTP_PORT             9934 //!< TFTPD server port, non-standard port for simulations
#define TFTP_PORT_IETF        69   //!< TFTPD server port, standard specified by the IETF

//! select which DSPs are used for scan post-processing tasks
#define ZERO_DOPPLER_DSP_SELECT  ICCQ_TARGET_DSP1
#define HISTOGRAM_DSP_SELECT     ICCQ_TARGET_DSP1
#define EGO_VELOCITY_DSP_SELECT  ICCQ_TARGET_DSP1
#define CLUTTER_IMAGE_DSP_SELECT ICCQ_TARGET_DSP1
#define POINT_CLOUD_DSP_SELECT   ICCQ_TARGET_DSP1
#define DETECTIONS_DSP_SELECT    ICCQ_TARGET_DSP1

#define NUM_FLASH_PARTITIONS 5

// Pack long-word offset, bitfield offset, and bitfield width into 32-bit long-word
// 31:10    Address offset is RGC addressing, in uint32_t steps (address / 4).
//          NOTE: The long-word align and bitshift are done by masking off the bottom two bits and shifting 10-2 bits
// 9:5      Bit-field offset.
// 4:0      Bit-field width.  Value of 0 indicates 32-bits wide.
// packed_write(..) writes given value into the register/field in the RGC Cache
#define PACKED_RMW(cache,addr,offset,width,val) cache.packed_write(((addr&0x00fffffc)<<8)|((offset&0x1f)<<5)|(width&0x1f),val)
// packed_read(..) reads register/field from the RGC Cache
#define PACKED_RO(cache,addr,offset,width,val)  cache.packed_read( ((addr&0x00fffffc)<<8)|((offset&0x1f)<<5)|(width&0x1f),val)

// Pack reg address offset, bitfield offset, and bitfield width into 32-bit long-word
// 31:10    Address offset is the register address offset for directly writing / reading from the HW
// 9:5      Bit-field offset.
// 4:0      Bit-field width.  Value of 0 indicates 32-bits wide.
// packed_status(..) reads register/field directly from HW
#define PACKED_STATUS(addr,offset,width,val)    packed_status((addr<<10)|((offset&0x1f)<<5)|(width&0x1f),val)
// packed_direct_write(..) writes given value in the register/field directly in HW
#define PACKED_DIRECT_RMW(addr,offset,width,val) packed_direct_write((addr<<10)|((offset&0x1f)<<5)|(width&0x1f), val)

#endif // ifndef COREDEFS_ENGINE_DEFS
