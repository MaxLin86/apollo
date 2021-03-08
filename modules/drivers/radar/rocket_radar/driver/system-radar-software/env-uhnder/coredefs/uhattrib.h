#ifndef SRS_HDR_UHSTRUCTPACK_H
#define SRS_HDR_UHSTRUCTPACK_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

#if SRS_PLATFORM_MSVC

// MSVC style
#define CHECK_PRINTF_ARGS(string_index,first_to_check)
#define PRAGMA_PRINTF_ARGS()
#define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#define ALIGN_VAR_4(T, var)  __declspec(align(4)) T var
#define ALIGN_VAR_8(T, var)  __declspec(align(8)) T var
#define ALIGN_VAR_16(T, var) __declspec(align(16)) T var
#define ALIGN_VAR_32(T, var) __declspec(align(32)) T var
#define ALIGN_VAR_64(T, var) __declspec(align(64)) T var
#define UHINLINE inline
#define DEBUG_BREAK()        __debugbreak()

#elif SRS_PLATFORM_GHS

// Greenhills
#define CHECK_PRINTF_ARGS(string_index,first_to_check)
#define PRAGMA_PRINTF_ARGS() _Pragma("__printf_args")
#define PRAGMA(x)           _Pragma(#x)
#define ALIGNVAR(align)     PRAGMA( alignvar(align) )
#define DECL_ALIGN_BUF(name, type, size, align, sect) \
ALIGNVAR(align) \
CHANGE_SEC(bss, sect) \
type name[size]; \
CHANGE_SEC(bss, default)
#define UHINLINE inline

#define PACK( arg ) arg __attribute__((packed))
#define ALIGN_VAR_4(T, var)  ALIGNVAR(4)    T var
#define ALIGN_VAR_8(T, var)  ALIGNVAR(8)    T var
#define ALIGN_VAR_16(T, var) ALIGNVAR(16)   T var
#define ALIGN_VAR_32(T, var) ALIGNVAR(32)   T var
#define ALIGN_VAR_64(T, var) ALIGNVAR(64)   T var

#else

#if defined(__clang__)
#define DEBUG_BREAK()  __builtin_debugtrap()
#elif defined(__GNUC__)
#define DEBUG_BREAK()  __asm__ volatile("int $0x03")
#endif

#define CHECK_PRINTF_ARGS(string_index,first_to_check) __attribute__((format (printf, string_index, first_to_check)))
#define PRAGMA_PRINTF_ARGS()
#define PACK(__Declaration__) __Declaration__ __attribute((__packed__))
#define ALIGN_VAR_4(T, var)  T var __attribute__((aligned(4)))
#define ALIGN_VAR_8(T, var)  T var __attribute__((aligned(8)))
#define ALIGN_VAR_16(T, var) T var __attribute__((aligned(16)))
#define ALIGN_VAR_32(T, var) T var __attribute__((aligned(32)))
#define ALIGN_VAR_64(T, var) T var __attribute__((aligned(64)))
#define UHINLINE inline

#endif

// Define how your P5 (DSP) compiler specifies 'restrict' for pointers
#if SRS_PLATFORM_XTENSA
#define UH_DSP_RESTRICT __restrict
#else
#define UH_DSP_RESTRICT
#endif

#endif // if SRS_HDR_UHSTRUCTPACK_H

