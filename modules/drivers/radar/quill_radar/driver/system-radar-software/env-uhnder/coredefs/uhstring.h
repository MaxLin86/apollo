#ifndef SRS_HDR_UHSTRING_H
#define SRS_HDR_UHSTRING_H 1

// each environment can choose to include their system header and/or define
// their own implementations
#if SRS_PLATFORM_SOC
#include <string.h>
#include <stdio.h>
#else
#include <cstring>
#include <cstdio>
#endif

// specify which implementations are used by the engine

#if SRS_PLATFORM_SOC

// NOTE: uh_memcpy() must use 32bit writes when the source, dest, and length are
// 32bit aligned.

    #if SRS_BOOT_IMAGE
        // boot libraries provide memcpy internally
        #define uh_memcpy(dst, src, n)           ::memcpy(dst, src, n)
    #elif __DSP__
        // the XTensa clibrary provides a functional memcpy
        #define uh_memcpy(dst, src, n)           ::memcpy(dst, src, n)
    #elif SRS_PLATFORM_GHS
        // RoLi -> use memcpy
        #define uh_memcpy(dst, src, n)           ::memcpy(dst, src, n)
    #else
        // for the remaining ARM codes, use our libc.cpp implementation
        #define uh_memcpy(dst, src, n)           uh_env_memcpy(dst, src, n)
    #endif

#define uh_strcasecmp(s1, s2)            strcasecmp(s1, s2)
#define uh_snprintf(str, size, fmt, ...) snprintf(str, size, fmt, __VA_ARGS__)

SRS_DECLARE_NAMESPACE()
size_t   uh_strnlen(const char* ptr, size_t maxlen);
void*    uh_env_memcpy(void *dest, const void *src, size_t n); // libc.cpp
SRS_CLOSE_NAMESPACE()

#else // if SRS_PLATFORM_SOC  (simulation)

    #define uh_memcpy(dst, src, n)           ::memcpy(dst, src, n)
    #define uh_strnlen(str, maxlen)          ::strnlen(str, maxlen)

    #if SRS_PLATFORM_WINDOWS
        #define __func__ __FUNCTION__
        #define uh_strcasecmp(s1, s2)            ::_stricmp(s1, s2)
        #define uh_snprintf(str, size, fmt, ...) ::_snprintf(str, size, fmt, __VA_ARGS__)
    #else
        #define uh_strcasecmp(s1, s2)            ::strcasecmp(s1, s2)
        #define uh_snprintf(str, size, fmt, ...) ::snprintf(str, size, fmt, __VA_ARGS__)
    #endif

#endif  /* simulation */

#if SRS_PLATFORM_GHS
    #if defined(__ARM__)
        #include <arm_ghs.h>
    #endif
    #ifndef __func__
        #define __func__ __FUNCTION__
    #endif
#endif  /* SRS_PLATFORM_GHS */

#endif // SRS_HDR_UHSTRING_H
