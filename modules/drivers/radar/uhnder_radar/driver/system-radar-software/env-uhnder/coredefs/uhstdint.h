#ifndef SRS_HDR_UHSTDINT_H
#define SRS_HDR_UHSTDINT_H 1

// the engine uses standard integer data types like uint32_t and int32_t. If
// stdint.h is not provided, you must define these data types

#include <stdint.h>
#if 0
typedef signed char            int8_t;
typedef unsigned char          uint8_t;
typedef signed short           int16_t;
typedef unsigned short         uint16_t;
typedef signed long int        int32_t;
typedef unsigned long int      uint32_t;
typedef signed long long int   int64_t;
typedef unsigned long long int uint64_t;
typedef int32_t                intptr_t;
typedef uint32_t               uintptr_t;
#endif // if 0

#if SRS_PLATFORM_SOC
#include <stddef.h>
#endif

#if SRS_PLATFORM_MSVC
// including inttypes.h makes MSVC very unhappy, so we must hack
#ifndef PRIu64
#define PRIu64 "I64u"
#endif
#else
#include <inttypes.h> // PRIu64, etc
#endif

// MISRA does not allow base types int, char, float to be used except within a
// typedef
typedef int32_t                INT;
typedef float                  FLOAT;
typedef char                   CHAR;


#if SRS_PLATFORM_GHS
#if !(defined(_BOOL))
    #define bool    _Bool
#endif
#endif

#endif // SRS_HDR_UHSTDINT_H
