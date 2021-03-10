// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHMATHTYPES_H
#define SRS_HDR_UHMATHTYPES_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

#define _USE_MATH_DEFINES 1
#if SRS_PLATFORM_SOC
#include <math.h>
#include <float.h>
#else
#if SRS_PLATFORM_MSVC
#define _MATH_DEFINES_DEFINED 1
#if _MSC_VER <= 1600 // VS2010 fmin/fmax
#include <algorithm>
#endif
#endif
#if defined(__cplusplus)
#include <cmath>
#include <cfloat>
#else
#include <math.h>
#include <float.h>
#endif
#endif

#define uh_fabsf(x)  ::fabsf(x)
#define uh_fabsd(x)  ::fabs(x)
#define uh_roundf(x) ::roundf(x)
#define uh_floorf(x) ::floorf(x)
#define uh_ceilf(x)  ::ceilf(x)
#define uh_sqrtf(x)  ::sqrtf(x)
#define uh_atan2f(q,i) ::atan2f(q,i)
#define uh_cosf(x)   ::cosf(x)
#define uh_sinf(x)   ::sinf(x)
#define uh_asinf(x)  ::asinf(x)

#if SRS_PLATFORM_MSVC && (_MSC_VER <= 1600)     // VS2010
#define uh_fmaxf(x, y) max(x,y)
#define uh_fminf(x, y) min(x,y)
#else
#define uh_fmaxf(x, y) ::fmaxf(x,y)
#define uh_fminf(x, y) ::fminf(x,y)
#endif

#if SRS_PLATFORM_MSVC && (_MSC_VER <= 1500)     // VS2008
#define uh_hypotf(q,i) ::_hypotf(q,i)
#else
#define uh_hypotf(q,i) ::hypotf(q,i)
#endif

#define UH_ABS(x)    (((x) < 0) ? -(x) : (x))
#define UH_MAX(x, y) (((x) >= (y)) ? (x) : (y))
#define UH_MIN(x, y) (((x) <= (y)) ? (x) : (y))

#ifndef M_PI
#define M_PI        3.14159265358979323846264338327950288F   /* pi */
#endif

#ifndef M_2PI
#define M_2PI       (2.0F * (M_PI))                          /* (2 * pi) */
#endif
#define M_PI_DOUBLE 3.14159265358979323846264338327950288

#define UH_FLT_MAX   FLT_MAX
#define UH_MAX_INT16 (((1 << 15) - 1))

#define SPEED_OF_LIGHT (299792458.0F)

#define TWO_IN_DB  (6.020599913279624F)

// Useful trick to allow printing of a FLOAT as:   "%d.%02d"
// e.g.:   SystemLogger::instance().emit_warn(A, B, C, (uint32_t)x, TWO_FRACTIONAL_DIGITS(x));
#define TWO_FRACTIONAL_DIGITS_OF(x)   ((uint32_t)((x - (FLOAT)(uint32_t)x) * 100))
#define THREE_FRACTIONAL_DIGITS_OF(x) ((uint32_t)((x - (FLOAT)(uint32_t)x) * 1000))

#if SRS_PLATFORM_MSVC && (_MSC_VER < 1800)
static UHINLINE FLOAT acoshf(FLOAT x)
{
    return logf(x + sqrtf((x + 1.0F) * (x - 1.0F)));
}
#endif /* _MSC_VER < 1800 */

#endif // SRS_HDR_MATHTYPES_H
