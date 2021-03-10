#ifndef SRS_HDR_UHSTDARG_H
#define SRS_HDR_UHSTDARG_H 1

#include <stdarg.h>
#define uh_vsnprintf(str, size, fmt, ap) ::vsnprintf(str, size, fmt, ap)

#endif // SRS_HDR_UHSTDARG_H

