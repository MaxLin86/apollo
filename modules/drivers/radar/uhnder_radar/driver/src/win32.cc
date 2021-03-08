#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/sra.h"

#ifdef _WIN32

#include <time.h>
#include <iomanip>
#include <sstream>

int strncasecmp(char *s1, char *s2, size_t n)
{
    if (n == 0)
    {
        return 0;
    }

    while (n-- != 0 && tolower(*s1) == tolower(*s2))
    {
        if ((n == 0) || (*s1 == '\0') || (*s2 == '\0'))
        {
            break;
        }
        s1++;
        s2++;
    }

    return tolower(*(unsigned char*)s1) - tolower(*(unsigned char*)s2);
}


#if defined(_MSC_VER)
int poll(struct pollfd *pfd, int nfds, int timeout)
{
    return WSAPoll(pfd, nfds, timeout);
}


int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
    // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
    // This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
    // until 00:00:00 January 1, 1970
    static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time;

    GetSystemTime(&system_time);
    SystemTimeToFileTime(&system_time, &file_time);
    time = ((uint64_t)file_time.dwLowDateTime);
    time += ((uint64_t)file_time.dwHighDateTime) << 32;

    tp->tv_sec = (long)((time - EPOCH) / 10000000L);
    tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
    return 0;
}


char * strptime(const char *buf, const char *fmt, struct tm *tm)
{
    std::istringstream input(buf);
    input.imbue(std::locale(setlocale(LC_ALL, nullptr)));
    input >> std::get_time(tm, fmt);
    if (input.fail())
    {
        return nullptr;
    }
    return (char*)buf + input.tellg();
}


#else // if defined(_MSC_VER)

/* MINGW compatible implementation of strptime */
typedef unsigned char u_char;
typedef unsigned int uint;
typedef unsigned __int64 uint64_t;

#define _ctloc(x)       (_CurrentTimeLocale->x)

/*
 * We do not implement alternate representations. However, we always
 * check whether a given modifier is allowed for a certain conversion.
 */
#define ALT_E           0x01
#define ALT_O           0x02
#define LEGAL_ALT(x)        { if (alt_format & ~(x)) { return NULL; } }

static int TM_YEAR_BASE = 1900;
static char gmt[] = { "GMT" };
static char utc[] = { "UTC" };
/* RFC-822/RFC-2822 */
static const char * const nast[5] = {
    "EST",    "CST",    "MST",    "PST",    "\0\0\0"
};
static const char * const nadt[5] = {
    "EDT",    "CDT",    "MDT",    "PDT",    "\0\0\0"
};
static const char * const am_pm[2] = {
    "am", "pm"
};
static const char * const day[7] = {
    "sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday"
};
static const char * const abday[7] = {
    "sun", "mon", "tue", "wed", "thu", "fri", "sat"
};
static const char * const mon[12] = {
    "january", "february", "march", "april", "may", "june", "july", "august", "september", "october", "november", "december"
};
static const char * const abmon[12] = {
    "jan", "feb", "mar", "apr", "may", "jun", "jul", "aug", "sep", "oct", "nov", "dec"
};

static const u_char *conv_num(const unsigned char *, int *, uint, uint);
static const u_char *find_string(const u_char *, int *, const char * const *,
                                 const char * const *, int);

char *strptime(const char *buf, const char *fmt, struct tm *tm)
{
    unsigned char c;
    const unsigned char *bp, *ep;
    int alt_format, i, split_year = 0, neg = 0, offs;
    const char *new_fmt;

    bp = (const u_char*)buf;

    while (bp != NULL && (c = *fmt++) != '\0')
    {
        /* Clear `alternate' modifier prior to new conversion. */
        alt_format = 0;
        i = 0;

        /* Eat up white-space. */
        if (isspace(c))
        {
            while (isspace(*bp))
            {
                bp++;
            }

            continue;
        }

        if (c != '%')
        {
            goto literal;
        }

again:  switch (c = *fmt++)
        {
        case '%':   /* "%%" is converted to "%". */
literal:
            if (c != *bp++)
            {
                return NULL;
            }
            LEGAL_ALT(0);
            continue;

        /*
         * "Alternative" modifiers. Just set the appropriate flag
         * and start over again.
         */
        case 'E':   /* "%E?" alternative conversion modifier. */
            LEGAL_ALT(0);
            alt_format |= ALT_E;
            goto again;

        case 'O':   /* "%O?" alternative conversion modifier. */
            LEGAL_ALT(0);
            alt_format |= ALT_O;
            goto again;

        /*
         * "Complex" conversion rules, implemented through recursion.
         */

        /* we do not need 'c'
      case 'c': Date and time, using the locale's format.
            new_fmt = _ctloc(d_t_fmt);
            goto recurse;
      */

        case 'D':   /* The date as "%m/%d/%y". */
            new_fmt = "%m/%d/%y";
            LEGAL_ALT(0);
            goto recurse;

        case 'F':   /* The date as "%Y-%m-%d". */
            new_fmt = "%Y-%m-%d";
            LEGAL_ALT(0);
            goto recurse;

        case 'R':   /* The time as "%H:%M". */
            new_fmt = "%H:%M";
            LEGAL_ALT(0);
            goto recurse;

        case 'r':   /* The time in 12-hour clock representation. */
            new_fmt = "%I:%M:S %p"; //_ctloc(t_fmt_ampm);
            LEGAL_ALT(0);
            goto recurse;

        case 'T':   /* The time as "%H:%M:%S". */
            new_fmt = "%H:%M:%S";
            LEGAL_ALT(0);
            goto recurse;

            /* we don't use 'X'
          case 'X': The time, using the locale's format.
                new_fmt =_ctloc(t_fmt);
                goto recurse;
          */

            /* we do not need 'x'
          case 'x': The date, using the locale's format.
                new_fmt =_ctloc(d_fmt);*/
recurse:
            bp = (const u_char*)strptime((const char*)bp,
                                         new_fmt, tm);
            LEGAL_ALT(ALT_E);
            continue;

        /*
         * "Elementary" conversion rules.
         */
        case 'A':   /* The day of week, using the locale's form. */
        case 'a':
            bp = find_string(bp, &tm->tm_wday, day, abday, 7);
            LEGAL_ALT(0);
            continue;

        case 'B':   /* The month, using the locale's form. */
        case 'b':
        case 'h':
            bp = find_string(bp, &tm->tm_mon, mon, abmon, 12);
            LEGAL_ALT(0);
            continue;

        case 'C':   /* The century number. */
            i = 20;
            bp = conv_num(bp, &i, 0, 99);

            i = i * 100 - TM_YEAR_BASE;
            if (split_year)
            {
                i += tm->tm_year % 100;
            }
            split_year = 1;
            tm->tm_year = i;
            LEGAL_ALT(ALT_E);
            continue;

        case 'd':   /* The day of month. */
        case 'e':
            bp = conv_num(bp, &tm->tm_mday, 1, 31);
            LEGAL_ALT(ALT_O);
            continue;

        case 'k':   /* The hour (24-hour clock representation). */
            LEGAL_ALT(0);
        /* FALLTHROUGH */
        case 'H':
            bp = conv_num(bp, &tm->tm_hour, 0, 23);
            LEGAL_ALT(ALT_O);
            continue;

        case 'l':   /* The hour (12-hour clock representation). */
            LEGAL_ALT(0);
        /* FALLTHROUGH */
        case 'I':
            bp = conv_num(bp, &tm->tm_hour, 1, 12);
            if (tm->tm_hour == 12)
            {
                tm->tm_hour = 0;
            }
            LEGAL_ALT(ALT_O);
            continue;

        case 'j':   /* The day of year. */
            i = 1;
            bp = conv_num(bp, &i, 1, 366);
            tm->tm_yday = i - 1;
            LEGAL_ALT(0);
            continue;

        case 'M':   /* The minute. */
            bp = conv_num(bp, &tm->tm_min, 0, 59);
            LEGAL_ALT(ALT_O);
            continue;

        case 'm':   /* The month. */
            i = 1;
            bp = conv_num(bp, &i, 1, 12);
            tm->tm_mon = i - 1;
            LEGAL_ALT(ALT_O);
            continue;

        case 'p':   /* The locale's equivalent of AM/PM. */
            bp = find_string(bp, &i, am_pm, NULL, 2);
            if (tm->tm_hour > 11)
            {
                return NULL;
            }
            tm->tm_hour += i * 12;
            LEGAL_ALT(0);
            continue;

        case 'S':   /* The seconds. */
            bp = conv_num(bp, &tm->tm_sec, 0, 61);
            LEGAL_ALT(ALT_O);
            continue;

#ifndef TIME_MAX
#define TIME_MAX    INT64_MAX
#endif
        case 's':   /* seconds since the epoch */
        {
            time_t sse = 0;
            uint64_t rulim = TIME_MAX;

            if ((*bp < '0') || (*bp > '9'))
            {
                bp = NULL;
                continue;
            }

            do
            {
                sse *= 10;
                sse += *bp++ - '0';
                rulim /= 10;
            }
            while ((sse * 10 <= TIME_MAX) &&
                   rulim && *bp >= '0' && *bp <= '9');

            if ((sse < 0) || ((uint64_t)sse > TIME_MAX))
            {
                bp = NULL;
                continue;
            }

            tm = localtime(&sse);
            if (tm == NULL)
            {
                bp = NULL;
            }
        }
            continue;

        case 'U':   /* The week of year, beginning on sunday. */
        case 'W':   /* The week of year, beginning on monday. */

            /*
             * XXX This is bogus, as we can not assume any valid
             * information present in the tm structure at this
             * point to calculate a real value, so just check the
             * range for now.
             */
            bp = conv_num(bp, &i, 0, 53);
            LEGAL_ALT(ALT_O);
            continue;

        case 'w':   /* The day of week, beginning on sunday. */
            bp = conv_num(bp, &tm->tm_wday, 0, 6);
            LEGAL_ALT(ALT_O);
            continue;

        case 'u':   /* The day of week, monday = 1. */
            bp = conv_num(bp, &i, 1, 7);
            tm->tm_wday = i % 7;
            LEGAL_ALT(ALT_O);
            continue;

        case 'g':   /* The year corresponding to the ISO week
                 * number but without the century.
                 */
            bp = conv_num(bp, &i, 0, 99);
            continue;

        case 'G':   /* The year corresponding to the ISO week
                 * number with century.
                 */
            do
            {
                bp++;
            }
            while (isdigit(*bp));
            continue;

        case 'V':   /* The ISO 8601:1988 week number as decimal */
            bp = conv_num(bp, &i, 0, 53);
            continue;

        case 'Y':   /* The year. */
            i = TM_YEAR_BASE;   /* just for data sanity... */
            bp = conv_num(bp, &i, 0, 9999);
            tm->tm_year = i - TM_YEAR_BASE;
            LEGAL_ALT(ALT_E);
            continue;

        case 'y':   /* The year within 100 years of the epoch. */
            /* LEGAL_ALT(ALT_E | ALT_O); */
            bp = conv_num(bp, &i, 0, 99);

            if (split_year)
            {
                /* preserve century */
                i += (tm->tm_year / 100) * 100;
            }
            else
            {
                split_year = 1;
                if (i <= 68)
                {
                    i = i + 2000 - TM_YEAR_BASE;
                }
                else
                {
                    i = i + 1900 - TM_YEAR_BASE;
                }
            }
            tm->tm_year = i;
            continue;

        case 'Z':
            _tzset();
            if ((strncasecmp((const char*)bp, gmt, 3) == 0)
                || (strncasecmp((const char*)bp, utc, 3) == 0))
            {
                tm->tm_isdst = 0;
#ifdef TM_GMTOFF
                tm->TM_GMTOFF = 0;
#endif
#ifdef TM_ZONE
                tm->TM_ZONE = gmt;
#endif
                bp += 3;
            }
            else
            {
                ep = find_string(bp, &i,
                                 (const char*const*)tzname,
                                 NULL, 2);
                if (ep != NULL)
                {
                    tm->tm_isdst = i;
#ifdef TM_GMTOFF
                    tm->TM_GMTOFF = -(timezone);
#endif
#ifdef TM_ZONE
                    tm->TM_ZONE = tzname[i];
#endif
                }
                bp = ep;
            }
            continue;

        case 'z':

            /*
             * We recognize all ISO 8601 formats:
             * Z	= Zulu time/UTC
             * [+-]hhmm
             * [+-]hh:mm
             * [+-]hh
             * We recognize all RFC-822/RFC-2822 formats:
             * UT|GMT
             *          North American : UTC offsets
             * E[DS]T = Eastern : -4 | -5
             * C[DS]T = Central : -5 | -6
             * M[DS]T = Mountain: -6 | -7
             * P[DS]T = Pacific : -7 | -8
             *          Military
             * [A-IL-M] = -1 ... -9 (J not used)
             * [N-Y]  = +1 ... +12
             */
            while (isspace(*bp))
            {
                bp++;
            }

            switch (*bp++)
            {
            case 'G':
                if (*bp++ != 'M')
                {
                    return NULL;
                }
            /*FALLTHROUGH*/
            case 'U':
                if (*bp++ != 'T')
                {
                    return NULL;
                }
            /*FALLTHROUGH*/
            case 'Z':
                tm->tm_isdst = 0;
#ifdef TM_GMTOFF
                tm->TM_GMTOFF = 0;
#endif
#ifdef TM_ZONE
                tm->TM_ZONE = utc;
#endif
                continue;
            case '+':
                neg = 0;
                break;
            case '-':
                neg = 1;
                break;
            default:
                --bp;
                ep = find_string(bp, &i, nast, NULL, 4);
                if (ep != NULL)
                {
#ifdef TM_GMTOFF
                    tm->TM_GMTOFF = -5 - i;
#endif
#ifdef TM_ZONE
                    tm->TM_ZONE = __UNCONST(nast[i]);
#endif
                    bp = ep;
                    continue;
                }
                ep = find_string(bp, &i, nadt, NULL, 4);
                if (ep != NULL)
                {
                    tm->tm_isdst = 1;
#ifdef TM_GMTOFF
                    tm->TM_GMTOFF = -4 - i;
#endif
#ifdef TM_ZONE
                    tm->TM_ZONE = __UNCONST(nadt[i]);
#endif
                    bp = ep;
                    continue;
                }

                if (((*bp >= 'A') && (*bp <= 'I')) ||
                    ((*bp >= 'L') && (*bp <= 'Y')))
                {
#ifdef TM_GMTOFF
                    /* Argh! No 'J'! */
                    if ((*bp >= 'A') && (*bp <= 'I'))
                    {
                        tm->TM_GMTOFF =
                            ('A' - 1) - (int)*bp;
                    }
                    else if ((*bp >= 'L') && (*bp <= 'M'))
                    {
                        tm->TM_GMTOFF = 'A' - (int)*bp;
                    }
                    else if ((*bp >= 'N') && (*bp <= 'Y'))
                    {
                        tm->TM_GMTOFF = (int)*bp - 'M';
                    }
#endif // ifdef TM_GMTOFF
#ifdef TM_ZONE
                    tm->TM_ZONE = NULL; /* XXX */
#endif
                    bp++;
                    continue;
                }
                return NULL;
            }

            offs = 0;
            for (i = 0; i < 4;)
            {
                if (isdigit(*bp))
                {
                    offs = offs * 10 + (*bp++ - '0');
                    i++;
                    continue;
                }
                if ((i == 2) && (*bp == ':'))
                {
                    bp++;
                    continue;
                }
                break;
            }

            switch (i)
            {
            case 2:
                offs *= 100;
                break;
            case 4:
                i = offs % 100;
                if (i >= 60)
                {
                    return NULL;
                }
                /* Convert minutes into decimal */
                offs = (offs / 100) * 100 + (i * 50) / 30;
                break;
            default:
                return NULL;
            }

            if (neg)
            {
                offs = -offs;
            }
            tm->tm_isdst = 0;   /* XXX */
#ifdef TM_GMTOFF
            tm->TM_GMTOFF = offs;
#endif
#ifdef TM_ZONE
            tm->TM_ZONE = NULL; /* XXX */
#endif
            continue;

        /*
         * Miscellaneous conversions.
         */
        case 'n':   /* Any kind of white-space. */
        case 't':
            while (isspace(*bp))
            {
                bp++;
            }

            LEGAL_ALT(0);
            continue;

        default:    /* Unknown/unsupported conversion. */
            return NULL;
        }
    }

    return (char*)(bp);
}


static const u_char *conv_num(const unsigned char *buf, int *dest, uint llim, uint ulim)
{
    uint result = 0;
    unsigned char ch;

    /* The limit also determines the number of valid digits. */
    uint rulim = ulim;

    ch = *buf;
    if ((ch < '0') || (ch > '9'))
    {
        return NULL;
    }

    do
    {
        result *= 10;
        result += ch - '0';
        rulim /= 10;
        ch = *++buf;
    }
    while ((result * 10 <= ulim) && rulim && ch >= '0' && ch <= '9');

    if ((result < llim) || (result > ulim))
    {
        return NULL;
    }

    *dest = result;
    return buf;
}


static const u_char *find_string(const u_char *bp, int *tgt, const char * const *n1,
                                 const char * const *n2, int c)
{
    int i;
    size_t len;

    /* check full name - then abbreviated ones */
    for (; n1 != NULL; n1 = n2, n2 = NULL)
    {
        for (i = 0; i < c; i++, n1++)
        {
            len = strlen(*n1);
            if (strncasecmp(*n1, (const char*)bp, len) == 0)
            {
                *tgt = i;
                return bp + len;
            }
        }
    }

    /* Nothing matched */
    return NULL;
}


int inet_pton(int af, const char *src, void *dst)
{
    struct sockaddr_storage ss;
    int size = sizeof(ss);
    char src_copy[INET6_ADDRSTRLEN + 1];

    ZeroMemory(&ss, sizeof(ss));
    /* stupid non-const API */
    strncpy(src_copy, src, INET6_ADDRSTRLEN + 1);
    src_copy[INET6_ADDRSTRLEN] = 0;

    if (WSAStringToAddress(src_copy, af, NULL, (struct sockaddr *)&ss, &size) == 0)
    {
        switch (af)
        {
        case AF_INET:
            *(struct in_addr *)dst = ((struct sockaddr_in *)&ss)->sin_addr;
            return 1;
        case AF_INET6:
            *(struct in6_addr *)dst = ((struct sockaddr_in6 *)&ss)->sin6_addr;
            return 1;
        }
    }
    return 0;
}


const char *inet_ntop(int af, const void *src, char *dst, socklen_t size)
{
    struct sockaddr_storage ss;
    unsigned long s = size;

    ZeroMemory(&ss, sizeof(ss));
    ss.ss_family = af;

    switch (af)
    {
    case AF_INET:
        ((struct sockaddr_in *)&ss)->sin_addr = *(struct in_addr *)src;
        break;
    case AF_INET6:
        ((struct sockaddr_in6 *)&ss)->sin6_addr = *(struct in6_addr *)src;
        break;
    default:
        return NULL;
    }

    /* cannot direclty use &size because of strict aliasing rules */
    return (WSAAddressToString((struct sockaddr *)&ss, sizeof(ss), NULL, dst, &s) == 0) ?
           dst : NULL;
}


#endif // if defined(_MSC_VER)

#endif // ifdef _WIN32
