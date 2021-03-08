#ifndef SRS_HDR_UHUTILS_H
#define SRS_HDR_UHUTILS_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

// these cores can buffer log messages
#if __SCP__ || __HSM__
#define HAVE_BUFFERING 1
#else
#define HAVE_BUFFERING 0
#endif

#if SRS_PLATFORM_ARMCC
#define SRS_SHORT_FILE __MODULE__
#else
#define SRS_SHORT_FILE __FUNCTION__
#endif

#if ENABLE_UHPRINTF


#if SRS_PLATFORM_GHS
#define VA_ARGS(...)
#define UHPRINTF(fmt, ...)          uh_printf(fmt VA_ARGS(__VA_ARGS__))
#define UHFATAL(fmt, ...)           uh_fatal_printf(false, SRS_SHORT_FILE, __LINE__, fmt VA_ARGS(__VA_ARGS__))
#else
#define UHPRINTF(...)               uh_printf(__VA_ARGS__)
#define UHFATAL(...)                uh_fatal_printf(false, SRS_SHORT_FILE, __LINE__, __VA_ARGS__)
#endif


/*! if expression evaluates to false, print exception message and enter fatal
 * exception handler */
#define UHASSERT(expr)              uh_fatal_printf((expr), SRS_SHORT_FILE, __LINE__, "ASSERT FAIL: ( %s )\n", # expr)

#define UHFATALLOG(p, s, e, d1, d2) SystemLogger::instance().emit_fatal(SRS_SHORT_FILE, __LINE__, p, s, e, d1, d2)

#else // if ENABLE_UHPRINTF

#define UHPRINTF(fmt, ...)          uh_nop()
#define UHFATAL(fmt, ...)           uh_fatal_error(SRS_SHORT_FILE, __LINE__)
#define UHASSERT(expr)              uh_fatal_printf((expr), SRS_SHORT_FILE, __LINE__, "ASSERT FAIL")
#define UHFATALLOG(p, s, e, d1, d2) uh_fatal_error(SRS_SHORT_FILE, __LINE__)

#endif // if ENABLE_UHPRINTF


#define UH_CLOCK_TICS_PER_SECOND        1000000U // A tick is 1us - conversion is done at the driver
#define UH_CLOCK_TICS_PER_MSEC          (UH_CLOCK_TICS_PER_SECOND / 1000U)
#define UH_CLOCK_TICS_PER_MICRO_SECONDS (UH_CLOCK_TICS_PER_SECOND / 1000000U)

SRS_DECLARE_NAMESPACE()

/*! maximum length of UPRINTF and UHFATAL output string */
enum { MAX_LOG_LENGTH = 192 };

// engine/common/src/uhutils.cpp
void     uh_sw_nop(void);
void     uh_printf(const CHAR* format, ...) CHECK_PRINTF_ARGS(1, 2);
void     uh_fatal_printf(bool isok, const CHAR* sourcefile, INT lineno, const CHAR* format, ...) CHECK_PRINTF_ARGS(4, 5);
void     uh_puts(const CHAR* msg);                /*! same as uh_printf but without varargs */
void     uh_fatal_error(const CHAR* sourcefile, INT lineno);
void     uh_exception_uart_monitor(const CHAR* sourcefile, INT lineno);
void     uh_print_symbol_list(CHAR** symbol_list, INT num_symbols);
void     uh_memcpy32(void *dest, const void *src, size_t n);

#if __DSP__
INT      uh_get_dsp_id(void); /* returns 0 for DSP1, 1 for DSP2 */
#endif

// low-level debugging prints, bypass system logger and UART enable/disable
// control and write directly to UART 0
void     uh_uart_dump_mem(void *p, INT len, INT b4);
void     uh_uart_printf(const CHAR* format, ...) CHECK_PRINTF_ARGS(1, 2);

uint32_t uh_read_clock(void);                    /*! returns wall time with resolution of UH_CLOCK_TICS_PER_SECOND */
uint64_t uh_read_64bit_counter(void);
void     uh_wait_us(uint32_t wait);              /*! wait (blocking) for wait usec */

#if SRS_PLATFORM_POSIX && !defined(BUILD_SABINE_REMOTE_API)
// These are *NOT* supposed to be implemented. They only exist to cause link errors
// if engine code inadvertenntly uses these functions instead of uh_iabs or uh_fabsf
int   abs(int x);
float fmin(float x, float y);
float fmax(float x, float y);
float fminf(float x, float y);
float fmaxf(float x, float y);
float fabs(float x);
float floor(float x);
float ceil(float x);
void* memcpy(void *dest, const void *src, size_t n);
int   atoi(const char *str);
#endif

/*! called by log agent with room to transmit buffered messages. this function
 * fills in up to 'available' bytes of log data into 'dest' and returns a count
 * of the bytes filled. */
uint32_t uh_get_buffered_messages(uint8_t* dest, uint32_t available);
/*! reports dropped message count, if non-zero */
uint32_t uh_get_dropped_messages(void);

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHUTILS_H
