#ifndef SRS_HDR_LOGGING_ENUMS_H
#define SRS_HDR_LOGGING_ENUMS_H 1
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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

/*! each log event occupies 16 bytes */
struct LogEvent
{
    struct LogEventTypeData
    {
        uint8_t producer : 3;
        uint8_t loglevel : 3;
        uint8_t type :     2;
    } p;

    uint8_t  subunit;

    uint16_t messageId;

    uint32_t time;
    uint32_t data1;
    uint32_t data2;
};

enum LogTypes
{
    LOG_TYPE_START = 0,
    LOG_TYPE_STOP,
    LOG_TYPE_MESSAGE,
};

/*! high level logging producers */
enum LogProducerEnum
{
    LOG_ENV  = 0,
    LOG_FUSA = 1,
    LOG_RHAL = 2,
    LOG_RDC  = 3,
    LOG_DIAG = 4,
    LOG_UHDP = 5,
    LOG_DSP  = 6,
    LOG_ENG  = 7,
    NUM_LOG_PRODUCERS
};

/*! priority level of each log message */
enum LogLevelEnum
{
    LL_PEDANTIC,  //! noise, usually filtered
    LL_DEBUG,     //! only enabled by developers
    LL_VERBOSE,   //! only enabled by testers, integrators
    LL_INFO,      //! informational details (normal operations, default)
    LL_WARN,      //! exceptional occurances
    LL_ERROR,     //! breakages great and small
    LL_ALWAYS,    //! never filtered
    LL_PROFILE,   //! used by profile events

    NUM_LOG_LEVELS
};

enum ICCQTargetEnum
{
    ICCQ_TARGET_SCP,
    ICCQ_TARGET_DSP1,
    ICCQ_TARGET_DSP2,
    ICCQ_TARGET_CCP,
    ICCQ_TARGET_HSM,

    ICCQ_TARGET_LOCAL,
};

enum ICCQUserIdEnum
{
    ICCQ_USER_CCP,
    ICCQ_USER_HSM,
    ICCQ_USER_SCP0,
    ICCQ_USER_SCP1,
    ICCQ_USER_DSP0,
    ICCQ_USER_DSP1,
};

void uh_buffer_log(ICCQTargetEnum cpu, const struct LogEvent& ev);
void uh_buffer_puts(ICCQTargetEnum cpu, const CHAR* msg, uint32_t slen);

enum LogMessageEnum
{
#define DEFINE_MESSAGE(id, string) id,
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/engine-messagedef.inc"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/environment-messagedef.inc"
    NUM_MESSAGE_ENUMS
};

#undef DEFINE_MESSAGE

enum LogTaskEnum
{
#undef DEFINE_TASK
#define DEFINE_TASK(id, string) id,
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/engine-taskdef.inc"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/environment-taskdef.inc"
#undef DEFINE_TASK
    NUM_TASK_ENUMS
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_LOGGING_ENUMS_H
