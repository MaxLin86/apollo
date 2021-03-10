// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_LOGGING_ENUMS_H
#define SRS_HDR_LOGGING_ENUMS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

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
#include "engine-messagedef.inc"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/environment-messagedef.inc"
    NUM_MESSAGE_ENUMS
};

#undef DEFINE_MESSAGE

enum LogTaskEnum
{
#undef DEFINE_TASK
#define DEFINE_TASK(id, string) id,
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/engine-taskdef.inc"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/environment-taskdef.inc"
#undef DEFINE_TASK
    NUM_TASK_ENUMS
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_LOGGING_ENUMS_H
