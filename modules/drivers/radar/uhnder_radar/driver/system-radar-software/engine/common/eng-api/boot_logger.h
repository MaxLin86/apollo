#pragma once
#ifndef SRS_HDR_BOOT_LOGGER_H
#define SRS_HDR_BOOT_LOGGER_H 1
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/b0_debug_logs.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/b1_debug_logs.h"
#include "device.h"


#ifndef B0_DEBUG_EMULATION
#define B0_DEBUG_EMULATION 0
#endif
SRS_DECLARE_NAMESPACE()
void sysctl_debug_log(uint32_t c);
#if B0_DEBUG_EMULATION
#define PRFCTL_DEBUG(val) sysctl_debug_log(val)
#define TSTCTL_DEBUG(val) sysctl_debug_log(val)
//#define PRFCTL_DEBUG(val)
//#define TSTCTL_DEBUG(val)
#else
#define TSTCTL_DEBUG(val)
#define PRFCTL_DEBUG(val)
#endif

class Logger
{
public:
#if __BARE_METAL__
    virtual void init(DEVICE * const arg) = 0;
#else
    virtual void init(void) = 0;
#endif
    virtual void my_putc(const CHAR c) = 0;
    virtual void puts(const CHAR *s);
    virtual void putx(const uint32_t hex_data);
    virtual void log_mesg(const CHAR * const s, const uint32_t val);
    virtual void log_err(const int32_t err, const uint32_t data);
    virtual uint32_t get_log_wr(void){ return 0;}
    virtual uint32_t get_log_wrap(void){ return 0;}
};

class UARTLogger : public Logger
{
public:

#if __BARE_METAL__
    virtual void init(DEVICE * const arg);
#else
    virtual void init(void);
#endif
    virtual void my_putc(const CHAR c);

protected:

#if __BARE_METAL__
    DEVICE *log_dev;
#endif
};

class NULLLogger : public Logger
{
public:
#if __BARE_METAL__
    virtual void init(DEVICE * const arg);
#else
    virtual void init(void);
#endif
    virtual void my_putc(const CHAR c);
    virtual void puts(const CHAR *s);
    virtual void putx(const uint32_t hex_data);
    virtual void log_mesg(const CHAR * const s, const uint32_t val);
    virtual void log_err(const int32_t err, const uint32_t data);
    virtual uint32_t get_log_wr(void){return wr_ptr;}
    virtual uint32_t get_log_wrap(void)
    {
        if(is_wrap)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
private:

    uint32_t wr_ptr;
    bool is_wrap;
};

extern Logger *logger;

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_BOOT_LOGGER_H
