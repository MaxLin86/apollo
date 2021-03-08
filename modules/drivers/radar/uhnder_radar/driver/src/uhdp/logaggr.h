#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/include/sra.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/uhdp/prothandlerbase.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/logcontrol.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"


static const char* level_names[] = { "pedantic", "debug", "verbose", "info",
                                     "warn", "error", "always", "profile" };

struct LogAggregateHandler : public ProtHandlerBase
{
    LogAggregateHandler(UserLogAgent& _log_agent, const LogControl_Impl& _log_ctrl, const LogMsgDiscHandler& _msg_disc)
        : log_agent(_log_agent)
        , log_ctrl(_log_ctrl)
        , msg_disc(_msg_disc)
    {
    }


    void dump_log_event(uint32_t cpuid, const LogEvent& ev)
    {
        uint32_t pctcount = 0;
        const char* prodname = log_ctrl.get_prod_name((LogProducerEnum)ev.p.producer);
        const char* subname  = log_ctrl.get_subunit_name((LogProducerEnum)ev.p.producer, ev.subunit);
        const char* level    = level_names[ev.p.loglevel];
        const char* message  = msg_disc.get_message(ev.messageId, pctcount);

        char buf[1024];
        int len = 0;

        if (subname)
        {
            len = sprintf(buf, "<%s:%s><%s> ", prodname, subname, level);
        }
        else
        {
            len = sprintf(buf, "<%s><%s> ", prodname, level);
        }

        /* if the message string has printf-format declarator like %d, then use it
         * directly as the printf format string. Else only print data if it is
         * non-zero */
        if (pctcount == 0)
        {
            len += sprintf(buf + len, "%s", message);
        }
        else if (pctcount == 1)
        {
            len += sprintf(buf + len, message, ev.data1);
        }
        else if (pctcount == 2)
        {
            len += sprintf(buf + len, message, ev.data1, ev.data2);
        }
        else
        {
            len += sprintf(buf + len, "Log message ID %d <%s> is invalid", ev.messageId, message);
        }

        buf[len++] = 0;
        log_agent.radar_log_message((ICCQTargetEnum)cpuid, (LogLevelEnum)ev.p.loglevel, buf);
    }


    void handle_packet(const char* payload, uint32_t len)
    {
        //uint32_t seq = *(const uint32_t*)payload;

        payload += sizeof(uint32_t);
        len     -= sizeof(uint32_t);

        while (len > 2)
        {
            const uint8_t cpuid = *(const uint8_t*)payload;
            const uint8_t msglen = *(const uint8_t*)(payload + 1);

            if (msglen == 0xFF)
            {
                const LogEvent& ev = *reinterpret_cast<const LogEvent*>(payload + 2);

                dump_log_event(cpuid, ev);

                payload += sizeof(LogEvent) + 2;
                len     -= sizeof(LogEvent) + 2;
            }
            else if (msglen + 2U <= len)
            {
                const char* msg = payload + 2;

                // strip trailing line feeds
                uint32_t displen = msglen;
                while (displen && msg[displen - 1] == '\n')
                {
                    displen--;
                }

                char buf[1024];

                if (displen)
                {
                    memcpy(buf, msg, displen);
                    buf[displen] = 0;
                    log_agent.radar_print_message((ICCQTargetEnum)cpuid, buf);
                }

                payload += msglen + 2;
                len     -= msglen + 2;
            }
            else
            {
                printf("Invalid message length %d > %d\n", msglen, len);
                return;
            }
        }
    }


    UserLogAgent&            log_agent;
    const LogControl_Impl&   log_ctrl;
    const LogMsgDiscHandler& msg_disc;
};
