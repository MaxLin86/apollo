#pragma once

#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/prothandlerbase.h"

struct LogMsgDiscHandler : public ProtHandlerBase
{
    LogMsgDiscHandler() : num_message_types(0) {}

    virtual ~LogMsgDiscHandler() { cleanup(); }

    void cleanup()
    {
        for (uint32_t i = 0; i < num_message_types; i++)
        {
            delete [] message[i];
        }

        num_message_types = 0;
    }


    void handle_packet(const char* payload, uint32_t len)
    {
        if (disabled)
        {
            return;
        }

        while (len)
        {
            if (num_message_types >= MAX_LOG_MESSAGE)
            {
                printf("MAX_LOG_MESSAGE is insufficient\n");
                return;
            }

            uint32_t msglen = strnlen(payload, len);
            message[num_message_types] = new char[msglen + 1];
            memcpy(message[num_message_types], payload, msglen);
            message[num_message_types][msglen] = 0;

            pcount[num_message_types] = 0;
            for (const char* p = message[num_message_types]; *p; p++)
            {
                if (*p == '%')
                {
                    p++; /* skip next char 'd' or 'x' */
                    pcount[num_message_types]++;
                }
            }

            num_message_types++;

            if (msglen == len)
            {
                return;
            }

            payload += msglen + 1;
            len -= msglen + 1;
        }
    }


    enum { MAX_LOG_MESSAGE = 512 };

    char*    message[MAX_LOG_MESSAGE];
    uint32_t pcount[MAX_LOG_MESSAGE];
    uint32_t num_message_types;

    const char* get_message(uint32_t type, uint32_t& pct_count) const
    {
        if (type < num_message_types)
        {
            pct_count = pcount[type];
            return message[type];
        }
        else
        {
            return "LOG_MESSAGE_OUT_OF_RANGE";
        }
    }
};
