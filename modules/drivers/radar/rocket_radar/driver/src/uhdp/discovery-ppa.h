// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/prothandlerbase.h"

struct PPADiscHandler : public ProtHandlerBase
{
    PPADiscHandler() : num_task_types(0) {}

    void cleanup()
    {
        for (uint32_t i = 0; i < num_task_types; i++)
        {
            delete [] task_name[i];
        }

        num_task_types = 0;
    }


    void handle_packet(const char* payload, uint32_t len)
    {
        if (disabled)
        {
            return;
        }

        while (len)
        {
            if (num_task_types >= MAX_PPA_NAMES)
            {
                printf("MAX_PPA_NAMES is insufficient\n");
                return;
            }

            uint32_t tlen = strnlen(payload, len);
            task_name[num_task_types] = new char[tlen + 1];
            memcpy(task_name[num_task_types], payload, tlen);
            task_name[num_task_types][tlen] = 0;
            num_task_types++;

            if (tlen == len)
            {
                return;
            }

            payload += tlen + 1;
            len -= tlen + 1;
        }
    }


    enum { MAX_PPA_NAMES = 512 };

    char*    task_name[MAX_PPA_NAMES];
    uint32_t num_task_types;

    const char* get_task_name(uint32_t type)
    {
        if (type < num_task_types)
        {
            return task_name[type];
        }
        else
        {
            return "TASK_NAME_OUT_OF_RANGE";
        }
    }
};
