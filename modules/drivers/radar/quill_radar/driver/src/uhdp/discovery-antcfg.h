#pragma once

#include "modules/drivers/radar/quill_radar/driver/src/uhdp/prothandlerbase.h"

struct AntennaConfigDiscHandler : public ProtHandlerBase
{
    enum { MAX_ANTENNA_CONFIGS = 64 };
    enum { MAX_BASIC_ANTENNA_CONFIGS = 8 };

    char*       antenna_config_name[MAX_ANTENNA_CONFIGS];
    uint32_t    basic_antenna_config[MAX_BASIC_ANTENNA_CONFIGS];
    uint32_t    num_antenna_configs;
    bool        first_packet;

    AntennaConfigDiscHandler()
        : num_antenna_configs(0)
        , first_packet(true)
    {
        memset(basic_antenna_config, 0, sizeof(basic_antenna_config));
    }

    ~AntennaConfigDiscHandler() { cleanup(); }

    void cleanup()
    {
        for (uint32_t i = 0; i < num_antenna_configs; i++)
        {
            delete [] antenna_config_name[i];
        }

        num_antenna_configs = 0;
        first_packet = true;
    }

    void handle_packet(const char* payload, uint32_t len)
    {
        if (disabled)
        {
            return;
        }

        if (first_packet)
        {
            for (uint32_t i = 0; i < MAX_BASIC_ANTENNA_CONFIGS; i++)
            {
                basic_antenna_config[i] = payload[i];
            }

            payload += MAX_BASIC_ANTENNA_CONFIGS;
            len     -= MAX_BASIC_ANTENNA_CONFIGS;
            first_packet = false;
        }

        while (len)
        {
            uint8_t nlen = payload[0];
            antenna_config_name[num_antenna_configs] = new char[nlen + 1];
            memcpy(antenna_config_name[num_antenna_configs], payload + 1, nlen);
            antenna_config_name[num_antenna_configs][nlen] = 0;
            num_antenna_configs++;
            assert(MAX_ANTENNA_CONFIGS >= num_antenna_configs);

            payload += nlen + 1;
            len     -= nlen + 1;
        }
    }


    uint32_t     get_num_antenna_configs() const
    {
        return num_antenna_configs;
    }


    const char*  get_antenna_config_name(uint32_t d) const
    {
        if (d < num_antenna_configs)
        {
            return antenna_config_name[d];
        }
        else
        {
            return NULL;
        }
    }


    uint32_t get_basic_antenna_config_id(uint32_t b) const
    {
        if (b < MAX_BASIC_ANTENNA_CONFIGS)
        {
            return basic_antenna_config[b];
        }
        else
        {
            return 0;
        }
    }
};
