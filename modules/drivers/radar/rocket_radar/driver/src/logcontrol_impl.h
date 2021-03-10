// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/include/logcontrol.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/ip-protocols.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/prothandlerbase.h"

class Connection_Impl;

class LogControl_Impl : public LogControl, public ProtHandlerBase
{
public:

    enum { MAX_MAX_SUBUNITS = 512 }; // old radar limit was 36

    LogControl_Impl(Connection_Impl& con)
        : mycon(con)
        , num_producers(0)
        , num_changes(0)
    {
        UhdpHeader* hdr = reinterpret_cast<UhdpHeader*>(change_buffer);
        hdr->message_type = UHDP_TYPE_LOG_CONTROL;
        changes = reinterpret_cast<UhdpLogLevelControl*>(hdr + 1);
    }

    virtual ~LogControl_Impl() { cleanup(); }

    virtual uint32_t    get_num_log_producers() const;

    virtual const char* get_log_producer_name(uint32_t prod_id) const;

    virtual uint32_t    get_num_log_producer_subunits(uint32_t prod_id) const;

    virtual const char* get_log_producer_subunit_name(uint32_t prod_id, uint32_t subunit_id) const;

    virtual void        set_log_filter(uint32_t prod_id, uint32_t subunit_id, LogLevelEnum level);

    virtual void        apply_log_filter_changes();

    void cleanup()
    {
        for (uint32_t i = 0; i < num_producers; i++)
        {
            delete [] producers[i].prod_name;
            delete [] producers[i].subunit_names;
        }

        num_producers = 0;
    }


    void handle_packet(const char* payload, uint32_t len)
    {
        if (disabled)
        {
            return;
        }

        while (len)
        {
            uint16_t csize = *(uint16_t*)payload;
            payload += sizeof(uint16_t);
            len -= sizeof(uint16_t);

            if (len < csize)
            {
                printf("ERROR: malformed log producer message\n");
                return;
            }

            if (num_producers >= MAX_PRODUCER)
            {
                printf("ERROR: log producer overflow\n");
                return;
            }

            /* payload to payload+csize contains zero terminated names, the
             * first is the producer name, the remaining are subunit names */

            LP& prod = producers[num_producers++];
            prod.num_subunits = 0;
            prod.subunit_names = NULL;

            uint32_t plen = strnlen(payload, csize);
            prod.prod_name = new char[plen + 1];
            memcpy(prod.prod_name, payload, plen);
            prod.prod_name[plen] = 0;

            //printf("Parsed Log Producer: <%s>\n", prod.prod_name);

            if (csize > plen + 1)
            {
                uint32_t nsize = csize - (plen + 1);
                prod.subunit_names = new char[csize + 1];
                memcpy(prod.subunit_names, payload + plen + 1, nsize);
                prod.subunit_names[nsize] = 0;

                uint32_t offset = 0;
                while (offset < nsize && prod.num_subunits < MAX_MAX_SUBUNITS)
                {
                    prod.subunit_offsets[prod.num_subunits++] = offset;
                    //printf("\tsubunit <%s>\n", prod.subunit_names + offset);

                    offset += strnlen(prod.subunit_names + offset, nsize) + 1;
                }
            }

            len -= csize;
            payload += csize;
        }
    }

    Connection_Impl&    mycon;

    struct LP
    {
        char*    prod_name;
        char*    subunit_names;
        uint32_t subunit_offsets[MAX_MAX_SUBUNITS];
        uint32_t num_subunits;
    };

    enum { MAX_PRODUCER = 8 };

    LP       producers[MAX_PRODUCER];
    uint32_t num_producers;

    const char* get_prod_name(uint32_t prodid) const
    {
        if (prodid < num_producers)
        {
            return producers[prodid].prod_name;
        }
        else
        {
            return "LOG_PRODUCER_OUT_OF_RANGE";
        }
    }

    const char* get_subunit_name(uint32_t prodid, uint32_t subunit) const
    {
        if ((prodid < num_producers) &&
            (subunit < producers[prodid].num_subunits))
        {
            uint32_t offset = producers[prodid].subunit_offsets[subunit];
            return producers[prodid].subunit_names + offset;
        }
        else
        {
            return NULL;
        }
    }

    uint32_t            num_changes;

    uint8_t             change_buffer[MAX_UDP_DATAGRAM];

    enum { MAX_CHANGES = (MAX_UDP_DATAGRAM - sizeof(UhdpHeader)) / sizeof(UhdpLogLevelControl) };

    UhdpLogLevelControl* changes;
};
