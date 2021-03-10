// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/prothandlerbase.h"

struct DiagDiscHandler : public ProtHandlerBase
{
    enum { MAX_DIAGS = 64 };

    enum { MAX_TESTS_PER_DIAG = 128 };

    struct Diag
    {
        const char* diagname;
        const char* testname[MAX_TESTS_PER_DIAG];
        uint8_t     test_api_ver[MAX_TESTS_PER_DIAG];
        char        namedata[1024];
        uint32_t    num_tests;
    };

    Diag     diags[MAX_DIAGS];
    uint32_t num_diags;

    DiagDiscHandler() : num_diags(0) {}

    void handle_packet(const char* payload, uint32_t len)
    {
        if (disabled)
        {
            return;
        }

        while (len)
        {
            /* Process the info for one diag:
             * uint16_t diag_size
             * uint8_t  namelen;
             * char     name[namelen];
             * struct
             * {
             *     uint8_t api_ver;
             *     uint8_t tlen;
             *     char    testname[tlen];
             * } test[num_tests];
             */
            uint16_t diag_size = *(uint16_t*)payload;
            len     -= sizeof(diag_size);
            payload += sizeof(diag_size);

            Diag& d = diags[num_diags++];
            assert(diag_size < sizeof(d.namedata));
            memcpy(d.namedata, payload, diag_size);
            d.namedata[diag_size] = 0;

            payload += diag_size;
            len     -= diag_size;

            char*   nd      = d.namedata;
            uint8_t namelen = d.namedata[0];
            d.diagname  = nd + 1;
            d.num_tests = 0;

            nd        += namelen + 1;
            diag_size -= namelen + 1;
            while (diag_size)
            {
                uint8_t api_ver = nd[0];
                uint8_t tlen    = nd[1];

                d.test_api_ver[d.num_tests] = api_ver;
                d.testname[d.num_tests++] = nd + 2;
                nd[0] = 0; // zero-terminate previous string

                nd        += tlen + 2;
                diag_size -= tlen + 2;
            }

            nd[0] = 0; // zero-terminate last string

            assert(d.num_tests < MAX_TESTS_PER_DIAG);
        }

        assert(num_diags < MAX_DIAGS);
    }


    uint32_t     get_num_diags() const
    {
        return num_diags;
    }


    const char*  get_diag_name(uint32_t d) const
    {
        if (d < num_diags)
        {
            return diags[d].diagname;
        }
        else
        {
            return NULL;
        }
    }


    uint32_t     get_num_tests_in_diag(uint32_t d) const
    {
        if (d < num_diags)
        {
            return diags[d].num_tests;
        }
        else
        {
            return 0;
        }
    }


    const char*  get_diag_test_name(uint32_t d, uint32_t test_id) const
    {
        if ((d < num_diags) && (test_id < diags[d].num_tests))
        {
            return diags[d].testname[test_id];
        }
        else
        {
            return NULL;
        }
    }


    uint8_t      get_diag_test_api_version(uint32_t d, uint32_t test_id) const
    {
        if ((d < num_diags) && (test_id < diags[d].num_tests))
        {
            return diags[d].test_api_ver[test_id];
        }
        else
        {
            return 0xFF;
        }
    }
};
