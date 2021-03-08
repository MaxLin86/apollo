#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/logcontrol_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/connection_impl.h"

uint32_t    LogControl_Impl::get_num_log_producers() const
{
    return num_producers;
}

const char* LogControl_Impl::get_log_producer_name(uint32_t prod_id) const
{
    return get_prod_name(prod_id);
}

uint32_t    LogControl_Impl::get_num_log_producer_subunits(uint32_t prod_id) const
{
    return producers[prod_id].num_subunits;
}

const char* LogControl_Impl::get_log_producer_subunit_name(uint32_t prod_id, uint32_t subunit_id) const
{
    return get_subunit_name(prod_id, subunit_id);
}

void        LogControl_Impl::set_log_filter(uint32_t prod_id, uint32_t subunit_id, LogLevelEnum level)
{
    changes[num_changes].producer = prod_id;
    changes[num_changes].subunit = subunit_id;
    changes[num_changes].loglevel = level;

    if (++num_changes == MAX_CHANGES)
    {
        apply_log_filter_changes();
    }
}

void        LogControl_Impl::apply_log_filter_changes()
{
    if (num_changes)
    {
        UhdpHeader* hdr = reinterpret_cast<UhdpHeader*>(change_buffer);
        hdr->total_length = sizeof(changes[0]) * num_changes + sizeof(UhdpHeader);
        mycon.send_uhdp(hdr);
        num_changes = 0;
    }
}
