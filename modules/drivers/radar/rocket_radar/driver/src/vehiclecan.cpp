#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/vehiclecan.h"
#include "modules/drivers/radar/rocket_radar/driver/include/sra.h"
#include <time.h>

// elapsed_ms and elapsed_us are the elapsed time (in ms + us) since either the
// driver was booted or the device was opened. We don't really care which, so
// long as the difference from the current timeval is fixed. Note a 32bit
// elapsed_ms timer will wrap in 1000 hours.  If you keep your connection longer
// than 49 days you will have a problem.

void VehicleCAN::set_update_time(uint32_t elapsed_ms, uint16_t elapsed_us)
{
    if (!hw_epoc_time.tv_sec)
    {
        gettimeofday(&update_time, NULL);
        hw_epoc_time.tv_sec = update_time.tv_sec;
        hw_epoc_time.tv_usec = update_time.tv_usec;

        uint32_t elapsed_sec = elapsed_ms / 1000;
        uint32_t remainder_us = elapsed_us + 1000 * (elapsed_ms - 1000 * elapsed_sec);

        hw_epoc_time.tv_sec -= elapsed_sec;
        if (hw_epoc_time.tv_usec < (int32_t)remainder_us)
        {
            hw_epoc_time.tv_sec--;
            hw_epoc_time.tv_usec += 1000 * 1000;
        }
        hw_epoc_time.tv_usec -= remainder_us;
    }
    else
    {
        uint64_t total_us = hw_epoc_time.tv_usec + elapsed_us + elapsed_ms * 1000;
        uint64_t elapsed_sec = total_us / (1000 * 1000);

        update_time.tv_sec = hw_epoc_time.tv_sec + elapsed_sec;
        update_time.tv_usec = total_us - (elapsed_sec * 1000 * 1000);
    }
}
