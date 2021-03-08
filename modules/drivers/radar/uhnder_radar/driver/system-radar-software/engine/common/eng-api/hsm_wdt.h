#ifndef SRS_HDR_HSM_WDT_H
#define SRS_HDR_HSM_WDT_H 1
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

/* XO CLK = 40MHz, LP_MODE CLK = 360 MHz */
#define WDT_TRIGG_1000MS (0x1312D00U * 9)
#define WDT_TRIGG_100MS (0x1E8480U * 9)
#define WDT_TRIGG_10MS (0x30D40U * 9)
#define WDT_TRIGG_1MS (0x4E20U * 9)

SRS_DECLARE_NAMESPACE()

/*! hsm_wdt_init() - Initializes the HSM Watch Dog Timer
 *  loadval -  Initial load value for watch dog timer
 *
 */
void hsm_wdt_init(const uint32_t loadval);

/*!Reload HSM WDT with the value passed in loadval
 * BOOT code should reload HSM value periodically
 * to avoid WDT triggering reset
 * */
void hsm_wdt_reload(const uint32_t loadval);

/*!
 * Stop HSM WDT timer
 * This will disable WDT and stop it from triggering reset
 * */
void hsm_wdt_stop(void);

/*!
 * Start HSM WDT timer
 * This will enable WDT
 * */
void hsm_wdt_start(void);

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_HSM_WDT_H
