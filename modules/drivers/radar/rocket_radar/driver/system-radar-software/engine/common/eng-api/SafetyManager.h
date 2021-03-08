#ifndef SRS_HDR_SAFETYMANAGER_H
#define SRS_HDR_SAFETYMANAGER_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "lifeCycleManager.h"

SRS_DECLARE_NAMESPACE()

/*! Boot Safety Manager which handles the Boot time errors and controls reboot and shutdown
 *  based on Lifecycle
 * */
class SafetyManager
{
public:

    static SafetyManager& instance();

    /*!
     * init -  initializes the safety manager
     * count -  input parameter which sets the number of reboots for this boot
     */
    void init(const uint32_t count);
    /*!handles the error during boot based on lifecycle triggers
     *input:
     *     trigg -  Lifecyle trigger for this error
     *     error_code -  error code
     * */
    void handle_error(const LifeCycleTriggers_t trigg, const int32_t error_code);
    /*! API to trigger warm boot/ reboot of chip
     *
     * */
    bool warm_boot(void);
    /*! boot count which indicates how many times chip rebooted (warm boot)
     *
     * */
    uint32_t boot_count;
    /*! API to shut down the chip*/
    void shut_down();
    /*! API to reboot the chip*/
    void reboot();
};

int32_t get_bist_fail_status(void);


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_SAFETYMANAGER_H
