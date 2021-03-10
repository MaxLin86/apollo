#ifndef SRS_HDR_STATE_MANAGER_H
#define SRS_HDR_STATE_MANAGER_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE
/*! \file */

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"

SRS_DECLARE_NAMESPACE()

/*! Attribute flags describing the current state of the radar system */
enum SystemStateAttributes
{
    STATE_ATTR_RHAL_INIT    = (1 << 0),  /*! Radar-HAL has been initialized */
    STATE_ATTR_RDC_INIT     = (1 << 1),  /*! RDC layer has been initialized */
    STATE_ATTR_OBJECT_INIT  = (1 << 2),  /*! Object layer has been initialized */
    STATE_ATTR_FEATURE_INIT = (1 << 3),  /*! Feature layer has been initialized */
    STATE_ATTR_DMA_INIT     = (1 << 4),  /*! DMA driver has been initialized */
    STATE_ATTR_ETH_INIT     = (1 << 5),  /*! Ethernet driver has been initialized */
    STATE_ATTR_ISR_INIT     = (1 << 6),  /*! ISR driver has been initialized */

    STATE_ATTR_VRX_ALIGN    = (1 << 8),  /*! VRX Align Field Mode (boot cal) passed */
    STATE_ATTR_ADI_SYNC     = (1 << 9),  /*! Analog/Digital Interface synchronized correctly at boot */

    STATE_ATTR_DSP1_BOOTED  = (1 << 11), /*! Tensillica P5 is booted */
    STATE_ATTR_DSP2_BOOTED  = (1 << 12), /*! Tensillica P5 is booted */
    STATE_ATTR_CCP_BOOTED   = (1 << 13), /*! M0+ control CPU is booted */
    STATE_ATTR_SCANS_PAUSED = (1 << 14), /*! Scans are temporarily suspended */

    /*! Scans are running, an RDC_FrameConfig is actively scanning. This flag is
     * not cleared when scans are paused, scanning is considered to still be
     * active. The flag is cleared when the RDC_FrameConfig has been completely
     * released, signaling it is safe to start scanning with a new
     * RDC_FrameConfig */
    STATE_ATTR_SCANNING     = (1 << 15),

    /*! This flag will be set unconditionally at boot on SabineB SOC */
    STATE_ATTR_SABINE_B     = (1 << 16),

    /*! Antenna module or other boot configuration is invalid. Radar will not
     * boot into a working state capable of running scans */
    STATE_ATTR_BAD_CONFIG   = (1 << 17),

    /* ... */
};

/*! Global manager of the radar system state. It is the central authority on
 * which services have been enabled and which processes are active. All state
 * changes are communicated via method calls. It has no event handlers */
class StateManager
{
public:

    static StateManager& instance();

    void init();

    uint32_t get_state_mask() const { return state_mask; }

    void set_flag(uint32_t flag)   { state_mask |= flag; }

    void clear_flag(uint32_t flag) { state_mask &= ~flag; }

    /* The SCP and HSM are implied to always both be booted */

    /*! when an entity needs a pause or gap between scans, they call this method
     * with an event handle they would like to be posted when the pause begins.
     * There is no time guaruntee for when the pause will occur, the RDC layer
     * will insert a pause as soon as is practical, but no sooner */
    void request_scan_pause(EventEnum ev);

    /*! when an entitity no longer needs a pause or gap between scans, they call
     * this method to clear their request. Scans will not resume until all pause
     * requests are cleared */
    void release_scan_pause(EventEnum ev);

    /*! called by the RDC layer when it begins active scan processing */
    void set_scans_running()
    {
        state_mask |= STATE_ATTR_SCANNING;
        state_mask &= ~STATE_ATTR_SCANS_PAUSED;
    }


    /*! called by the RDC layer to determine if a scan pause is being requested */
    bool is_pause_pending() const { return num_pause_handles > 0; }

    /*! called by the RDC layer when a pause has been affected (scans are paused).
     * The state manager will post the provided event once all scan-pause
     * requests have been released. The RDC layer must call set_scans_running()
     * once scans are resumed. */
    void set_pause_started(EventEnum pause_complete_ev);

private:

    uint32_t state_mask;

    enum { MAX_PAUSE_REQUESTORS = 4 };

    uint32_t num_pause_handles;
    EventEnum pause_handles[MAX_PAUSE_REQUESTORS];

    EventEnum pause_complete;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_STATE_MANAGER_H
