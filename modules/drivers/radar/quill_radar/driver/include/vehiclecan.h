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
#pragma once

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhinet.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhunistd.h"
#include "modules/drivers/radar/quill_radar/driver/include/rra.h"
#include <time.h>

//! abstract API which a CAN vehicle model must implement in order to be
//! integrated into the Connection object's polling and telemetry update
//! infrastructure.
class VehicleCAN
{
public:

    VehicleCAN()
        : axle_dist(3.5)
        , vel_kmph_x(0.0f)
        , vel_kmph_y(0.0f)
        , vel_kmph_z(0.0f)
        , acc_mpsps_x(0.0f)
        , acc_mpsps_y(0.0f)
        , acc_mpsps_z(0.0f)
        , yaw_rate(0.0f)
        , steering_angle(0.0f)
        , brake_pressure(0.0f)
    {
        update_time.tv_sec = 0;
        update_time.tv_usec = 0;
        hw_epoc_time.tv_sec = 0;
        hw_epoc_time.tv_usec = 0;
    }

    virtual ~VehicleCAN() {}

    //! returns the native file descriptor for the CAN device for use in poll() or
    //! select() statements. Return 0 if a native descriptor is not available
    virtual int get_file_handle() const = 0;

    //! poll the CAN device for messages, updates telemetry data, returns true
    //! if the velocity data has been updated
    virtual bool poll() = 0;

    //! can be called by poll when a velocity update is received with a
    //! hardware timestamp. If your CAN driver does not provide read hardware
    //! timestamps, a reasonable alternative is to immediately call
    //! gettimeofday(&update_time, NULL)
    void set_update_time(uint32_t elapsed_ms, uint16_t elapsed_us);

    timeval     update_time;      //!< host time of last velocity update

    // If your CAN model does not calculate some of these fields, leave them 0

    float       axle_dist;        //!< distance of radar from rear axle
    float       vel_kmph_x;       //!< vehicle forward velocity, kilometers per hour
    float       vel_kmph_y;       //!< vehicle Y tangential velocity, kilometers per hour
    float       vel_kmph_z;       //!< vehicle Z tangential velocity, kilometers per hour
    float       acc_mpsps_x;      //!< vehicle forward acceleration, meters per second^2
    float       acc_mpsps_y;      //!< vehicle Y tangential acceleration, meters per second^2
    float       acc_mpsps_z;      //!< vehicle Z tangential acceleration, meters per second^2
    float       yaw_rate;         //!< radians per second
    float       steering_angle;   //!< radians from boresight
    float       brake_pressure;   //!< bars

protected:

    timeval     hw_epoc_time;     //!< estimated host time of hardware timestamp clock epoch
};

