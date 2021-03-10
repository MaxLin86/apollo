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
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"

struct Track
{
    uint32_t  id;
    float     confidence;
    vec3f_t   posRelHost;
    vec3f_t   velRelHost;
    vec3f_t   accRelHost;
    float     yawRate;
    float     yawAg;
    uint32_t  age;
    uint8_t   mtnSt;
    uint8_t   objSt;
    uint8_t   objClassn;
    uint8_t   reserved_u8;
    float     objWidth;
    float     objLen;
    vec3f_t   posStdDe;
    vec3f_t   velStdDe;
    vec3f_t   accStdDe;
    uint32_t  reserved_u32[4];
};


class Tracks
{
public:

    virtual ~Tracks() {}

    //! returns the number of tracks reported in this scan
    virtual uint32_t             get_count() const = 0;

    //! returns a specific track instance
    virtual const Track&         get_track(uint32_t idx) const = 0;

    //! send this scan to the tracker and update tracks
    virtual void                 step_tracker() = 0;

    //! release all of the memory associated with this Tracks instance
    virtual void                 release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        TRACK_NO_ERROR,
        TRACK_INDEX_OUT_OF_RANGE,
    };

    //! returns the last error encountered by this instance
    virtual Err                  get_last_error() const = 0;

    //! overwrite tracks with user-supplied tracks (intended for use by external
    //! trackers). The new_tracks are copied into allocated heap memory
    virtual void                 set_tracks(uint32_t new_count, const Track* new_tracks) = 0;
};
