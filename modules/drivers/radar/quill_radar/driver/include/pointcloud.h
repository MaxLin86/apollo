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
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"

//! Point cloud detections. Users must capture DL_POINT_CLOUD to get dynamic points
//! from the radar. Users must capture DL_CI to get static points from the radar
class PointCloud
{
public:

    virtual ~PointCloud() {}

    //! returns the number of points
    virtual uint32_t              get_count() = 0;

    enum PointFlags
    {
        PF_CLUTTER                = 1 << 0,  //!< from clutter image/static slice (not activations)
        PF_TWO_PEAKS              = 1 << 1,  //!< two peaks were at this location (different dopplers) (PF_CLUTTER only)
        PF_THREE_PEAKS            = 1 << 2,  //!< three peaks were at this location (different dopplers) (PF_CLUTTER only)
        PF_MORE_THAN_THREE_PEAKS  = 1 << 3,  //!< 4+ peaks were at this location (different dopplers) (PF_CLUTTER only)
        PF_STATIONARY             = 1 << 4,
    };

    //! output point structure
    struct Point
    {
        float    range;        //!< meters
        float    doppler;      //!< mps
        float    azimuth;      //!< radians from boresight
        float    elevation;    //!< radians from boresight
        float    mag_snr;      //!< decibels
        uint32_t flags;        //!< PointFlags
    };

    //! user must provide output storage for get_count() Point instances, this
    //! method writes them all
    virtual void                  get_points(Point* output) = 0;

    //! returns raw fixed-point (uninterpolated) point cloud points - the user
    //! is responsible for properly interpreting the fixed-point data
    virtual const PointCloudData* get_raw_points() = 0;

    //! discard all points below the specified SNR threshold. This function can
    //! obviously reduce the number returned by get_count().  If the scan is
    //! serialized after this function is called, only the surviving points are
    //! serialized.
    virtual void                  apply_threshold(float point_cloud_snr_dB) = 0;

    //! release all storage of the point cloud
    virtual void                  release() = 0;

    //! an enumeration of the errors potentially returned by this class
    enum Err
    {
        PC_NO_ERROR,
        PC_INVALID_INPUT,
        PC_FILE_WRITE_FAILURE,
    };

    //! returns the last error encountered by this instance
    virtual Err                   get_last_error() const = 0;
};
