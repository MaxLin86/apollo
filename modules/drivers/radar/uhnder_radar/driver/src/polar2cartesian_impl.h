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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/polar2cartesian.h"

class Polar2Cartesian_Impl : public Polar2Cartesian
{
public:

    struct Frac2D
    {
        // fractional bin indices
        float range;
        float azimuth;
        float yc;
        float xc;
    };

    struct Frac2DFrontView
    {
        // fractional bin indices
        float azimuth;
        float elevation;
    };

    struct Frac3D
    {
        // fractional bin indices
        float range;
        float azimuth;
        float elevation;
    };

    Polar2Cartesian_Impl()
        : last_err(P2C_NO_ERROR)
        , frac2d(NULL)
        , frac3d(NULL)
        , frac2dfront(NULL)
        , obuf(NULL)
    {
        // invalid subspace extent, so first process() does not blend
        buffered_subspace.extent_X = (uint32_t)-1;
    }


    virtual ~Polar2Cartesian_Impl()
    {
        delete [] frac2d;
        delete [] frac3d;
        delete [] frac2dfront;
        delete [] obuf;
    }


    virtual const uint32_t* process(const ClutterImage&, SubSpace*, float alpha);

    virtual Err             get_last_error() const { return last_err; }

    virtual void            release()              { delete this; }

    Err                     last_err;

    uint32_t                dim_X;

    uint32_t                dim_Y;

    uint32_t                dim_Z;

    uint32_t                range_bins;

    uint32_t                azimuth_bins;

    uint32_t                elevation_bins;

    Frac2D*                 frac2d;

    Frac3D*                 frac3d;

    Frac2DFrontView*        frac2dfront;

    uint32_t*               obuf;

    SubSpace                buffered_subspace;
};
