#ifndef SRS_HDR_UHMATH_H
#define SRS_HDR_UHMATH_H 1
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


#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhmathtypes.h"

SRS_DECLARE_NAMESPACE()


static UHINLINE void  fmaxabs(FLOAT* max, FLOAT val)
{
    FLOAT aval = uh_fabsf(val);

    if (aval > *max)
    {
        *max = aval;
    }
}

static UHINLINE uint32_t uh_uintmin(uint32_t val_a, uint32_t val_b)
{
    if (val_a < val_b)
    {
        return val_a;
    }
    else
    {
        return val_b;
    }
}

static UHINLINE uint32_t uh_uintmax(uint32_t val_a, uint32_t val_b)
{
    if (val_a > val_b)
    {
        return val_a;
    }
    else
    {
        return val_b;
    }
}

static UHINLINE FLOAT deg2rad(FLOAT a)               { return a * M_PI / 180.0F;  }

static UHINLINE FLOAT rad2deg(FLOAT a)               { return a / M_PI * 180.0F;  }

static UHINLINE FLOAT db2mag(FLOAT db)               { return powf(10.0F, db / 20.0F); }

static UHINLINE FLOAT mag2db(FLOAT f)                { return 20.0F * log10f(f); }


static UHINLINE uint16_t f2qu16(FLOAT f, INT frac_bit)
{
    FLOAT v = f * (1 << frac_bit) + 0.5F;
    return (uint16_t)(v < 0.0F ? 0 : v > 65535.0F ? 65535.0F : v);
}

static UHINLINE int16_t  f2q16(FLOAT f, INT frac_bit)
{
    FLOAT round = f < 0 ? -0.5 : 0.5;
    FLOAT v = f * (1 << frac_bit) + round;
    return (int16_t)(v < -32768.0F ? -32768.0F : v > 32767.0F ? 32767.0F : v);
}

static UHINLINE uint32_t f2qu32(FLOAT f, INT frac_bit)
{
    FLOAT v = f * (1 << frac_bit) + 0.5F;
    return (uint32_t)(v < 0.0F ? 0.0F : v > 4294967295.0F ? 4294967295.0F : v);
}

static UHINLINE int32_t f2q32(FLOAT f, INT frac_bit)
{
    FLOAT round = f < 0 ? -0.5 : 0.5;
    FLOAT v = f * (1 << frac_bit) + round;
    return (int32_t)(v < -2147483648.0F ? -2147483648.0F : v > 2147483647.0F ? 2147483647.0F : v);
}

template <class T> static UHINLINE FLOAT q2f(T v, INT frac_bit)
{
    return (FLOAT)v / (1 << frac_bit);
}


static UHINLINE bool    linear_interpolate_array(     // Returns: true upon success, false upon failure (x not in valid range)
                            FLOAT      &y_interp,   // Output: Interpolated Y value, if successful, else zero
                            FLOAT       x,          // Input:  Fractional X index into Y vector ( 0 <= x <= y_len-1 )
                            FLOAT      *y,          // Input:  Y vector
                            int32_t     y_len)      // Input:  Number of entries in Y vector
{
    y_interp = 0.0F;

    if ((x < 0.0F) || (x > ((FLOAT)y_len - 1.0F)))
    {
        return false;
    }

    FLOAT x1 = floorf(x);
    int32_t x1_int = (int32_t)x1;

    if ((x1_int < 0) || (x1_int > (y_len - 1)))
    {
        return false;
    }

    FLOAT frac = x - x1;
    FLOAT y1 = y[x1_int];

    if (x1_int == (y_len - 1))
    {
        y_interp = y1;
        return true;
    }

    FLOAT y2 = y[x1_int + 1];
    y_interp = y1 + (frac * (y2 - y1));
    return true;
}

static UHINLINE void     quadratic_interpolate(FLOAT &y, FLOAT &x, FLOAT yl, FLOAT yc, FLOAT yu)
{
    if ((yl > yc) && (yl > yu))
    {
        y = yl;
        x = -1.0F;
        return;
    }
    if (yu > yc)
    {
        y = yu;
        x = 1.0F;
        return;
    }

    // X coordinates
    FLOAT xl = -1.0F;
    FLOAT xc =  0.0F;
    FLOAT xu =  1.0F;

    // Quadratic interpolation
    FLOAT d2 = ((yu - yc) / (xu - xc) - (yl - yc) / (xl - xc)) * 2.0F / (xu - xl);
    FLOAT d1 = ((yu - yc) / (xu - xc)) - (d2 * (xu - xc) * 0.5F);

    if (d2 == 0.0F)
    {
        x = xc;
        y = yc;
    }
    else
    {
        x = xc - (d1 / d2);
        y = yc + (d1 * ((x - xc) * 0.5F));
    }
}

static UHINLINE void     quadratic_interpolate_db(cfloat &y, FLOAT &x, cfloat a, cfloat b, cfloat c)
{
    // Values in db
    FLOAT yl = mag2db(a.abs());
    FLOAT yc = mag2db(b.abs());
    FLOAT yu = mag2db(c.abs());

    FLOAT y1;

    quadratic_interpolate(y1, x, yl, yc, yu);

    y = db2mag(y1);
}


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHMATH_H
