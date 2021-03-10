// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHMATH_H
#define SRS_HDR_UHMATH_H 1


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhmathtypes.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhstdlib.h"

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

static UHINLINE FLOAT db2mag(FLOAT db)               { return powf(10.0F, db * 0.05F); }    // use * 0.05 instead of / 20.0 for speed

static UHINLINE FLOAT mag2db(FLOAT f)                { return 20.0F * log10f(f); }

static UHINLINE FLOAT rfa_to_db(FLOAT rfa, size_t num_cells) { return mag2db(uh_sqrtf(-2 * logf(float(double(rfa) / double(num_cells))))); }

static UHINLINE int32_t uh_lroundf(FLOAT f)
{
    return (f < 0) ? (-(int32_t)(-(f) + 0.5F)) : ((int32_t)((f) + 0.5F));
}

static UHINLINE uint16_t f2qu16(FLOAT f, INT frac_bit)
{
    FLOAT v = f * float(1 << frac_bit) + 0.5F;
    return (uint16_t)(v < 0.0F ? 0 : v > 65535.0F ? 65535.0F : v);
}

static UHINLINE int16_t  f2q16(FLOAT f, INT frac_bit)
{
    FLOAT round = f < 0 ? -0.5F : 0.5F;
    FLOAT v = f * float(1 << frac_bit) + round;
    return (int16_t)(v < -32768.0F ? -32768.0F : v > 32767.0F ? 32767.0F : v);
}

static UHINLINE uint32_t f2qu32(FLOAT f, INT frac_bit)
{
    FLOAT v = f * float(1 << frac_bit) + 0.5F;
    return (uint32_t)(v < 0.0F ? 0.0F : v > 4294967295.0F ? 4294967295.0F : v);
}

static UHINLINE int32_t f2q32(FLOAT f, INT frac_bit)
{
    FLOAT round = f < 0 ? -0.5F : 0.5F;
    FLOAT v = f * float(1 << frac_bit) + round;
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

    FLOAT x1 = uh_floorf(x);
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
    FLOAT yucxuc = (yu - yc) / (xu - xc);
    FLOAT d2 = (yucxuc - (yl - yc) / (xl - xc)) * 2.0F / (xu - xl);
    FLOAT d1 =  yucxuc - (d2 * (xu - xc) * 0.5F);

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


uint16_t ransac(                         // Return: number of inliers (static targets)
                FLOAT*      best_model,  // Output: best model
                FLOAT*      data,        // Input:  data array
                uint16_t    n_data,      // Input:  number of data (detections)
                FLOAT*      initial,     // Input:  initial conditions
                FLOAT       swing,       // Input:  max deviation from initial conditions
                uint16_t    n_sample,    // Input:  number of samples selected randomly (not used now, taking 2 samples)
                uint16_t    iteration,   // Input:  number of iterations
                FLOAT       threshold);  // Input:  residual threshold (0.005)

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHMATH_H
