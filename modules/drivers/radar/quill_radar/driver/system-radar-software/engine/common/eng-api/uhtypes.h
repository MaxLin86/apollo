#ifndef SRS_HDR_UHTYPES_H
#define SRS_HDR_UHTYPES_H 1
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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhmathtypes.h"

SRS_DECLARE_NAMESPACE()

/* Uhnder defined types, adequately defined */

struct cint8
{
    int8_t  i, q;

    cint8() {}

    cint8(const int8_t _i, const int8_t _q) : i(_i), q(_q)   {}

    cint8(const int8_t _i) : i(_i), q(0)   {}

    cint8(const cint8& a) : i(a.i), q(a.q) {}

    UHINLINE uint8_t abs() const
    {
        uint8_t xi = (uint8_t)UH_ABS(i);
        uint8_t xq = (uint8_t)UH_ABS(q);

        return UH_MAX(xi, xq) + (UH_MIN(xi, xq) >> 2);
    }


    UHINLINE cint8 operator +(const cint8& a) const { return cint8(i + a.i, q + a.q); }

    UHINLINE cint8 operator -(const cint8& a) const { return cint8(i - a.i, q - a.q); }

    UHINLINE cint8 operator *(const cint8& a) const
    {
        int8_t mi = i * a.i - q * a.q;
        int8_t mq = i * a.q + q * a.i;

        return cint8(mi, mq);
    }


    UHINLINE cint8 operator /(const cint8& a) const
    {
        int8_t recip = a.i * a.i + a.q * a.q;
        int8_t di = (i * a.i + q * a.q) / recip;
        int8_t dq = (q * a.i - i * a.q) / recip;

        return cint8(di, dq);
    }


    UHINLINE cint8& operator  =(const cint8& a) { i  = a.i; q  = a.q; return *this; }

    UHINLINE cint8& operator +=(const cint8& a) { i += a.i; q += a.q; return *this; }

    UHINLINE cint8& operator -=(const cint8& a) { i -= a.i; q -= a.q; return *this; }

    UHINLINE cint8& operator *=(const cint8& a)
    {
        int8_t mi = (i * a.i - q * a.q);
        int8_t mq = (i * a.q + q * a.i);

        i = mi;
        q = mq;
        return *this;
    }


    UHINLINE cint8& operator /=(const cint8& a)
    {
        int8_t recip = a.i * a.i + a.q * a.q;
        int8_t di = (i * a.i + q * a.q) / recip;
        int8_t dq = (q * a.i - i * a.q) / recip;

        i = di;
        q = dq;
        return *this;
    }


    UHINLINE bool operator <(const cint8& rhs) const { return abs() < rhs.abs(); }

    UHINLINE bool operator >(const cint8& rhs) const { return rhs < *this; }
};

struct cint16_core
{
    int16_t i, q;
};

struct cint16
{
    int16_t i, q;

    cint16() {}

    cint16(const int16_t _i, const int16_t _q) : i(_i), q(_q)   {}

    cint16(const int16_t _i) : i(_i), q(0)   {}

    cint16(const cint16& a) : i(a.i), q(a.q) {}

    cint16(const cint8& a)  : i((int16_t)a.i), q((int16_t)a.q) {}

    UHINLINE uint16_t abs() const
    {
        uint16_t xi = (uint16_t)UH_ABS(i);
        uint16_t xq = (uint16_t)UH_ABS(q);

        return UH_MAX(xi, xq) + (UH_MIN(xi, xq) >> 2);
    }


    UHINLINE cint16 operator +(const cint16& a) const { return cint16(i + a.i, q + a.q); }

    UHINLINE cint16 operator -(const cint16& a) const { return cint16(i - a.i, q - a.q); }

    UHINLINE cint16 operator *(const cint16& a) const
    {
        int16_t mi = i * a.i - q * a.q;
        int16_t mq = i * a.q + q * a.i;

        return cint16(mi, mq);
    }


    UHINLINE cint16 operator /(const cint16& a) const
    {
        int16_t recip = a.i * a.i + a.q * a.q;
        int16_t di = (i * a.i + q * a.q) / recip;
        int16_t dq = (q * a.i - i * a.q) / recip;

        return cint16(di, dq);
    }


    UHINLINE cint16& operator  =(const cint16& a) { i  = a.i; q  = a.q; return *this; }

    UHINLINE cint16& operator +=(const cint16& a) { i += a.i; q += a.q; return *this; }

    UHINLINE cint16& operator -=(const cint16& a) { i -= a.i; q -= a.q; return *this; }

    UHINLINE cint16& operator *=(const cint16& a)
    {
        int16_t mi = (i * a.i - q * a.q);
        int16_t mq = (i * a.q + q * a.i);

        i = mi;
        q = mq;
        return *this;
    }


    UHINLINE cint16& operator /=(const cint16& a)
    {
        int16_t recip = a.i * a.i + a.q * a.q;
        int16_t di = (i * a.i + q * a.q) / recip;
        int16_t dq = (q * a.i - i * a.q) / recip;

        i = di;
        q = dq;
        return *this;
    }


    UHINLINE bool operator <(const cint16& rhs) const { return abs() < rhs.abs(); }

    UHINLINE bool operator >(const cint16& rhs) const { return rhs < *this; }
};

struct cint32
{
    int32_t i, q;

    cint32() {}

    cint32(const int32_t _i, const int32_t _q) : i(_i), q(_q)   {}

    cint32(const int32_t _i) : i(_i), q(0)   {}

    cint32(const cint32& a) : i(a.i), q(a.q) {}

    cint32(const cint8& a)  : i((int32_t)a.i), q((int32_t)a.q) {}

    cint32(const cint16& a) : i((int32_t)a.i), q((int32_t)a.q) {}

    // From https://dspguru.com/dsp/tricks/magnitude-estimator/
    UHINLINE uint32_t abs() const
    {
        uint32_t xi = (uint32_t)UH_ABS(i);
        uint32_t xq = (uint32_t)UH_ABS(q);

        return UH_MAX(xi, xq) + (UH_MIN(xi, xq) >> 2);
    }


    UHINLINE cint32 operator +(const cint32& a) const { return cint32(i + a.i, q + a.q); }

    UHINLINE cint32 operator -(const cint32& a) const { return cint32(i - a.i, q - a.q); }

    UHINLINE cint32 operator *(const cint32& a) const
    {
        int32_t mi = i * a.i - q * a.q;
        int32_t mq = i * a.q + q * a.i;

        return cint32(mi, mq);
    }


    UHINLINE cint32 operator /(const cint32& a) const
    {
        int32_t recip = a.i * a.i + a.q * a.q;
        int32_t di = (i * a.i + q * a.q) / recip;
        int32_t dq = (q * a.i - i * a.q) / recip;

        return cint32(di, dq);
    }


    UHINLINE cint32& operator  =(const cint32& a) { i  = a.i; q  = a.q; return *this; }

    UHINLINE cint32& operator +=(const cint32& a) { i += a.i; q += a.q; return *this; }

    UHINLINE cint32& operator -=(const cint32& a) { i -= a.i; q -= a.q; return *this; }

    UHINLINE cint32& operator *=(const cint32& a)
    {
        int32_t mi = (i * a.i - q * a.q);
        int32_t mq = (i * a.q + q * a.i);

        i = mi;
        q = mq;
        return *this;
    }


    UHINLINE cint32& operator /=(const cint32& a)
    {
        int32_t recip = a.i * a.i + a.q * a.q;
        int32_t di = (i * a.i + q * a.q) / recip;
        int32_t dq = (q * a.i - i * a.q) / recip;

        i = di;
        q = dq;
        return *this;
    }


    UHINLINE bool operator <(const cint32& rhs) const { return abs() < rhs.abs(); }

    UHINLINE bool operator >(const cint32& rhs) const { return rhs < *this; }
};

struct cint64
{
    int64_t i, q;

    cint64() {}

    cint64(const int64_t _i, const int64_t _q) : i(_i), q(_q)   {}

    cint64(const int64_t _i) : i(_i), q(0)   {}

    cint64(const cint64& a) : i(a.i), q(a.q) {}

    cint64(const cint8& a)  : i((int64_t)a.i), q((int64_t)a.q) {}

    cint64(const cint16& a) : i((int64_t)a.i), q((int64_t)a.q) {}

    cint64(const cint32& a) : i((int64_t)a.i), q((int64_t)a.q) {}

    UHINLINE uint64_t abs() const
    {
        uint64_t xi = (uint64_t)UH_ABS(i);
        uint64_t xq = (uint64_t)UH_ABS(q);

        return UH_MAX(xi, xq) + (UH_MIN(xi, xq) >> 2);
    }


    UHINLINE cint64 operator +(const cint64& a) const { return cint64(i + a.i, q + a.q); }

    UHINLINE cint64 operator -(const cint64& a) const { return cint64(i - a.i, q - a.q); }

    UHINLINE cint64 operator *(const cint64& a) const
    {
        int64_t mi = i * a.i - q * a.q;
        int64_t mq = i * a.q + q * a.i;

        return cint64(mi, mq);
    }


    UHINLINE cint64 operator /(const cint64& a) const
    {
        int64_t recip = a.i * a.i + a.q * a.q;
        int64_t di = (i * a.i + q * a.q) / recip;
        int64_t dq = (q * a.i - i * a.q) / recip;

        return cint64(di, dq);
    }


    UHINLINE cint64& operator  =(const cint64& a) { i  = a.i; q  = a.q; return *this; }

    UHINLINE cint64& operator +=(const cint64& a) { i += a.i; q += a.q; return *this; }

    UHINLINE cint64& operator -=(const cint64& a) { i -= a.i; q -= a.q; return *this; }

    UHINLINE cint64& operator *=(const cint64& a)
    {
        int64_t mi = (i * a.i - q * a.q);
        int64_t mq = (i * a.q + q * a.i);

        i = mi;
        q = mq;
        return *this;
    }


    UHINLINE cint64& operator /=(const cint64& a)
    {
        int64_t recip = a.i * a.i + a.q * a.q;
        int64_t di = (i * a.i + q * a.q) / recip;
        int64_t dq = (q * a.i - i * a.q) / recip;

        i = di;
        q = dq;
        return *this;
    }


    UHINLINE bool operator <(const cint64& rhs) const { return abs() < rhs.abs(); }

    UHINLINE bool operator <=(const cint64& rhs) const { return abs() <= rhs.abs(); }

    UHINLINE bool operator >(const cint64& rhs) const { return rhs < *this; }

    UHINLINE bool operator >=(const cint64& rhs) const { return rhs <= *this; }
};

struct cfloat
{
    FLOAT i, q;

    cfloat() {}

    cfloat(const FLOAT _i, const FLOAT _q) : i(_i), q(_q)   {}

    cfloat(const FLOAT _i) : i(_i), q(0)   {}

    cfloat(const cfloat& a) : i(a.i), q(a.q) {}

    cfloat(const cint16& a) : i((FLOAT)a.i), q((FLOAT)a.q) {}

    cfloat(const cint32& a) : i((FLOAT)a.i), q((FLOAT)a.q) {}

    cfloat(const cint64& a) : i((FLOAT)a.i), q((FLOAT)a.q) {}

    UHINLINE FLOAT abssquare() const { return i * i + q * q; }

    UHINLINE FLOAT abs() const { return uh_sqrtf(abssquare()); }

    UHINLINE cfloat operator +(const cfloat& a) const { return cfloat(i + a.i, q + a.q); }

    UHINLINE cfloat operator -(const cfloat& a) const { return cfloat(i - a.i, q - a.q); }

    UHINLINE cfloat operator *(const float& a) const { return cfloat(i * a, q * a); }

    UHINLINE cfloat operator *(const cfloat& a) const
    {
        FLOAT mi = i * a.i - q * a.q;
        FLOAT mq = i * a.q + q * a.i;

        return cfloat(mi, mq);
    }


    UHINLINE cfloat operator /(const float& a) const { return cfloat(i / a, q / a); }

    UHINLINE cfloat operator /(const cfloat& a) const
    {
        FLOAT recip = a.i * a.i + a.q * a.q;
        FLOAT di = (i * a.i + q * a.q) / recip;
        FLOAT dq = (q * a.i - i * a.q) / recip;

        return cfloat(di, dq);
    }


    UHINLINE bool operator ==(const cfloat& a) { return (i == a.i) && (q == a.q); }

    UHINLINE cfloat& operator  =(const cfloat& a) { i  = a.i; q  = a.q; return *this; }

    UHINLINE cfloat& operator +=(const cfloat& a) { i += a.i; q += a.q; return *this; }

    UHINLINE cfloat& operator -=(const cfloat& a) { i -= a.i; q -= a.q; return *this; }

    UHINLINE cfloat& operator *=(const cfloat& a)
    {
        FLOAT mi = (i * a.i - q * a.q);
        FLOAT mq = (i * a.q + q * a.i);

        i = mi;
        q = mq;
        return *this;
    }


    UHINLINE cfloat& operator /=(const cfloat& a)
    {
        FLOAT recip = a.i * a.i + a.q * a.q;
        FLOAT di = (i * a.i + q * a.q) / recip;
        FLOAT dq = (q * a.i - i * a.q) / recip;

        i = di;
        q = dq;
        return *this;
    }


    UHINLINE bool operator <(const cfloat& rhs) const { return abssquare() < rhs.abssquare(); }

    UHINLINE bool operator >(const cfloat& rhs) const { return rhs < *this; }

    UHINLINE FLOAT angle() const { return uh_atan2f(q, i); }

    UHINLINE FLOAT phase() const { return angle(); }

    UHINLINE FLOAT magdB() const { return 10.0F * log10f(abssquare()); }
};

struct vec3f_t
{
    FLOAT x, y, z;

    vec3f_t() {}

    vec3f_t(const FLOAT _x, const FLOAT _y, const FLOAT _z) : x(_x), y(_y), z(_z) {}

    vec3f_t(const FLOAT a) : x(a), y(a), z(a) {}

    vec3f_t(const vec3f_t &a) : x(a.x), y(a.y), z(a.z) {}

    UHINLINE FLOAT abssquare() const { return x * x + y * y + z * z; }

    UHINLINE bool iszero() const { return (x == 0.0F) && (y == 0.0F) && (z == 0.0F); }

    UHINLINE FLOAT abs()       const { return sqrtf(abssquare()); }

    UHINLINE vec3f_t operator +(const vec3f_t& a) const { return vec3f_t(x + a.x, y + a.y, z + a.z); }

    UHINLINE vec3f_t operator -(const vec3f_t& a) const { return vec3f_t(x - a.x, y - a.y, z - a.z); }

    UHINLINE vec3f_t operator *(const vec3f_t& a) const { return vec3f_t(x * a.x, y * a.y, z * a.z); }

    UHINLINE vec3f_t operator /(FLOAT d) const          { d = 1 / d; return vec3f_t(x * d, y * d, z * d); }

    UHINLINE vec3f_t& operator  =(const vec3f_t& a) { x  = a.x; y  = a.y; z  = a.z; return *this; }

    UHINLINE vec3f_t& operator +=(const vec3f_t& a) { x += a.x; y += a.y; z += a.z; return *this; }

    UHINLINE vec3f_t& operator -=(const vec3f_t& a) { x -= a.x; y -= a.y; z -= a.z; return *this; }

    UHINLINE vec3f_t& operator *=(const vec3f_t& a) { x *= a.x; y *= a.y; z *= a.z; return *this; }

    UHINLINE vec3f_t& operator /=(FLOAT d)          { d = 1 / d; x *= d; y *= d; z *= d; return *this; }

    UHINLINE bool operator <(const vec3f_t& rhs) const { return abssquare() < rhs.abssquare(); }

    UHINLINE bool operator >(const vec3f_t& rhs) const { return rhs < *this; }

    UHINLINE FLOAT dot(const vec3f_t& a) { return x * a.x + y * a.y + z * a.z; }
};

struct quatf_t
{
    FLOAT x, y, z, w;

    quatf_t() {}

    quatf_t(const FLOAT _x, const FLOAT _y, const FLOAT _z, const FLOAT _w) : x(_x), y(_y), z(_z), w(_w) {}

    quatf_t(const quatf_t &a) : x(a.x), y(a.y), z(a.z), w(a.w) {}
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHTYPES_H
