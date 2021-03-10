#ifndef SRS_HDR_UHNDER_HELPERS_H
#define SRS_HDR_UHNDER_HELPERS_H 1
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

#define REPORT_TO_BITS_TRUNCATE 0

SRS_DECLARE_NAMESPACE()

/*************************************
 * C++ only function definitions
 ************************************/

#if __cplusplus

template<class T>
T* from_address(const uint32_t addr)
{
    return reinterpret_cast<T*>(addr); //lint !e923 !e925 !e930
}


template<class T>
T* from_charptr(int8_t* const addr)
{
    return reinterpret_cast<T*>(addr); //lint !e9176
}


template<class T>
int8_t* to_charptr(T* const addr)
{
    return reinterpret_cast<int8_t*>(addr); //lint !e9176
}


template<class T>
const int8_t* to_charptr(const T* const addr)
{
    return reinterpret_cast<const int8_t*>(addr); //lint !e9176
}

template<class T>
T* from_voidptr(void* addr)
{
    return reinterpret_cast<T*>(addr); //lint !e9176
}

template<uint32_t bits>
uint32_t to_bits(uint32_t value)
{
    uint32_t mask = (1UL << bits) - 1;

#if REPORT_TO_BITS_TRUNCATE
    if ((mask & value) != value)
    {
        UHPRINTF("to_bits<%d> is truncating %x to %x\n", bits, value, mask & value);
    }
#endif

    return value & mask;
}

template<uint32_t bits>
int32_t sign_extend(uint32_t x)
{
    int32_t r;
    r = (int32_t)(x << (32 - bits));
    r =           r >> (32 - bits);

    return r;
}


#if defined(offsetof) /* gtest includes stddef.h, which defines offsetof */
#undef offsetof
#endif

template<class P, class M>
size_t offsetof(const M P::*member) { return (size_t)&(reinterpret_cast<P*>(0)->*member); }

/* see code examples for usage, or see also:
 * https://en.wikipedia.org/wiki/Offsetof
 * http://shimpossible.blogspot.com/2013/08/containerof-and-offsetof-in-c.html */

template<class P, class M>
P* container_of(M* ptr, const M P::*member) { return (P*)((CHAR*)ptr - offsetof(member)); }

/* compile-time static assert, used like this:
 *
 *    StaticAssert<sizeof(foo) == 16>::istrue();
 *
 * if the condition is false at compile time, you will get this compiler error:
 *
 *    no member named 'istrue' in 'StaticAssert<false>';
 *
 * static assertions have no impact on the resulting binary, no code is emitted
 * by the compiler. so use them liberally to verify the compile environment */
template<bool b>
struct StaticAssert {};

template<>
struct StaticAssert<true>{ static void istrue() {}
};

template<class T>
T uh_iabs(T val)
{
    // properly handle the saturation cases
    if (val == T(1ULL << (8 * sizeof(T) - 1U)))
    {
        return ~val;
    }
    else
    {
        return val < 0 ? -val : val;
    }
}


#endif // if __cplusplus

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHNDER_HELPERS_H
