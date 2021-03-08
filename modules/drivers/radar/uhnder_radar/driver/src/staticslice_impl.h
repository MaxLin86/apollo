#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/staticslice.h"

class ScanObject_Impl;
class ScanSerializer;

class StaticSlice_Impl: public StaticSlice
{
public:

    StaticSlice_Impl(ScanObject_Impl& s)
        : myscan(s)
        , last_err(SS_NO_ERROR)
        , ss(NULL)
        , rexp(NULL)
        , aborted(false)
        , skewer_size(0)
        , cur_doppler_bin(0)
        , cur_range_bin(0)
    {
        allocate();
    }

    virtual ~StaticSlice_Impl();

    virtual uint32_t             get_num_slices() const;

    virtual uint32_t             get_num_angle_bins() const;

    virtual float                get_slice_velocity(uint32_t slice) const;

    virtual const uint16_t*      get_skewer(uint32_t range_bin, uint32_t slice) const;

    virtual int16_t              get_exponent_at_range(uint32_t rbin) const;

    virtual void                 release();

    virtual Err                  get_last_error() const { return last_err; }

            void                 allocate();

            void                 handle_uhdp(const UhdpStaticSliceHeader* msg, uint32_t total_size);

            void                 setup();

            bool                 serialize(ScanSerializer& s) const;

            bool                 deserialize(ScanSerializer& s);

    ScanObject_Impl& myscan;

    mutable Err      last_err;

    uint16_t*        ss;

    int16_t*         rexp;

    bool             aborted;

    char*            write_ptr;

    uint32_t         skewer_size;

    uint32_t         cur_doppler_bin;

    uint32_t         cur_range_bin;
};
