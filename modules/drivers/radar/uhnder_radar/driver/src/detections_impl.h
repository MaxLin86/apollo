#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/detections.h"

class ScanObject_Impl;
class ScanSerializer;

struct RangeSort;

class Detections_Impl : public Detections
{
public:

    Detections_Impl(ScanObject_Impl& s)
        : myscan(s)
        , dets(NULL)
        , range_sort(NULL)
        , recv_count(0)
        , num_detections(0)
        , aborted(false)
        , last_err(DET_NO_ERROR)
    {
        allocate();
    }

    virtual ~Detections_Impl();

    virtual uint32_t             get_count() const;

    virtual const DetectionData& get_detection(uint32_t idx) const;

    virtual void                 release();

    virtual Err                  get_last_error() const { return last_err; }

    virtual uint32_t             filter_sidelobes(const SideLobeRegion* regions,
                                                  uint32_t num_regions,
                                                  float max_range_delta);

            void                 try_sidelobe(const DetectionData& curr,
                                              DetectionData& other,
                                              const SideLobeRegion* regions,
                                              uint32_t num_regions);

            void                 allocate();

            void                 handle_uhdp(const DetectionData* msg, uint32_t total_len);

            void                 setup();

            bool                 serialize(ScanSerializer& s) const;

            bool                 deserialize(ScanSerializer& s);

    ScanObject_Impl& myscan;

    DetectionData*   dets;

    RangeSort*       range_sort;

    uint32_t         recv_count;

    uint32_t         num_detections;

    float            ss_half_width;

    bool             aborted;

    mutable Err      last_err;
};
