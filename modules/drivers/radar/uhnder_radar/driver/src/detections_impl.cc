#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/detections_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include"modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/uhnder_radar/driver/include/serializer.h"

#include <assert.h>
#include <stdlib.h>


Detections_Impl::~Detections_Impl()
{
    delete [] dets;
}


uint32_t             Detections_Impl::get_count() const
{
    return num_detections;
}


const DetectionData& Detections_Impl::get_detection(uint32_t idx) const
{
    if (idx < num_detections)
    {
        last_err = DET_NO_ERROR;
        return dets[idx];
    }

    last_err = DET_INDEX_OUT_OF_RANGE;
    return dets[0];
}


enum { DELETED = 0xFFFF };

struct RangeSort
{
    RangeSort() : inst(NULL) {}

    DetectionData* inst;

    static int sort_compare(const void *A, const void *B)
    {
        float ar = reinterpret_cast<const RangeSort*>(A)->inst->range;
        float br = reinterpret_cast<const RangeSort*>(B)->inst->range;
        return ar > br ? 1 : -1;
    }
};


void Detections_Impl::try_sidelobe(const DetectionData& cur,
                                   DetectionData& check,
                                   const SideLobeRegion* regions,
                                   uint32_t num_regions)
{
    if (fabsf(cur.doppler - check.doppler) > 4 * myscan.scan_info.doppler_bin_width)
        return;

    float delta_mag = cur.snr - check.snr;

    // if this detection is less than 19dB lower than the cur detection, delete
    // it unconditionally regardless of the configured angle ranges and thresholds
    if (17.0 < delta_mag)
    {
        check.flags = DELETED;
        return;
    }

    float delta_az  = rad2deg(cur.azimuth - check.azimuth);
    for (uint32_t i = 0; i < num_regions; i++)
    {
        const SideLobeRegion& r = regions[i];

        if (r.azimuth_delta_max_deg >= delta_az &&
            r.azimuth_delta_min_deg <= delta_az &&
            r.mag_thresh_dB         <= delta_mag)
        {
            check.flags = DELETED;
            return;
        }
    }
}


uint32_t             Detections_Impl::filter_sidelobes(const SideLobeRegion* regions,
                                                       uint32_t num_regions,
                                                       float max_range_delta)
{
    last_err = DET_NO_ERROR;

    if (num_detections < 2 || !num_regions)
    {
        return 0;
    }
    for (uint32_t i = 0; i < num_regions; i++)
    {
        const SideLobeRegion& r = regions[i];

        if (r.azimuth_delta_max_deg < r.azimuth_delta_min_deg)
        {
            last_err = DET_INVALID_AZIMUTH_DELTA_RANGE;
            return 0;
        }
        if ((r.azimuth_delta_max_deg <= 0 && r.azimuth_delta_min_deg >= 0) ||
            (r.azimuth_delta_max_deg >= 0 && r.azimuth_delta_min_deg <= 0))
        {
            last_err = DET_INVALID_AZIMUTH_DELTA_RANGE;
            return 0;
        }
        if (r.mag_thresh_dB < 0)
        {
            last_err = DET_INVALID_MAGNITUDE_THRESHOLD;
            return 0;
        }
    }

    // sort detections by their range
    range_sort = new RangeSort[num_detections];

    for (uint32_t i = 0; i < num_detections; i++)
    {
        range_sort[i].inst = &dets[i];
    }

    qsort(range_sort, num_detections, sizeof(range_sort[0]), RangeSort::sort_compare);

    for (uint32_t i = 0; i < num_detections; i++)
    {
        const DetectionData& cur = *range_sort[i].inst;

        // walk list backwards until end or distance
        for (uint32_t j = i; j > 0; j--)
        {
            DetectionData& check = *range_sort[j - 1].inst;

            if ((cur.range - check.range) < max_range_delta) // out-of-range
            {
                try_sidelobe(cur, check, regions, num_regions);
            }
            else
            {
                break;
            }
        }

        // walk list forwards until end or distance
        for (uint32_t j = i + 1; j < num_detections; j++)
        {
            DetectionData& check = *range_sort[j].inst;

            if ((check.range - cur.range) < max_range_delta) // out-of-range
            {
                try_sidelobe(cur, check, regions, num_regions);
            }
            else
            {
                break;
            }
        }
    }

    delete [] range_sort;

    uint32_t num_deleted = 0;
    for (uint32_t i = 0; i < num_detections; i++)
    {
        while (dets[i].flags == DELETED)
        {
            num_detections--;
            num_deleted++;
            if (num_detections == i)
                break;
            else
                dets[i] = dets[num_detections];
        }
    }

    // Update UhdpScanInformation data in case the scan is re-serialized
    myscan.scan_info.num_detections = num_detections;

    return num_deleted;
}


void                 Detections_Impl::allocate()
{
    num_detections = myscan.scan_info.num_detections;

    if (num_detections)
    {
        dets = new DetectionData[num_detections];
    }
    else
    {
        dets = new DetectionData[1]; // need storage for get_detection()
    }

    ss_half_width = (myscan.scan_info.SS_size_D - 1) / 2;
    ss_half_width += 2;
    ss_half_width *= myscan.scan_info.doppler_bin_width;
}


void                 Detections_Impl::handle_uhdp(const DetectionData* msg, uint32_t total_len)
{
    if (aborted)
    {
        return;
    }

    uint32_t num = total_len / sizeof(DetectionData);

    if (num + recv_count <= myscan.scan_info.num_detections)
    {
        memcpy(dets + recv_count, msg, total_len);
        recv_count += num;
    }
    else
    {
        printf("Too many detections received\n");
        aborted = true;
    }
}


void                 Detections_Impl::setup()
{
    if (aborted)
    {
        release();
        return;
    }
    if (recv_count < myscan.scan_info.num_detections)
    {
        printf("Incomplete detections, discarding\n");
        release();
        return;
    }

    // post network receive work here (build spatial data structs, etc)
}


void                 Detections_Impl::release()
{
    myscan.release_detections(*this);
}


bool                 Detections_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];
    sprintf(fname, "scan_%06d_detectreport.bin", myscan.scan_info.scan_sequence_number);
    bool ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= s.write_scan_data_type(dets, sizeof(DetectionData), get_count());
        s.end_write_scan_data_type(!ok);
    }

    return ok;
}

namespace UhDP21
{
    struct DetectionData
    {
        float                   range;
        float                   azimuth;
        float                   elevation;
        float                   doppler;
        float                   magnitude;
        float                   snr;
        float                   rcs;
        vec3f_t                 position;
        uint32_t                flags;
        uint32_t                reserved[5];
    };
}

bool                 Detections_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];

    sprintf(fname, "scan_%06d_detectreport.bin", myscan.scan_info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len == num_detections * sizeof(DetectionData))
    {
        bool ok = s.read_scan_data_type(dets, sizeof(DetectionData), num_detections);
        s.end_read_scan_data_type();
        return ok;
    }
    else if (len == num_detections * sizeof(UhDP21::DetectionData))
    {
        /* read old DetectionData format */
        bool ok = true;

        UhDP21::DetectionData old;
        for (uint32_t i = 0; i < num_detections; i++)
        {
            ok &= s.read_scan_data_type(&old, sizeof(old), 1);
            if (ok)
            {
                // format is the same as current, up to flags
                memcpy(dets + i, &old, sizeof(dets[0]));
            }
            else
            {
                break;
            }
        }
        s.end_read_scan_data_type();
        return ok;
    }

    return false;
}
