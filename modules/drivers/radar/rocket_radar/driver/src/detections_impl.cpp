#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/detections_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/uhmath.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"

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
