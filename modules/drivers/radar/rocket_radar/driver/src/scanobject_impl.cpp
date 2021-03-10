#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/src/scanobject_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/clutterimage_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/detections_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/staticslice_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/zerodoppler_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/activations_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/musicdata_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/radardatacube1_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/adccapturedata_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/pointcloud_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rhal_out.h"

#include "modules/drivers/radar/rocket_radar/driver/src/tracks_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/include/tracks.h"
#include <assert.h>

#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"

class EnvScanData
{
public:

    EnvScanData(uint32_t id)
        : buffer(NULL)
        , next(NULL)
        , msg_type(id)
        , alloc_size(0)
        , data_size(0)
    {
    }

    ~EnvScanData() { delete [] buffer; }

    void append_data(const char* data, uint32_t bytelen)
    {
        if (!buffer)
        {
            alloc_size = MALLOC_SIZE + bytelen;
            buffer = new char[alloc_size];
        }
        else if (data_size + bytelen > alloc_size)
        {
            alloc_size = MALLOC_SIZE + bytelen + alloc_size;
            char* newbuf = new char[alloc_size];
            memcpy(newbuf, buffer, data_size);
            delete [] buffer;
            buffer = newbuf;
        }

        memcpy(buffer + data_size, data, bytelen);
        data_size += bytelen;
    }

    enum { MALLOC_SIZE = 32 * 1024 };

    char*        buffer;
    EnvScanData* next;
    uint32_t     msg_type;
    uint32_t     alloc_size;
    uint32_t     data_size;
};


EnvScanData* ScanObject_Impl::find_envdata(uint32_t msg_id, bool create)
{
    // if there was any packet drops, we can't know for certain if any of the
    // environment data was valid, so do not return any
    if (!create && env_data_aborted)
        return NULL;

    if (last_envdata && last_envdata->msg_type == msg_id)
        return last_envdata;

    EnvScanData* cur = first_envdata;
    while (cur)
    {
        if (cur->msg_type == msg_id)
            return cur;
        cur = cur->next;
    }

    if (create)
    {
        cur = new EnvScanData(msg_id);
        if (last_envdata)
        {
            last_envdata->next = cur;
            last_envdata = cur;
        }
        else
        {
            first_envdata = last_envdata = cur;
        }

        return cur;
    }

    return NULL;
}

ScanObject_Impl::~ScanObject_Impl()
{
    delete myci;
    delete myss;
    delete mydet;
    delete myzd;
    delete myact;
    delete mypc;
    delete mytracks;
    delete mymusic;
    delete myrdc1;
    delete myadc;
    delete dprobe_out;
    delete adi_measured_dc;
    delete [] completed_aprobes;
    delete [] vrx_mapping;
    delete [] zero_d_bins;
    delete [] hw_coredump_pi;
    delete [] hw_coredump_si;
    delete [] range_bins;
    delete [] angle_bins;
    delete [] histograms;
    delete [] rx_pos;
    delete [] tx_pos;
    delete [] rdc3_blob;

    free(const_cast<char*>(srs_version_str));
    free(const_cast<char*>(module_name));
    free(const_cast<char*>(module_type_name));
    free(const_cast<char*>(motherboard_type_name));
    free(const_cast<char*>(antennaboard_type_name));
    free(const_cast<char*>(antenna_module_type_name));

    while (first_envdata)
    {
        EnvScanData* cur = first_envdata;
        first_envdata = cur->next;
        delete cur;
    }
}


void ScanObject_Impl::handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size)
{
    // all scan data message types will be sent to this function

    switch (message_type)
    {
    case UHDP_TYPE_DPROBE_DC_DATA:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);

        if (total_size >= sizeof(RDC_DProbe_Output) + sizeof(RDC_adi_measured_dc))
        {
            dprobe_out = new RDC_DProbe_Output();
            adi_measured_dc = new RDC_adi_measured_dc();
            memcpy(dprobe_out, payload, sizeof(RDC_DProbe_Output));
            payload += sizeof(RDC_DProbe_Output);
            memcpy(adi_measured_dc, payload, sizeof(RDC_adi_measured_dc));
            payload += sizeof(RDC_adi_measured_dc);

            total_size -= sizeof(RDC_DProbe_Output) + sizeof(RDC_adi_measured_dc);
            num_completed_aprobes = total_size / sizeof(RDC_AprobeRequest);
            if ((num_completed_aprobes > 0) && (total_size == num_completed_aprobes * sizeof(RDC_AprobeRequest)))
            {
                completed_aprobes = new RDC_AprobeRequest[num_completed_aprobes];
                memcpy(completed_aprobes, payload, total_size);
            }
            else
            {
                num_completed_aprobes = 0;
            }
        }
        else
        {
            printf("DProbe/DC Data payload has invalid size %d\n", total_size);
        }
        break;

    case UHDP_TYPE_RANGE_BINS:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);

        if ((total_size / sizeof(UhdpRangeBinInfo)) + num_range_bins_rcvd <= scan_info.num_range_bins)
        {
            if (!range_bins)
            {
                range_bins = new UhdpRangeBinInfo[scan_info.num_range_bins];
            }

            uint32_t count = total_size / sizeof(UhdpRangeBinInfo);
            if (count * sizeof(UhdpRangeBinInfo) == total_size)
            {
                memcpy(range_bins + num_range_bins_rcvd, payload, total_size);
                num_range_bins_rcvd += count;
                return;
            }
        }

        printf("Range Bin Info payload has invalid size %d\n", total_size);
        break;

    case UHDP_TYPE_ANGLE_BINS:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);

        if (uhdp_version < 31)
        {
            if ((total_size / sizeof(UhdpAngleBinInfo071)) + num_angle_bins_rcvd <= scan_info.num_beamforming_angles)
            {
                if (!angle_bins)
                {
                    angle_bins = new UhdpAngleBinInfo[scan_info.num_beamforming_angles];
                }

                uint32_t count = total_size / sizeof(UhdpAngleBinInfo071);
                if (count * sizeof(UhdpAngleBinInfo071) == total_size)
                {
                    for (uint32_t i = 0; i < count; i++)
                    {
                        const UhdpAngleBinInfo071* in = reinterpret_cast<const UhdpAngleBinInfo071*>(payload);
                        angle_bins[num_angle_bins_rcvd].azimuth = in->azimuth;
                        angle_bins[num_angle_bins_rcvd].elevation = in->elevation;
                        angle_bins[num_angle_bins_rcvd].angle_noise_floorQ8 = 0;
                        num_angle_bins_rcvd++;
                    }
                    return;
                }
            }
        }
        else
        {
            if ((total_size / sizeof(UhdpAngleBinInfo)) + num_angle_bins_rcvd <= scan_info.num_beamforming_angles)
            {
                if (!angle_bins)
                {
                    angle_bins = new UhdpAngleBinInfo[scan_info.num_beamforming_angles];
                }

                uint32_t count = total_size / sizeof(UhdpAngleBinInfo);
                if (count * sizeof(UhdpAngleBinInfo) == total_size)
                {
                    memcpy(angle_bins + num_angle_bins_rcvd, payload, total_size);
                    num_angle_bins_rcvd += count;
                    return;
                }
            }
        }

        printf("Angle Bin Info payload has invalid size %d\n", total_size);
        break;

    case UHDP_TYPE_HISTOGRAM:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);
        if (!histograms)
        {
            if (histogram_bytes_received)
            {
                printf("aborted\n");
                break; // aborted
            }
            histograms = new RDC_Histogram[scan_info.num_range_bins];
        }
        if (histogram_bytes_received + total_size <= scan_info.num_range_bins * sizeof(RDC_Histogram))
        {
            memcpy((char*)histograms + histogram_bytes_received, payload, total_size);
            histogram_bytes_received += total_size;
        }
        else
        {
            delete [] histograms;
            histograms = NULL;
        }
        break;

    case UHDP_TYPE_CLUTTER_IMAGE:
        if (!myci)
        {
            myci = new ClutterImage_Impl(*this);
        }
        myci->handle_uhdp((const UhdpClutterImageHeader*)payload, total_size);
        break;

    case UHDP_TYPE_DETECTIONS:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);
        if (!mydet)
        {
            mydet = new Detections_Impl(*this);
        }
        mydet->handle_uhdp((const DetectionData*)payload, total_size);
        break;

    case UHDP_TYPE_STATIC_SLICE_BIN:
        payload    += sizeof(UhdpStaticSliceBinHeader);
        total_size -= sizeof(UhdpStaticSliceBinHeader);
        if (total_size == sizeof(uint16_t) * scan_info.SS_size_A)
        {
            zero_d_bins = new uint16_t[scan_info.SS_size_A];
            memcpy(zero_d_bins, payload, total_size);
        }
        else
        {
            printf("Static slice bins payload has invalid size %d\n", total_size);
        }
        break;

    case UHDP_TYPE_STATIC_SLICE:
        if (!myss)
        {
            myss = new StaticSlice_Impl(*this);
        }
        myss->handle_uhdp((const UhdpStaticSliceHeader*)payload, total_size, uhdp_version);
        break;

    case UHDP_TYPE_ZERO_DOPPLER:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);
        if (!myzd)
        {
            myzd = new ZeroDoppler_Impl(*this);
        }
        myzd->handle_uhdp((uint32_t*)payload, total_size);
        break;

    case UHDP_TYPE_RDC3:
        if (!myact)
        {
            myact = new Activations_Impl(*this);
        }
        myact->handle_uhdp((UhdpRDC3Header*)payload, total_size);
        break;

    case UHDP_TYPE_POINT_CLOUD:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);
        if (!mypc)
        {
            mypc = new PointCloud_Impl(*this);
        }
        mypc->handle_uhdp((PointCloudData*)payload, total_size);
        break;

    case UHDP_TYPE_RDC1:
        if (!myrdc1)
        {
            myrdc1 = new RadarDataCube1_Impl(*this);
        }
        myrdc1->handle_uhdp(payload, total_size);
        break;

    case UHDP_TYPE_RDC1_PLAYBACK:
        if (!myrdc1)
        {
            myrdc1 = new RadarDataCube1_Impl(*this);
        }
        myrdc1->handle_replay_uhdp(payload, total_size);
        break;

    case UHDP_TYPE_RDC2:
    case UHDP_TYPE_COVARIANCE:
    case UHDP_TYPE_SVD:
    case UHDP_TYPE_MUSIC_SAMPLES:
        if (!mymusic)
        {
            mymusic = new MUSICData_Impl(*this);
        }
        mymusic->handle_uhdp(message_type, payload, total_size);
        break;

    case UHDP_TYPE_ADC:
        if (!myadc)
        {
            myadc = new ADCCaptureData_Impl(this);
        }
        myadc->handle_uhdp(message_type, payload, total_size);
        break;

    case UHDP_TYPE_REG_CORE_DUMP:
        {
            UhdpCoreDumpHeader* hdr = (UhdpCoreDumpHeader*)payload;
            payload    += sizeof(UhdpCoreDumpHeader);
            total_size -= sizeof(UhdpCoreDumpHeader);

            if (hdr->proc_or_scan_flag)
            {
                if (!hw_coredump_si)
                {
                    if (hw_coredump_si_total_bytes)
                    {
                        // aborted
                        break;
                    }

                    hw_coredump_si = new char[hdr->total_core_dump_size];
                    hw_coredump_si_total_bytes = hdr->total_core_dump_size;
                    hw_coredump_si_received = 0;
                }

                if (hw_coredump_si_received + total_size <= hw_coredump_si_total_bytes)
                {
                    memcpy(hw_coredump_si + hw_coredump_si_received, payload, total_size);
                    hw_coredump_si_received += total_size;
                }
                else
                {
                    delete [] hw_coredump_si;
                    hw_coredump_si = NULL;
                }
            }
            else
            {
                if (!hw_coredump_pi)
                {
                    if (hw_coredump_pi_total_bytes)
                    {
                        // aborted
                        break;
                    }

                    hw_coredump_pi = new char[hdr->total_core_dump_size];
                    hw_coredump_pi_total_bytes = hdr->total_core_dump_size;
                    hw_coredump_pi_received = 0;
                }

                if (hw_coredump_pi_received + total_size <= hw_coredump_pi_total_bytes)
                {
                    // TODO: we could keep unit data associated with unit IDs in
                    // memory, but it seems writing to file is sufficient
                    memcpy(hw_coredump_pi + hw_coredump_pi_received, payload, total_size);
                    hw_coredump_pi_received += total_size;
                }
                else
                {
                    delete [] hw_coredump_pi;
                    hw_coredump_pi = NULL;
                }
            }
        }
        break;

    case UHDP_TYPE_ENV_SCAN_DATA:
        payload    += sizeof(UhdpDataHeader);
        total_size -= sizeof(UhdpDataHeader);
        uint32_t msg_type = *(uint32_t*)payload;
        payload    += sizeof(uint32_t);
        total_size -= sizeof(uint32_t);
        EnvScanData* env = find_envdata(msg_type, true);
        env->append_data(payload, total_size);
        break;
    }
}


void ScanObject_Impl::abort_uhdp(uint8_t message_type)
{
    switch (message_type)
    {
    case UHDP_TYPE_HISTOGRAM:
        delete [] histograms;
        histograms = NULL;
        break;

    case UHDP_TYPE_CLUTTER_IMAGE:
        if (myci)
        {
            myci->aborted = true;
        }
        break;

    case UHDP_TYPE_DETECTIONS:
        if (mydet)
        {
            mydet->aborted = true;
        }
        break;

    case UHDP_TYPE_STATIC_SLICE:
        if (myss)
        {
            myss->aborted = true;
        }
        break;

    case UHDP_TYPE_ZERO_DOPPLER:
        if (myzd)
        {
            myzd->aborted = true;
        }
        break;

    case UHDP_TYPE_RDC3:
        if (myact)
        {
            myact->aborted = true;
        }
        break;

    case UHDP_TYPE_POINT_CLOUD:
        if (mypc)
        {
            mypc->aborted = true;
        }
        break;

    case UHDP_TYPE_RDC1:
    case UHDP_TYPE_RDC1_PLAYBACK:
        if (myrdc1)
        {
            myrdc1->aborted = true;
        }
        break;

    case UHDP_TYPE_ENV_SCAN_DATA:
        env_data_aborted = true;
        break;

    case UHDP_TYPE_ADC:
        if (myadc)
        {
            myadc->aborted = true;
        }
        break;

    case UHDP_TYPE_REG_CORE_DUMP:
        delete [] hw_coredump_pi;
        hw_coredump_pi = NULL;
        delete [] hw_coredump_si;
        hw_coredump_si = NULL;
        break;

    case UHDP_TYPE_RDC2:
    case UHDP_TYPE_COVARIANCE:
    case UHDP_TYPE_SVD:
    case UHDP_TYPE_MUSIC_SAMPLES:
        if (mymusic)
        {
            mymusic->abort_uhdp(message_type);
        }
        break;

    default:
        // do not care
        break;
    }
}


void ScanObject_Impl::finish_uhdp(uint8_t message_type)
{
    switch (message_type)
    {
    case UHDP_TYPE_RANGE_BINS:
        if (num_range_bins_rcvd != scan_info.num_range_bins)
        {
            delete [] range_bins;
            range_bins = NULL;
        }
        break;

    case UHDP_TYPE_ANGLE_BINS:
        if (num_angle_bins_rcvd != scan_info.num_beamforming_angles)
        {
            delete [] angle_bins;
            angle_bins = NULL;
        }
        break;

    case UHDP_TYPE_HISTOGRAM:
        if (histogram_bytes_received != scan_info.num_range_bins * sizeof(RDC_Histogram))
        {
            delete [] histograms;
            histograms = NULL;
        }
        break;

    case UHDP_TYPE_CLUTTER_IMAGE:
        if (myci)
        {
            myci->setup();
        }
        break;

    case UHDP_TYPE_DETECTIONS:
        if (mydet)
        {
            mydet->setup();
        }
        break;

    case UHDP_TYPE_STATIC_SLICE:
        if (myss)
        {
            myss->setup();
        }
        break;

    case UHDP_TYPE_ZERO_DOPPLER:
        if (myzd)
        {
            myzd->setup();
        }
        break;

    case UHDP_TYPE_RDC3:
        if (myact)
        {
            myact->setup();
        }
        break;

    case UHDP_TYPE_POINT_CLOUD:
        if (mypc)
        {
            mypc->setup();
        }
        break;

    case UHDP_TYPE_RDC2:
    case UHDP_TYPE_COVARIANCE:
    case UHDP_TYPE_SVD:
    case UHDP_TYPE_MUSIC_SAMPLES:
        if (mymusic)
        {
            mymusic->finish_uhdp(message_type);
        }
        break;

    case UHDP_TYPE_RDC1_PLAYBACK:
    case UHDP_TYPE_RDC1:
        if (myrdc1)
        {
            myrdc1->setup();
        }
        break;

    case UHDP_TYPE_ADC:
        if (myadc)
        {
            myadc->finish_uhdp(message_type);
        }
        break;

    case UHDP_TYPE_REG_CORE_DUMP:
        if (hw_coredump_pi_received != hw_coredump_pi_total_bytes)
        {
            delete [] hw_coredump_pi;
            hw_coredump_pi = NULL;
        }
        if (hw_coredump_si_received != hw_coredump_si_total_bytes)
        {
            delete [] hw_coredump_si;
            hw_coredump_si = NULL;
        }
        break;

    default:
        // nop
        break;
    }
}

const RDC_Histogram* ScanObject_Impl::get_histogram(uint32_t range_bin) const
{
    if (histograms)
    {
        return &histograms[range_bin];
    }
    else
    {
        return NULL;
    }
}

const char* ScanObject_Impl::get_custom_data(uint32_t msg_type, uint32_t& length)
{
    EnvScanData* env = find_envdata(msg_type, false);
    if (env)
    {
        length = env->data_size;
        return env->buffer;
    }
    else
    {
        return NULL;
    }
}


ClutterImage*              ScanObject_Impl::get_clutter_image() { return myci; }

Detections*                ScanObject_Impl::get_detections()    { return mydet; }

StaticSlice*               ScanObject_Impl::get_static_slice()  { return myss; }

ZeroDoppler*               ScanObject_Impl::get_zero_doppler_rdc2() { return myzd; }

Activations*               ScanObject_Impl::get_dynamic_activations() { return myact; }

MUSICData*                 ScanObject_Impl::get_music_data() { return mymusic; }

RadarDataCube1*            ScanObject_Impl::get_rdc1() { return myrdc1; }

ADCCaptureData*            ScanObject_Impl::get_adc() { return myadc; }

PointCloud*                ScanObject_Impl::get_point_cloud()
{
    if (!mypc && myci)
    {
        mypc = new PointCloud_Impl(*this);
    }

    return mypc;
}

Tracks*                    ScanObject_Impl::get_tracks()
{
    // If an external tracker is enabled in this build, return an empty tracker
    // even if no tracks were serialized yet
    if (!mytracks)
    {
        mytracks = new Tracks_Impl(*this);
    }

    return mytracks;
}

bool                       ScanObject_Impl::get_completed_aprobe(INT index, RDC_AprobeRequest& request) const
{
    if (index >= 0 && index < INT(num_completed_aprobes))
    {
        request = completed_aprobes[index];
        return true;
    }
    else
    {
        return false;
    }
}

float                      ScanObject_Impl::get_noise_floor_dB(float range) const
{
    uint32_t d = range / scan_info.range_bin_width;

    if (d < scan_info.num_range_bins && range_bins)
    {
        int16_t rbin = range_bins[d].reverse_map;
        if (rbin >= 0)
        {
            return range_bins[rbin].noise_floor_max_peak_dB;
        }
    }

    return 0;
}


float                      ScanObject_Impl::get_azimuth_rad(uint32_t az_bin) const
{
    if (az_bin >= scan_info.num_azimuth_angles)
    {
        last_err = SCAN_INVALID_INPUT;
        return 0.0f;
    }
    if (!angle_bins)
    {
        last_err = SCAN_DATA_NOT_CAPTURED;
        return 0.0f;
    }

    last_err = SCAN_NO_ERROR;

    if (scan_info.num_elevation_angles > 1)
    {
        uint32_t mid_el = scan_info.num_azimuth_angles * (scan_info.num_elevation_angles / 2);
        return angle_bins[mid_el + az_bin].azimuth;
    }
    else
    {
        return angle_bins[az_bin].azimuth;
    }
}

float                      ScanObject_Impl::get_elevation_rad(uint32_t el_bin) const
{
    if (angle_bins && scan_info.num_azimuth_angles)
    {
        if (el_bin >= (uint32_t)scan_info.num_elevation_angles)
        {
            last_err = SCAN_INVALID_INPUT;
            return 0.0f;
        }

        last_err = SCAN_NO_ERROR;
        return angle_bins[el_bin * scan_info.num_azimuth_angles].elevation;
    }
    else
    {
        last_err = SCAN_DATA_NOT_CAPTURED;
        return 0.0f;
    }
}


void                       ScanObject_Impl::get_angle(uint32_t rough_angle, float& azrad, float& elrad) const
{
    if (rough_angle >= scan_info.num_beamforming_angles)
    {
        last_err = SCAN_INVALID_INPUT;
        return;
    }
    if (!angle_bins)
    {
        last_err = SCAN_DATA_NOT_CAPTURED;
        return;
    }

    last_err = SCAN_NO_ERROR;
    azrad = angle_bins[rough_angle].azimuth;
    elrad = angle_bins[rough_angle].elevation;
}

uint8_t                    ScanObject_Impl::get_vrx_mapping(uint32_t vrxid) const
{
    if (vrxid < scan_info.total_vrx && vrx_mapping)
        return vrx_mapping[vrxid];
    return 0;
}

static const vec3f_t null_vec3f(0);

const vec3f_t&             ScanObject_Impl::get_transmitter_position(uint32_t txid) const
{
    if (tx_pos && txid < scan_info.num_tx_prn)
        return tx_pos[txid];
    return null_vec3f;
}

const vec3f_t&             ScanObject_Impl::get_receiver_position(uint32_t rxid) const
{
    if (rx_pos && rxid < (uint32_t)(scan_info.total_vrx / scan_info.num_tx_prn))
        return rx_pos[rxid];
    return null_vec3f;
}

void                       ScanObject_Impl::get_mounting_distances(float& r, float& c, float& height) const
{
    r = rear_axle_distance;
    c = centerline_distance;
    height = mount_height;
}

void                       ScanObject_Impl::get_mounting_angles(float& azimuth_deg, float& elevation_deg) const
{
    azimuth_deg = mount_azimuth;
    elevation_deg = mount_elevation;
}

void                       ScanObject_Impl::release()
{
    delete this;
}


void                       ScanObject_Impl::release_clutter_image(ClutterImage& ci)
{
    assert(&ci == myci);

    delete myci;
    myci = NULL;
}


void                       ScanObject_Impl::release_detections(Detections& det)
{
    assert(&det == mydet);

    delete mydet;
    mydet = NULL;
}


void                       ScanObject_Impl::release_static_slice(StaticSlice& ss)
{
    assert(&ss == myss);

    delete myss;
    myss = NULL;
}


void                       ScanObject_Impl::release_zero_doppler(ZeroDoppler& zd)
{
    assert(&zd == myzd);

    delete myzd;
    myzd = NULL;
}


void                       ScanObject_Impl::release_activations(Activations& act)
{
    assert(&act == myact);

    delete myact;
    myact = NULL;
}


void                       ScanObject_Impl::release_pointcloud(PointCloud& pc)
{
    assert(&pc == mypc);

    delete mypc;
    mypc = NULL;
}


void                       ScanObject_Impl::release_tracks(Tracks& t)
{
    assert(&t == mytracks);

    delete mytracks;
    mytracks = NULL;
}


void                       ScanObject_Impl::release_music(MUSICData& mus)
{
    assert(&mus == mymusic);
    delete mymusic;
    mymusic = NULL;
}


void                       ScanObject_Impl::release_rdc1(RadarDataCube1& rdc1)
{
    assert(&rdc1 == myrdc1);

    delete myrdc1;
    myrdc1 = NULL;
}


void                       ScanObject_Impl::release_adc(ADCCaptureData& adc)
{
    assert(&adc == myadc);

    delete myadc;
    myadc = NULL;
}


void                       ScanObject_Impl::attach_blob(char* blob, uint32_t size_bytes)
{
    if (rdc3_blob)
    {
        delete [] rdc3_blob;
    }
    rdc3_blob = blob;
    rdc3_blob_size_bytes = size_bytes;
}


void                       ScanObject_Impl::save_to_session(const char* session_path) const
{
    SessionFolderScanSerializer s(session_path);
    return serialize(s);
}

void                       ScanObject_Impl::serialize(ScanSerializer& s) const
{
    char fname[128];

    bool ok = s.begin_write_scan(scan_info.scan_sequence_number);
    if (!ok)
    {
        return;
    }

    sprintf(fname, "scan_%06d_info.bin", scan_info.scan_sequence_number);
    ok = s.begin_write_scan_data_type(fname);
    if (ok)
    {
        ok &= serialize_scan_info_bin(s);
        s.end_write_scan_data_type(!ok);
    }

    if (range_bins && ok)
    {
        sprintf(fname, "scan_%06d_rb_info.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(range_bins, sizeof(range_bins[0]), scan_info.num_range_bins);
            s.end_write_scan_data_type(!ok);
        }

        if (uhdp_version < 31)
        {
            sprintf(fname, "scan_%06d_range_bins.bin", scan_info.scan_sequence_number);
            ok &= s.begin_write_scan_data_type(fname);
            if (ok)
            {
                for (uint32_t i = 0; i < scan_info.num_range_bins; i++)
                {
                    ok &= s.write_scan_data_type(&range_bins[i].distance_in_bins, sizeof(int16_t), 1);
                }
                s.end_write_scan_data_type(!ok);
            }
        }
    }

    if (angle_bins && ok)
    {
        sprintf(fname, "scan_%06d_angle_bins.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(angle_bins, sizeof(angle_bins[0]), scan_info.num_beamforming_angles);
            s.end_write_scan_data_type(!ok);
        }
    }

    if (histograms && ok)
    {
        sprintf(fname, "scan_%06d_histograms.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(histograms, sizeof(histograms[0]), scan_info.num_range_bins);
            s.end_write_scan_data_type(!ok);
        }
    }

    if (zero_d_bins && ok)
    {
        sprintf(fname, "scan_%06d_stslice_bins.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(zero_d_bins, sizeof(zero_d_bins[0]), scan_info.SS_size_A);
            s.end_write_scan_data_type(!ok);
        }
    }

    if (dprobe_out && adi_measured_dc && ok)
    {
        sprintf(fname, "scan_%06d_measurements.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(dprobe_out, sizeof(dprobe_out[0]), 1);
            ok &= s.write_scan_data_type(adi_measured_dc, sizeof(adi_measured_dc[0]), 1);
            if (num_completed_aprobes && completed_aprobes)
            {
                ok &= s.write_scan_data_type(completed_aprobes, sizeof(completed_aprobes[0]), num_completed_aprobes);
            }
            s.end_write_scan_data_type(!ok);
        }
    }

    if (rdc3_blob && ok)
    {
        sprintf(fname, "scan_%06d_sparse_rdc3.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(rdc3_blob, rdc3_blob_size_bytes, 1);
            s.end_write_scan_data_type(!ok);
        }
    }

    if (mydet && ok)
    {
        ok &= mydet->serialize(s);
    }
    if (myci && ok)
    {
        ok &= myci->serialize(s);
    }
    if (myss && ok)
    {
        ok &= myss->serialize(s);
    }
    if (myzd && ok)
    {
        ok &= myzd->serialize(s);
    }
    if (myact && ok)
    {
        ok &= myact->serialize(s);
    }
    if (mypc && ok)
    {
        ok &= mypc->serialize(s);
    }
    if (mytracks && ok)
    {
        ok &= mytracks->serialize(s);
    }
    if (mymusic && ok)
    {
        ok &= mymusic->serialize(s);
    }
    if (myrdc1 && ok)
    {
        ok &= myrdc1->serialize(s);
    }
    if (myadc && ok)
    {
        ok &= myadc->serialize(s);
    }

    if (hw_coredump_pi && ok)
    {
        sprintf(fname, "scan_%06d_coredump_pi.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(hw_coredump_pi, sizeof(hw_coredump_pi[0]), hw_coredump_pi_total_bytes);
            s.end_write_scan_data_type(!ok);
        }
    }
    if (hw_coredump_si && ok)
    {
        sprintf(fname, "scan_%06d_coredump_si.bin", scan_info.scan_sequence_number);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(hw_coredump_si, sizeof(hw_coredump_si[0]), hw_coredump_si_total_bytes);
            s.end_write_scan_data_type(!ok);
        }
    }

    EnvScanData* cur = first_envdata;
    while (cur && !env_data_aborted && ok)
    {
        sprintf(fname, "scan_%06d_env_data_%d.bin", scan_info.scan_sequence_number, cur->msg_type);
        ok &= s.begin_write_scan_data_type(fname);
        if (ok)
        {
            ok &= s.write_scan_data_type(cur->buffer, cur->data_size, 1);
            s.end_write_scan_data_type(!ok);
        }

        cur = cur->next;
    }

    s.end_write_scan(!ok);
}


ScanObject* ScanObject::load_from_session(const char* session_path, uint32_t scan_sequence_number)
{
    SessionFolderScanSerializer s(session_path);
    return deserialize(s, scan_sequence_number);
}


ScanObject* ScanObject::deserialize(ScanSerializer& s, uint32_t scan_sequence_number)
{
    char fname[128];

    if (!s.begin_read_scan(scan_sequence_number))
        return NULL;

    sprintf(fname, "scan_%06d_info.bin", scan_sequence_number);
    ScanObject_Impl* scan = NULL;

    // load the scan info, which *must* be present
    size_t len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        scan = new ScanObject_Impl();
        bool format_075 = false;
        bool ok = scan->deserialize_scan_info_bin(s, len, format_075);
        s.end_read_scan_data_type();

        if (!ok && format_075)
        {
            // retry with the 075 sized UhdpScanInformation struct
            len = s.begin_read_scan_data_type(fname);
            ok = scan->deserialize_scan_info_bin(s, len, format_075);
            s.end_read_scan_data_type();
        }

        if (ok)
        {
            // try to load all remaining data types, using class method
            scan->deserialize(s);
        }
        else
        {
            delete scan;
            scan = NULL;
        }
    }
    else
    {
        s.end_read_scan_data_type();

        sprintf(fname, "scan_%06d_info.json", scan_sequence_number);
        len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            scan = new ScanObject_Impl();
            bool ok = scan->deserialize_scan_info(s, len);
            s.end_read_scan_data_type();

            if (ok)
            {
                scan->deserialize(s);
            }
            else
            {
                delete scan;
                scan = NULL;
            }
        }
        else
        {
            s.end_read_scan_data_type();
        }
    }

    s.end_read_scan();
    return scan;
}


void        ScanObject_Impl::deserialize(ScanSerializer& s)
{
    char fname[128];

    // opportunistically load all available scan data
    sprintf(fname, "scan_%06d_rb_info.bin", scan_info.scan_sequence_number);
    size_t len = s.begin_read_scan_data_type(fname);
    if (len == sizeof(range_bins[0]) * scan_info.num_range_bins)
    {
        range_bins = new UhdpRangeBinInfo[scan_info.num_range_bins];
        s.read_scan_data_type(range_bins, sizeof(range_bins[0]), scan_info.num_range_bins);
    }
    s.end_read_scan_data_type();

    sprintf(fname, "scan_%06d_range_bins.bin", scan_info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len == sizeof(uint16_t) * scan_info.num_range_bins)
    {
        if (range_bins)
        {
            int16_t ignored;
            for (uint32_t i = 0; i < scan_info.num_range_bins; i++)
            {
                s.read_scan_data_type(&ignored, sizeof(int16_t), 1);
            }
        }
        else
        {
            range_bins = new UhdpRangeBinInfo[scan_info.num_range_bins];
            for (uint32_t i = 0; i < scan_info.num_range_bins; i++)
            {
                s.read_scan_data_type(&range_bins[i].distance_in_bins, sizeof(int16_t), 1);
                range_bins[i].noise_floor_max_peak_dB = 0;
                range_bins[i].exponent = 0;
                range_bins[i].reserved = 0;
            }
        }
    }
    s.end_read_scan_data_type();

    if (range_bins && uhdp_version < 31)
    {
        // discard previously stored but unused noise_floor_pfa_dB field
        for (uint32_t i = 0; i < scan_info.num_range_bins; i++)
        {
            range_bins[i].exponent = 0;
            range_bins[i].reserved = 0;
        }
    }

    sprintf(fname, "scan_%06d_angle_bins.bin", scan_info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len == sizeof(angle_bins[0]) * scan_info.num_beamforming_angles)
    {
        angle_bins = new UhdpAngleBinInfo[scan_info.num_beamforming_angles];
        s.read_scan_data_type(angle_bins, sizeof(angle_bins[0]), scan_info.num_beamforming_angles);
    }
    else if ((uhdp_version < 31) && (len == scan_info.num_beamforming_angles * sizeof(UhdpAngleBinInfo071)))
    {
        angle_bins = new UhdpAngleBinInfo[scan_info.num_beamforming_angles];
        for (uint32_t i = 0; i < scan_info.num_beamforming_angles; i++)
        {
            UhdpAngleBinInfo071 ang;
            s.read_scan_data_type(&ang, sizeof(ang), 1);
            angle_bins[i].azimuth = ang.azimuth;
            angle_bins[i].elevation = ang.elevation;
            angle_bins[i].angle_noise_floorQ8 = 0;
        }
    }
    s.end_read_scan_data_type();

    sprintf(fname, "scan_%06d_histograms.bin", scan_info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len == sizeof(histograms[0]) * scan_info.num_range_bins)
    {
        histograms = new RDC_Histogram[scan_info.num_range_bins];
        s.read_scan_data_type(histograms, sizeof(histograms[0]), scan_info.num_range_bins);
    }
    s.end_read_scan_data_type();

    sprintf(fname, "scan_%06d_measurements.bin", scan_info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len >= sizeof(RDC_DProbe_Output) + sizeof(RDC_adi_measured_dc))
    {
        dprobe_out = new RDC_DProbe_Output();
        adi_measured_dc = new RDC_adi_measured_dc();
        s.read_scan_data_type(dprobe_out, sizeof(RDC_DProbe_Output), 1);
        s.read_scan_data_type(adi_measured_dc, sizeof(RDC_adi_measured_dc), 1);

        len -= sizeof(RDC_DProbe_Output) + sizeof(RDC_adi_measured_dc);
        num_completed_aprobes = len / sizeof(RDC_AprobeRequest);
        if ((num_completed_aprobes > 0) && (len == num_completed_aprobes * sizeof(RDC_AprobeRequest)))
        {
            completed_aprobes = new RDC_AprobeRequest[num_completed_aprobes];
            s.read_scan_data_type(completed_aprobes, sizeof(completed_aprobes[0]), num_completed_aprobes);
        }
        else
        {
            num_completed_aprobes = 0;
        }
    }

    mydet = new Detections_Impl(*this);
    if (!mydet->deserialize(s))
    {
        mydet->release();
    }

    myadc = new ADCCaptureData_Impl(this);
    if (!myadc->deserialize(s))
    {
        myadc->release();
    }

    if (angle_bins && range_bins)
    {
        sprintf(fname, "scan_%06d_stslice_bins.bin", scan_info.scan_sequence_number);
        len = s.begin_read_scan_data_type(fname);
        if (len == sizeof(zero_d_bins[0]) * scan_info.SS_size_A)
        {
            zero_d_bins = new uint16_t[scan_info.SS_size_A];
            s.read_scan_data_type(zero_d_bins, sizeof(zero_d_bins[0]), scan_info.SS_size_A);
        }
        s.end_read_scan_data_type();
        if (zero_d_bins)
        {
            myss = new StaticSlice_Impl(*this);
            if (!myss->deserialize(s))
            {
                myss->release();
            }
        }
        myci = new ClutterImage_Impl(*this);
        if (!myci->deserialize(s))
        {
            myci->release();
        }
        myact = new Activations_Impl(*this);
        if (!myact->deserialize(s))
        {
            myact->release();
        }
        mypc = new PointCloud_Impl(*this);
        if (!mypc->deserialize(s))
        {
            mypc->release();
        }
    }

    if (range_bins)
    {
        mymusic = new MUSICData_Impl(*this);
        if (!mymusic->deserialize(s))
        {
            mymusic->release();
        }
        myzd = new ZeroDoppler_Impl(*this);
        if (!myzd->deserialize(s))
        {
            myzd->release();
        }
    }

    myrdc1 = new RadarDataCube1_Impl(*this);
    if (!myrdc1->deserialize(s))
    {
        myrdc1->release();
    }

    sprintf(fname, "scan_%06d_sparse_rdc3.bin", scan_info.scan_sequence_number);
    len = s.begin_read_scan_data_type(fname);
    if (len)
    {
        rdc3_blob_size_bytes = len;
        rdc3_blob = new char[len];
        s.read_scan_data_type(const_cast<char*>(rdc3_blob), rdc3_blob_size_bytes, 1);
        s.end_read_scan_data_type();
    }

    mytracks = new Tracks_Impl(*this);
    if (!mytracks->deserialize(s))
    {
        mytracks->release();
    }

    // Note: since there are no rdc1 or core-dump getter methods on the scan
    // object, we do not bother reading either

    for (uint32_t i = 0; i < 64; i++)
    {
        sprintf(fname, "scan_%06d_env_data_%d.bin", scan_info.scan_sequence_number, i);
        len = s.begin_read_scan_data_type(fname);
        if (len)
        {
            EnvScanData* env = find_envdata(i, true);
            delete [] env->buffer;
            env->buffer = new char[len];
            env->alloc_size = env->data_size = len;

            s.read_scan_data_type(env->buffer, len, 1);
            s.end_read_scan_data_type();
        }
    }
}


