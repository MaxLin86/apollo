#pragma once

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-threshctrl.h"
#include "modules/drivers/radar/quill_radar/driver/include/scanobject.h"

#ifdef _MSC_VER
#include <Windows.h>
#ifndef PATH_MAX
#define PATH_MAX MAX_PATH
#endif
#else
#include <limits.h>
#endif

class ClutterImage_Impl;
class Detections_Impl;
class StaticSlice_Impl;
class ZeroDoppler_Impl;
class Activations_Impl;
class PointCloud_Impl;
class Tracks_Impl;
class MUSICData_Impl;
class RadarDataCube1_Impl;
class ADCCaptureData_Impl;
class EnvScanData;

// This enum should be at least as large as the maximum number of beamforming
// angles of any radar that RRA communicate with
enum { MAX_MAX_ROUGH_ANGLES = 384 };

class ScanObject_Impl: public ScanObject
{
public:

    ScanObject_Impl()
        : last_err(SCAN_NO_ERROR)
        , myci(NULL)
        , mydet(NULL)
        , myss(NULL)
        , myzd(NULL)
        , myact(NULL)
        , mypc(NULL)
        , mytracks(NULL)
        , mymusic(NULL)
        , myrdc1(NULL)
        , myadc(NULL)
        , srs_version_str(NULL)
        , module_name(NULL)
        , module_type_name(NULL)
        , motherboard_type_name(NULL)
        , antennaboard_type_name(NULL)
        , antenna_module_type_name(NULL)
        , rdc3_blob(NULL)
        , vrx_mapping(NULL)
        , zero_d_bins(NULL)
        , hw_coredump_pi(NULL)
        , hw_coredump_si(NULL)
        , rdc1_playback_data(NULL)
        , range_bins(NULL)
        , angle_bins(NULL)
        , histograms(NULL)
        , dprobe_out(NULL)
        , adi_measured_dc(NULL)
        , completed_aprobes(NULL)
        , first_envdata(NULL)
        , last_envdata(NULL)
        , next_scan_object(NULL)
        , rx_pos(NULL)
        , tx_pos(NULL)
        , num_range_bins_rcvd(0)
        , num_angle_bins_rcvd(0)
        , histogram_bytes_received(0)
        , hw_coredump_pi_total_bytes(0)
        , hw_coredump_pi_received(0)
        , hw_coredump_si_total_bytes(0)
        , hw_coredump_si_received(0)
        , rdc1_playback_received(0)
        , rdc1_playback_total_bytes(0)
        , num_completed_aprobes(0)
        , rdc3_blob_size_bytes(0)
        , rear_axle_distance(3.91f)
        , centerline_distance(0)
        , mount_height(0.7f)
        , mount_azimuth(0)
        , mount_elevation(0)
        , env_data_aborted(false)
        , rdc1_aborted(false)
        , uhdp_version(0)
    {
        memset(&host_local_time, 0, sizeof(host_local_time));
        mythresh.defaults();
    }

    virtual                           ~ScanObject_Impl();

    virtual const UhdpScanInformation& get_scan_info() const         { return scan_info; }

    virtual const char*                get_scan_info_as_json() const;

    virtual Err                        get_last_error() const        { return last_err; }

    virtual ClutterImage*              get_clutter_image();

    virtual Detections*                get_detections();

    virtual StaticSlice*               get_static_slice();

    virtual ZeroDoppler*               get_zero_doppler_rdc2();

    virtual Activations*               get_dynamic_activations();

    virtual PointCloud*                get_point_cloud();

    virtual MUSICData*                 get_music_data();

    virtual RadarDataCube1*            get_rdc1();

    virtual ADCCaptureData*            get_adc();

    virtual Tracks*                    get_tracks();

    virtual const RDC_ThresholdControl* get_scan_thresholds() const    { return &mythresh; }

    virtual const timeval&             get_scan_end_local_time() const { return host_local_time; }

    virtual const char*                get_radar_software_version() const { return srs_version_str; }

    virtual const char*                get_module_name() const { return module_name; }

    virtual const char*                get_module_type_name() const { return module_type_name; }

    virtual const char*                get_motherboard_type_name() const { return motherboard_type_name; }

    virtual const char*                get_antennaboard_type_name() const { return antennaboard_type_name; }

    virtual const char*                get_antenna_module_type_name() const { return antenna_module_type_name; }

    virtual const char*                get_custom_data(uint32_t msg_type, uint32_t& length);

    virtual const UhdpRangeBinInfo*    get_range_bins() const        { return range_bins; }

    virtual float                      get_noise_floor_dB(float range) const;

    virtual const RDC_Histogram*       get_histogram(uint32_t range_bin) const;

    virtual const RDC_DProbe_Output*   get_dprobe_outputs() const { return dprobe_out; }

    virtual const RDC_adi_measured_dc* get_measured_dc() const { return adi_measured_dc; }

    virtual INT                        get_num_completed_aprobes() const { return num_completed_aprobes; }

    virtual bool                       get_completed_aprobe(INT index, RDC_AprobeRequest& request) const;

    virtual const uint16_t*            get_zero_doppler_bins() const { return zero_d_bins; }

    virtual const char*                get_sparse_rdc3_blob(size_t& length) { length = size_t(rdc3_blob_size_bytes); return rdc3_blob; }

    virtual float                      get_azimuth_rad(uint32_t az_bin) const;

    virtual float                      get_elevation_rad(uint32_t el_bin) const;

    virtual void                       get_angle(uint32_t rough_angle, float& azrad, float& elrad) const;

    virtual uint8_t                    get_vrx_mapping(uint32_t vrxid) const;

    virtual const vec3f_t&             get_transmitter_position(uint32_t txid) const;

    virtual const vec3f_t&             get_receiver_position(uint32_t rxid) const;

    virtual void                       get_mounting_distances(float& rear_axle_dist, float& centerline_dist, float& height) const;

    virtual void                       get_mounting_angles(float& azimuth_deg, float& elevation_deg) const;

    virtual void                       save_to_session(const char* session_path) const;

    virtual void                       serialize(ScanSerializer& s) const;

    virtual void                       serialize_scan_info_json(ScanSerializer& s) const;

    virtual void                       release();

            void                       deserialize(ScanSerializer& s);

            void                       release_clutter_image(ClutterImage& ci);

            void                       release_detections(Detections& det);

            void                       release_static_slice(StaticSlice& ss);

            void                       release_zero_doppler(ZeroDoppler& zd);

            void                       release_activations(Activations& act);

            void                       release_pointcloud(PointCloud& act);

            void                       release_tracks(Tracks& t);

            void                       release_music(MUSICData& mus);

            void                       release_rdc1(RadarDataCube1& rdc1);

            void                       release_adc(ADCCaptureData& adc);

            void                       handle_scan_info(const UhdpScanInformation* msg, uint32_t total_size, const timeval& ht, uint32_t uhdp_ver);

            void                       handle_uhdp(uint8_t message_type, const char* payload, uint32_t total_size);

            void                       abort_uhdp(uint8_t last_message_type);

            void                       finish_uhdp(uint8_t last_message_type);

            bool                       deserialize_scan_info(ScanSerializer& s, size_t bytes);

            bool                       serialize_scan_info(ScanSerializer& s) const;

            bool                       deserialize_scan_info_bin(ScanSerializer& s, size_t bytes);

            bool                       serialize_scan_info_bin(ScanSerializer& s) const;

            EnvScanData*               find_envdata(uint32_t msg_id, bool create);

            void                       attach_blob(char* blob, uint32_t size_bytes);

    UhdpScanInformation scan_info;

    mutable Err         last_err;

    ClutterImage_Impl*  myci;

    Detections_Impl*    mydet;

    StaticSlice_Impl*   myss;

    ZeroDoppler_Impl*   myzd;

    Activations_Impl*   myact;

    PointCloud_Impl*    mypc;

    Tracks_Impl*        mytracks;

    MUSICData_Impl*     mymusic;

    RadarDataCube1_Impl* myrdc1;

    ADCCaptureData_Impl* myadc;

    RDC_ThresholdControl mythresh;

    timeval             host_local_time;

    const char*         srs_version_str;

    const char*         module_name;

    const char*         module_type_name;

    const char*         motherboard_type_name;

    const char*         antennaboard_type_name;

    const char*         antenna_module_type_name;

    const char*         rdc3_blob;

    uint8_t*            vrx_mapping;

    uint16_t*           zero_d_bins;

    char*               hw_coredump_pi;

    char*               hw_coredump_si;

    char*               rdc1_playback_data;

    UhdpRangeBinInfo*   range_bins;

    UhdpAngleBinInfo*   angle_bins;

    RDC_Histogram*      histograms;

    RDC_DProbe_Output*  dprobe_out;

    RDC_adi_measured_dc* adi_measured_dc;

    RDC_AprobeRequest*  completed_aprobes;

    EnvScanData*        first_envdata;

    EnvScanData*        last_envdata;

    ScanObject_Impl*    next_scan_object; // singly linked list for Conn

    vec3f_t*            rx_pos;

    vec3f_t*            tx_pos;

    uint32_t            num_range_bins_rcvd;

    uint32_t            num_angle_bins_rcvd;

    uint32_t            histogram_bytes_received;

    uint32_t            hw_coredump_pi_total_bytes;

    uint32_t            hw_coredump_pi_received;

    uint32_t            hw_coredump_si_total_bytes;

    uint32_t            hw_coredump_si_received;

    uint32_t            rdc1_playback_received;

    uint32_t            rdc1_playback_total_bytes;

    uint32_t            num_completed_aprobes;

    uint32_t            rdc3_blob_size_bytes;

    float               rear_axle_distance;

    float               centerline_distance;

    float               mount_height;

    float               mount_azimuth;

    float               mount_elevation;

    bool                env_data_aborted;

    bool                rdc1_aborted;

    uint8_t             uhdp_version;
};
