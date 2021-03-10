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
#pragma once

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhinet.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhunistd.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/uhtypes.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/uhdp-msg-structs.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/scp-src/eng-api/rdc.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-common.h"
#include "modules/drivers/radar/quill_radar/driver/include/rra.h"
#include <time.h>

class ClutterImage;
class Detections;
class StaticSlice;
class ZeroDoppler;
class Activations;
class PointCloud;
class Tracks;
class MUSICData;
class RadarDataCube1;
class ScanSerializer;
class ADCCaptureData;

class ScanObject
{
public:

    virtual ~ScanObject() {}

    //! returns the scan information pertaining to the scan
    virtual const UhdpScanInformation& get_scan_info() const = 0;

    //! returns the scan information pertaining to the scan in JSON string
    //! format. caller must delete [] the returned string
    virtual const char*                get_scan_info_as_json() const = 0;

    //! might return NULL if a clutter image was not captured
    virtual ClutterImage*              get_clutter_image() = 0;

    //! might return NULL if a detections were not captured
    virtual Detections*                get_detections() = 0;

    virtual Tracks*                    get_tracks() = 0;

    //! might return NULL if static slice was not captured
    virtual StaticSlice*               get_static_slice() = 0;

    //! might return NULL if zero doppler RDC2 was not captured
    virtual ZeroDoppler*               get_zero_doppler_rdc2() = 0;

    //! might return NULL if RDC3 dynamic activations were not captured
    virtual Activations*               get_dynamic_activations() = 0;

    //! might return NULL if point cloud data was not captured. In 0.5.4 this
    //! requires DL_RDC3 and/or DL_SS to both be captured. In a future release the
    //! scan descriptor 'point_cloud' flag will be respected.
    virtual PointCloud*                get_point_cloud() = 0;

    //! might return NULL if no MUSIC data were captured
    virtual MUSICData*                 get_music_data() = 0;

    //! might return NULL if no RDC1 data were captured
    virtual RadarDataCube1*            get_rdc1() = 0;

    //! might return NULL if no ADC data were captured
    virtual ADCCaptureData*            get_adc() = 0;

    //! might return NULL if no thresholds were captured for this scan
    virtual const RDC_ThresholdControl* get_scan_thresholds() const = 0;

    //! returns the host local time of the scan completion. If you need the scan
    //! start time or middle, you must subtract all or half of the scan dwell
    //! time which is float scan_time in UhdpScanInformation, which is in units
    //! of seconds.
    virtual const timeval&             get_scan_end_local_time() const = 0;

    //! returns the version of the radar software which generated this scan
    virtual const char*                get_radar_software_version() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the module (board) name, eg: "MOD-222-Judo"
    virtual const char*                get_module_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the module type, eg: "Judo", "Wingchun", "Kenpo",
    //! or "Kung-fu"
    virtual const char*                get_module_type_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the motherboard type, eg: "PIB2", "PIB48", or
    //! "Laufenburg"
    virtual const char*                get_motherboard_type_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the antenna daughterboard type, eg: "ADB2",
    //! "Laufenburg", or "Dhaka"
    virtual const char*                get_antennaboard_type_name() const = 0;

    //! If the radar software supports this feature, the function will return a
    //! string describing the antenna module type, eg: "ADB2",
    //! "Rhine", "Turag", or "Blanco"
    virtual const char*                get_antenna_module_type_name() const = 0;

    //! might return NULL if no custom data was emitted for this data (message) type
    virtual const char*                get_custom_data(uint32_t msg_type, uint32_t& length) = 0;

    //! returns pointer to num_range_bins const values, might return NULL.
    //! BEWARE: Order is in hardware range bins, not distance!
    virtual const UhdpRangeBinInfo*    get_range_bins() const = 0;

    //! returns pointer to output of requested dprobes, might return NULL.
    virtual const RDC_DProbe_Output*   get_dprobe_outputs() const = 0;

    //! returns pointer to DC measurements made during this scan, might return NULL.
    virtual const RDC_adi_measured_dc* get_measured_dc() const = 0;

    virtual INT                        get_num_completed_aprobes() const = 0;

    //! Returns the measured value for one of the completed aprobes. If request
    //! is provided, it is filled with the A-Probe request info (bus, channel, probe, etc).
    //! Returns false if index is out of range
    virtual bool                       get_completed_aprobe(INT index, RDC_AprobeRequest& request) const = 0;

    //! returns the magnitude of the noise floor, in decibels, for this scan, at
    //! the specified range in meters (noise floor is range variant)
    virtual float                      get_noise_floor_dB(float range) const = 0;

    //! might return NULL if not captured, requires DL_HISTOGRAM
    virtual const RDC_Histogram*       get_histogram(uint32_t range_bin) const = 0;

    //! returns pointer to num_beamforming_angles const values, might return NULL
    virtual const uint16_t*            get_zero_doppler_bins() const = 0;

    //! might return NULL if sparse RDC3 was not captured for this scan
    virtual const char*                get_sparse_rdc3_blob(size_t& length) = 0;

    //! returns the azimuth angle in radians of an azimuth bin
    virtual float                      get_azimuth_rad(uint32_t az_bin) const = 0;

    //! returns the elevation angle in radians of an elevation bin
    virtual float                      get_elevation_rad(uint32_t el_bin) const = 0;

    //! returns both the azimuth and elevation, in radians, of a beamforming angle bin (0..127)
    virtual void                       get_angle(uint32_t rough_angle, float& azrad, float& elrad) const = 0;

    virtual uint8_t                    get_vrx_mapping(uint32_t vrxid) const = 0;

    //! retrieves the position of a physical transmitter (meters from origin).
    //! The total number of physical transmitter positions will be get_scan_info().num_tx_prn
    virtual const vec3f_t&             get_transmitter_position(uint32_t txid) const = 0;

    //! retrieves the position of a physical receiver (meters from origin)
    //! The total number of physical receiver positions will be
    //! get_scan_info().total_vrx / get_scan_info().num_tx_prn
    virtual const vec3f_t&             get_receiver_position(uint32_t rxid) const = 0;

    //! get the mounting geometry distances of this radar (as configured in its modulecfg file)
    virtual void                       get_mounting_distances(float& rear_axle_dist, float& centerline_dist, float& height) const = 0;

    //! get the mounting geometry angles of this radar (as configured in its modulecfg file)
    //! Positive mount angle implies the radar is pointing to the right of the vehicle boresight,
    //! i.e.: mounted on the right front corner of the vehicle).
    virtual void                       get_mounting_angles(float& azimuth_deg, float& elevation_deg) const = 0;

    virtual void                       serialize(ScanSerializer& s) const = 0;

    virtual void                       serialize_scan_info_json(ScanSerializer& s) const = 0;

    //! save all non-released scan data objects to files into the specified
    //! session folder, using the scan sequence number specified by the radar
    virtual void                       save_to_session(const char* session_path) const = 0;

    //! releases this scan object and all of its components (clutter image,
    //! etc). The user must drop all references to this scan and its data.
    virtual void                       release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        SCAN_NO_ERROR,
        SCAN_DATA_NOT_CAPTURED,
        SCAN_INVALID_INPUT,
        SCAN_ALREADY_RELEASED,
    };

    //! returns the last error encountered by this instance
    virtual Err                        get_last_error() const = 0;

    static ScanObject* deserialize(ScanSerializer& s, uint32_t scan_sequence_number);

    //! Factory method which returns a ScanObject instance which contains all of
    //! the data correlating to the specified scan sequence number within the
    //! specified session folder. May return NULL if the load or parse fails,
    //! and some data types (StaticSlice, ClutterImage, etc) might not be
    //! present.  Note that no Connection object is required to load scans
    //! from a session folder.
    static ScanObject* load_from_session(const char* session_path, uint32_t scan_sequence_number);
};
