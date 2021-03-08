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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

class ClutterImage;

class Polar2Cartesian
{
public:

    /* NOTE: Cartesian space uses the SAE vehicle coordinate system specified in
     * SAE J670 - http://standards.sae.org/j670_200801/ */

    virtual ~Polar2Cartesian() {}

    //! descriptor used to specify the portion of the defined volume that the
    //! user wishes to be updated in a given process() call, and whether to flip
    //! the data across one or more of the axes.  This allows efficient
    //! integration into various visualizers
    struct SubSpace
    {
        uint32_t first_X;  //!< the first X bin to be updated (0..DIM_X)
        uint32_t center_Y; //!< the center Y bin to be updated (-DIM_Y/2..DIM_Y/2)
        uint32_t center_Z; //!< the center Z bin to be updated (-DIM_Z/2..DIM_Z/2)
        uint32_t extent_X; //!< the number of X bins to be updated
        uint32_t extent_Y; //!< the number of Y bins to be updated
        uint32_t extent_Z; //!< the number of Z bins to be updated
        bool     flip_X;   //!< flip the X dimension indices
        bool     flip_Y;   //!< flip the Y dimension indices
        bool     flip_Z;   //!< flip the Z dimension indices
        float max_range_distance; /*!< max range bins  to filter */
        float min_range_distance; /*!< min range bins to filter */
        float max_angle_distance; /*!< max angle bins  to filter */
        float min_angle_distance; /*!< min anglebins to filter */
        float range_bin_width; /*!< to calc the real distance of certain bins*/

        //! When processing a 3D elevation scan into a 2D mapped space, this mask
        //! determines which elevation bins are mapped into the 2D azimuth-only
        //! surface. If a single bit is set, that elevation bin is used
        //! directly. If multiple bits are set, a MAX operation is performed
        //! across each of those elevation bins, and the combined plane is used
        //! as the interpolation source.
        uint32_t elevation_mask;

        // Note, for front-view Polar2Cartesian process() calls, first_X
        // and extent_X are used to select the RANGE BINS which are combined
        // into the front-view image. flip_Y will flip the azimuth angles while
        // flip_X will flip the elevation angles.
    };

    /*! The returned buffer is either a 2D pixel space or a 3D voxel space.
     * Azimuth (Y) is always the major dimension and distance (X) is always the
     * minor dimension, ie: cart2d[X][Y] and cart3d[X][Z][Y].
     *
     * The output samples are SNR (linear) with 8 fractional bits (Q24.8).
     * Background areas (no radar signature) are 0, the noise floor will be
     * approximately 1.0 (in Q24.8, 256).
     *
     * If the ClutterImage does not have the same format and dimensions as the
     * original ClutterImage from which this Polar2Cartesian instance was
     * created, the function will return NULL and a subsequent call to
     * get_last_error() will return P2C_INVALID_CLUTTER_IMAGE.
     *
     * The optional SubSpace parameter may be used to specify a subset of the
     * predefined pixel/voxel space to process, in which case the output buffer
     * will have the SubSpace extent dimensions. If the SubSpace does not fit
     * within the pre-calculated pixel/voxel space the function will return NULL
     * and a subsequent call to get_last_error() will return
     * P2C_SUBSPACE_OUT_OF_RANGE
     *
     * alpha is the argument to a flicker filter, merging the previous output
     * contents with the clutter image currently being plotted. alpha=0.0f
     * gives full weight to the new image, alpha=1.0f gives full weight to the
     * previous image */
    virtual const uint32_t* process(const ClutterImage&, SubSpace*, float alpha=0.0f) = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        P2C_NO_ERROR,
        P2C_INVALID_CLUTTER_IMAGE,
        P2C_SUBSPACE_OUT_OF_RANGE,
    };

    //! returns the last error encountered by this instance
    virtual Err             get_last_error() const = 0;

    //! release all of the memory held by the static slice
    virtual void            release() = 0;

    //! output structure of estimate_bounds()
    struct BoundsInfo
    {
        float*   az_angles;            //!< the steering angles of each azimuth bin (possibly wrapped)
        float*   el_angles;            //!< the steering angles of each elevation bin (possibly wrapped)
        uint32_t num_azimuth_bins;     //!< number of azimuth steering angle bins (including wraps)
        uint32_t num_elevation_bins;   //!< number of elevation steering angle bins (including wraps)
        uint32_t num_valid_range_bins; //!< less than or equal to range bins in scan

        //! ratio of range dimension (in pixels or meters) to azimuth dimension
        float    range_to_azimuth_ratio;
        //! ratio of range dimension (in pixels or meters) to elevation dimension
        float    range_to_elevation_ratio;

        BoundsInfo() : az_angles(NULL), el_angles(NULL) {}
        ~BoundsInfo()
        {
            delete [] az_angles;
            delete [] el_angles;
        }
    };

    //! Estimate the dimensions required for a scan to fit entirely in an image
    //! buffer.  See ciplot.cpp for example usage.
    static void estimate_bounds(BoundsInfo&,
                                const ClutterImage&,
                                bool  wrap_azimuth_ambiguity,
                                bool  wrap_elevation_ambiguity);

    //! descriptor which defines the dimensions and locality of the 2D pixel
    //! image which will be created by create2DMapping()
    struct PixelSpace2D
    {
        /* Note that pixels are square (X and Y have the same scale) */

        float    pixels_per_range_bin; /*!< Scaling factor of pixel space */
        uint32_t dim_X;    /*!< number of pixels in backward-/forward+ dimension */
        uint32_t dim_Y;    /*!< number of pixels in left-/right+ dimension */
        uint32_t first_X;  /*!< the space covers pixels first_X through first_X + dim_X - 1 */
        uint32_t center_Y; /*!< the space covers pixels center_Y - dim_Y/2 to center_Y + dim_Y/2 */

        /*! if a scan's azimuth beamforming angles cover the complete span of
         * the antenna array's field of view, it is possible to wrap the angle
         * bins to the left and right to more clearly articulate the ambiguity */
        bool     wrap_ambiguity;
    };


    struct FrontViewDesc
    {
        //! determines size of output image, which will be dim_X * dim_Y where
        //! dim_X = pixels_per_angle_bin * num_azimuth_bins
        //! dim_Y = pixels_per_angle_bin * num_elevation_bins
        //! where the count of angle bins includes possible ambiguity wrapping
        uint32_t pixels_per_angle_bin;
        bool     wrap_azimuth_ambiguity;
        bool     wrap_elevation_ambiguity;
    };


    //! descriptor which defines the dimensions and locality of the 3D voxel
    //! volume which will be created by create3DMapping()
    struct VoxelSpace3D
    {
        float    x_voxels_per_range_bin; /*!< Scaling factor of voxel space for X */
        float    y_voxels_per_range_bin; /*!< Scaling factor of voxel space for Y */
        float    z_voxels_per_range_bin; /*!< Scaling factor of voxel space for Z */
        uint32_t dim_X;    /*!< number of voxels in backward-/forward+ dimension */
        uint32_t dim_Y;    /*!< number of voxels in left-/right+ dimension */
        uint32_t dim_Z;    /*!< number of voxels in up-/down+ dimension */
        uint32_t first_X;  /*!< the space covers voxels first_X through first_X + dim_X - 1 */
        uint32_t center_Y; /*!< the space covers voxels center_Y - dim_Y/2 to center_Y + dim_Y/2 */
        uint32_t center_Z; /*!< the space covers voxels center_Z - dim_Z/2 to center_Z + dim_Z/2 */

        /*! if a scan's beamforming angles cover the complete span of the
         * antenna array's field of view in azimuth or elevation, it is possible
         * to wrap the angle bins to the left and right to more clearly
         * articulate the ambiguity */
        bool     wrap_azimuth_ambiguity;
        bool     wrap_elevation_ambiguity;
    };

    /*! Factory method which allocates a Polar2Cartesian object which can map a
     * 2D or 3D polar ClutterImage to a 2D top-view cartesian pixel space,
     * pre-calculating the indices and weights for optimal performance. The
     * returned object will only be capable of processing ClutterImage objects
     * of the same format and dimensions. The X dimension is range, the Y
     * dimension is left/right */
    static Polar2Cartesian* create2DMapping(const ClutterImage&, const PixelSpace2D&);

    /*! Factory method which allocates a Polar2Cartesian object which can map a
     * 3D polar ClutterImage to 2D front-view POLAR interpolated pixel space,
     * pre-calculating the indices and weights for optimal performance. The
     * returned object will only be capable of processing ClutterImage objects
     * of the same format and dimensions.  The output X dimension is elevation,
     * and the Y dimension is azimuth (both are angles, not meters). If
     * pixels_per_angle_bin is 2, for instance, the output image will be 256x256 */
    static Polar2Cartesian* createFrontView(const ClutterImage&, const FrontViewDesc&);

    /*! Factory method which allocates a Polar2Cartesian object which can map a
     * 3D polar ClutterImage to a 3D cartesian voxel space, pre-calculating the
     * indices and weights for optimal performance. The returned object will
     * only be capable of processing ClutterImage objects of the same format and
     * dimensions */
    static Polar2Cartesian* create3DMapping(const ClutterImage&, const VoxelSpace3D&);
};
