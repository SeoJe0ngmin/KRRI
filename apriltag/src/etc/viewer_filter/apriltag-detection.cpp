// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

// NOTE: the viewer's CMakeLists.txt hard-requires pkg-config module 'apriltag', so this file is
// always compiled -- a missing AprilTag library is a configure-time FATAL_ERROR, not a silent skip.

#include "post-processing-filters-list.h"
#include "post-processing-worker-filter.h"

#include <rs-config.h>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>


namespace {


// Tag edge length in meters, black border included. Read from the viewer configuration file so it
// can be changed without a rebuild; the default is our docking target.
char const * const TAG_SIZE_KEY = "apriltag.tag_size";
double const DEFAULT_TAG_SIZE = 0.20;

double const RAD2DEG = 57.295779513082320876798154814105;


// libapriltag exports the detector and pose API but not matd_destroy(). A matd_t is a single
// allocation with a flexible data[] member, so free() is exactly what matd_destroy() does.
void matd_free( matd_t * m )
{
    free( m );
}


// apriltag_detections_destroy() releases both the detections and the array holding them
struct detections_holder
{
    zarray_t * z = nullptr;
    ~detections_holder() { if( z ) apriltag_detections_destroy( z ); }
};


struct pose_holder
{
    apriltag_pose_t p { nullptr, nullptr };
    ~pose_holder() { matd_free( p.R ); matd_free( p.t ); }
};


// What the docking controller wants: where the vehicle stands relative to the tag.
// Ported from src/models/tag_pose.py -- docking_state() and tag_tilt_deg().
struct docking_numbers
{
    double lateral;         // [m] sideways offset off the tag's front axis
    double vertical;        // [m] up/down offset
    double forward;         // [m] perpendicular range; positive means in front of the tag
    double distance;        // [m] straight line
    double approach_deg;    // [deg] direction off the front axis
    double heading_deg;     // [deg] 0 when driving parallel to the front axis
    double tilt_deg;        // [deg] 0 when the tag plane squarely faces the camera
    bool reliable_angle;    // below ~10 degrees the perspective is sub-pixel and angles cannot be trusted
};


// estimate_tag_pose() already returns T_camera_tag in the same tag frame that src/models/tag_pose.py works
// in, so the rotation is copied out as-is. Do NOT "correct" it by flipping the y and z columns:
// src/models/tag_pose.py gets its pose from detector.detection_pose() (libapriltag's pose_from_homography),
// and that frame and estimate_tag_pose()'s frame agree. Flipping inverts `forward` and `vertical`
// and puts `heading` ~180 degrees out -- checked on synthetic tag36h11 renderings at yaw, pitch
// and roll, where the flipped variant disagreed with src/models/tag_pose.py on 23 of 42 numbers and this
// one on none.
void copy_rotation( matd_t const * R_apriltag, double R[9] )
{
    for( int r = 0; r < 3; ++r )
        for( int c = 0; c < 3; ++c )
            R[r * 3 + c] = MATD_EL( R_apriltag, r, c );
}


// 카메라 기준 태그의 roll/pitch/yaw [deg].
// src/utils/util.py 의 r2rpy() 와 **같은 분해 순서**를 쓴다. 오일러 각은 순서를 바꾸면 값이 달라져서,
// 여기서 다른 순서를 쓰면 tools/run.py 와 숫자가 어긋난다.
//     roll  = atan2(R21, R22)
//     pitch = atan2(-R20, hypot(R21, R22))
//     yaw   = atan2(R10, R00)
// 주의: 이 셋은 표시용이다. 도킹 제어는 heading_deg 를 쓴다 -- yaw 는 광축 둘레 회전이라
// "지게차가 태그 축에서 몇 도 틀어졌나" 와 다른 값이다.
void rpy_deg( double const R[9], double rpy[3] )
{
    double const r21 = R[7], r22 = R[8], r20 = R[6], r10 = R[3], r00 = R[0];
    rpy[0] = std::atan2( r21, r22 ) * 180. / M_PI;
    rpy[1] = std::atan2( -r20, std::sqrt( r21 * r21 + r22 * r22 ) ) * 180. / M_PI;
    rpy[2] = std::atan2( r10, r00 ) * 180. / M_PI;
}


// R and t are T_camera_tag in the tag frame, where +z points behind the tag
docking_numbers compute_docking( double const R[9], double const t[3] )
{
    // T_tag_cam = inverse( T_camera_tag ):  rotation R^T,  position p = -R^T * t
    double p[3];
    for( int i = 0; i < 3; ++i )
        p[i] = -( R[0 * 3 + i] * t[0] + R[1 * 3 + i] * t[1] + R[2 * 3 + i] * t[2] );

    docking_numbers d;
    d.lateral = p[0];
    d.vertical = p[1];
    d.forward = -p[2];  // flipped so that positive reads as "in front of the tag"
    d.distance = std::sqrt( p[0] * p[0] + p[1] * p[1] + p[2] * p[2] );
    d.approach_deg = RAD2DEG * std::atan2( std::fabs( d.lateral ), std::fabs( d.forward ) );

    // Camera optical axis (+z) expressed in the tag frame: R^T * (0,0,1), i.e. the third row of R
    d.heading_deg = RAD2DEG * std::atan2( R[2 * 3 + 0], R[2 * 3 + 2] );

    // Tag normal in the camera frame: R * (0,0,1); its z component says how square the tag is on
    d.tilt_deg = RAD2DEG * std::acos( std::min( 1., std::fabs( R[2 * 3 + 2] ) ) );

    d.reliable_angle = ( d.approach_deg >= 10. );
    return d;
}


// AprilTag works on 8-bit grayscale. The post-processing filter sees the raw sensor format -- the
// conversion to RGB8 only happens later, at texture-upload time -- so we take the luma out of
// whatever the user selected in the stream's Format dropdown.
bool convert_to_gray( rs2::video_frame const & cf, std::vector< uint8_t > & gray )
{
    int const w = cf.get_width();
    int const h = cf.get_height();
    int const stride = cf.get_stride_in_bytes();
    uint8_t const * const src = static_cast< uint8_t const * >( cf.get_data() );
    if( ! src || w <= 0 || h <= 0 )
        return false;

    rs2_format const format = cf.get_profile().format();
    gray.resize( size_t( w ) * size_t( h ) );
    uint8_t * const dst = gray.data();

    switch( format )
    {
    case RS2_FORMAT_Y8:
        for( int y = 0; y < h; ++y )
            memcpy( dst + size_t( y ) * w, src + size_t( y ) * stride, w );
        break;

    case RS2_FORMAT_Y16:
        for( int y = 0; y < h; ++y )
        {
            uint16_t const * const s = reinterpret_cast< uint16_t const * >( src + size_t( y ) * stride );
            uint8_t * const d = dst + size_t( y ) * w;
            for( int x = 0; x < w; ++x )
                d[x] = uint8_t( s[x] >> 8 );
        }
        break;

    case RS2_FORMAT_YUYV:  // Y0 U Y1 V -- luma is every even byte
    case RS2_FORMAT_UYVY:  // U Y0 V Y1 -- luma is every odd byte
        {
            int const luma = ( format == RS2_FORMAT_YUYV ) ? 0 : 1;
            for( int y = 0; y < h; ++y )
            {
                uint8_t const * const s = src + size_t( y ) * stride;
                uint8_t * const d = dst + size_t( y ) * w;
                for( int x = 0; x < w; ++x )
                    d[x] = s[2 * x + luma];
            }
        }
        break;

    case RS2_FORMAT_RGB8:
    case RS2_FORMAT_RGBA8:
    case RS2_FORMAT_BGR8:
    case RS2_FORMAT_BGRA8:
        {
            int const bpp = ( format == RS2_FORMAT_RGB8 || format == RS2_FORMAT_BGR8 ) ? 3 : 4;
            int const ri = ( format == RS2_FORMAT_RGB8 || format == RS2_FORMAT_RGBA8 ) ? 0 : 2;
            int const bi = 2 - ri;
#pragma omp parallel for schedule(static)
            for( int y = 0; y < h; ++y )
            {
                uint8_t const * const s = src + size_t( y ) * stride;
                uint8_t * const d = dst + size_t( y ) * w;
                for( int x = 0; x < w; ++x )
                {
                    uint8_t const * const px = s + x * bpp;
                    // CCIR 601 -- see https://en.wikipedia.org/wiki/Luma_(video)
                    d[x] = uint8_t( 0.2989f * px[ri] + 0.5870f * px[1] + 0.1140f * px[bi] );
                }
            }
        }
        break;

    default:
        return false;
    }
    return true;
}


// Project a point given in the tag frame through the tag pose (R,t) and the pinhole intrinsics.
// R,t are T_camera_tag -- the very same pair compute_docking() consumes -- so the drawn axes and
// the displayed numbers can never disagree. Returns false when the point lands on or behind the
// image plane (z <= 0), where the perspective divide is meaningless; the caller must then skip
// that axis rather than draw garbage.
bool project_tag_point( double const R[9], double const t[3], rs2_intrinsics const & intrin,
                        double X, double Y, double Z, float & u, float & v )
{
    double const xc = R[0] * X + R[1] * Y + R[2] * Z + t[0];
    double const yc = R[3] * X + R[4] * Y + R[5] * Z + t[1];
    double const zc = R[6] * X + R[7] * Y + R[8] * Z + t[2];
    if( ! ( zc > 1e-6 ) )
        return false;
    // Plain pinhole, no distortion -- the same model estimate_tag_pose() was handed above
    u = float( intrin.fx * ( xc / zc ) + intrin.ppx );
    v = float( intrin.fy * ( yc / zc ) + intrin.ppy );
    return true;
}


}  // namespace


/* Detect tag36h11 AprilTags and report the docking pose of each one.

   Besides the bounding box and text label that every overlay object carries, each detection also
   fills in object_in_frame::geometry: the tag's rotated quad (the detector's four corners) and the
   tag's 3D pose axes projected back into the image, so the viewer draws the same rotated outline
   and RGB axis cross the Python tool draws. The bounding box is still filled in -- the label is
   placed against it, and it is the fallback for any consumer that ignores the geometry.
*/
class apriltag_docking_pose : public post_processing_worker_filter
{
    apriltag_family_t * _family = nullptr;
    apriltag_detector_t * _detector = nullptr;
    double _tag_size = DEFAULT_TAG_SIZE;
    std::vector< uint8_t > _gray;
    rs2_format _reported_format = RS2_FORMAT_ANY;

    std::shared_ptr< atomic_objects_in_frame > _objects;

public:
    explicit apriltag_docking_pose( std::string const & name )
        : post_processing_worker_filter( name )
    {
    }

    ~apriltag_docking_pose()
    {
        // Complete background worker to ensure it releases the instance's resources in controlled manner
        release_background_worker();
    }

public:
    void start( rs2::subdevice_model & model ) override
    {
        // Grab the overlay channel BEFORE the base class spawns the worker thread: the worker
        // reads _objects on its very first frame, and assigning it afterwards is a data race.
        _objects = model.detected_objects;
        post_processing_worker_filter::start( model );
    }

private:
    void worker_start() override
    {
        _tag_size = rs2::config_file::instance().get_or_default( TAG_SIZE_KEY, DEFAULT_TAG_SIZE );
        if( ! ( _tag_size > 0. ) )
            _tag_size = DEFAULT_TAG_SIZE;

        _family = tag36h11_create();
        if( ! _family )
            throw std::runtime_error( "tag36h11_create() failed" );
        _detector = apriltag_detector_create();
        if( ! _detector )
        {
            tag36h11_destroy( _family );
            _family = nullptr;
            throw std::runtime_error( "apriltag_detector_create() failed" );
        }
        apriltag_detector_add_family( _detector, _family );
        _detector->quad_decimate = 1.f;  // docking needs the corners at full resolution
        _detector->quad_sigma = 0.f;
        _detector->refine_edges = 1;
        _detector->nthreads = 2;

        LOG(INFO) << get_name() << ": tag36h11, tag size " << _tag_size << " m (config key '" << TAG_SIZE_KEY << "')";
    }

    void worker_end() override
    {
        // The detector does not own the family, so it goes first and the family after it
        if( _detector )
        {
            apriltag_detector_destroy( _detector );
            _detector = nullptr;
        }
        if( _family )
        {
            tag36h11_destroy( _family );
            _family = nullptr;
        }
    }

    // Hand the results over to the render thread. Publishing an empty vector is how the overlay is
    // made to disappear, so this is called on every frame, detections or not.
    void publish( objects_in_frame & objects )
    {
        if( ! _objects )
            return;
        std::lock_guard< std::mutex > lock( _objects->mutex );
        if( is_pb_enabled() )
        {
            if( _objects->sensor_is_on )
                _objects->swap( objects );
        }
        else
        {
            _objects->clear();
        }
    }

    void worker_body( rs2::frame f ) override
    {
        objects_in_frame objects;

        rs2::frameset fs = f.as< rs2::frameset >();
        rs2::frame cf = f;
        if( fs )
            cf = fs.get_color_frame();

        if( ( ! fs && f.get_profile().stream_name() != "Color" ) || ( fs && ! cf ) )
        {
            publish( objects );
            return;
        }

        try
        {
            rs2::video_frame vf = cf.as< rs2::video_frame >();
            if( ! vf )
            {
                publish( objects );
                return;
            }

            rs2_format const format = vf.get_profile().format();
            if( ! convert_to_gray( vf, _gray ) )
            {
                if( format != _reported_format )
                {
                    _reported_format = format;
                    LOG(ERROR) << get_context( f ) << "unsupported color format: " << format;
                }
                publish( objects );
                return;
            }
            _reported_format = RS2_FORMAT_ANY;

            // The pinhole parameters come from the stream itself; distortion is ignored, exactly as
            // the tag-pose estimator (and our Python detection_pose path) does.
            rs2_intrinsics const intrin = vf.get_profile().as< rs2::video_stream_profile >().get_intrinsics();

            image_u8_t image = { vf.get_width(), vf.get_height(), vf.get_width(), _gray.data() };
            detections_holder detections;
            detections.z = apriltag_detector_detect( _detector, &image );
            if( ! detections.z )
                throw std::runtime_error( "apriltag_detector_detect() returned nothing" );

            rs2::rect const image_rect { 0, 0, float( image.width ), float( image.height ) };
            for( int i = 0; i < zarray_size( detections.z ); ++i )
            {
                apriltag_detection_t * det = nullptr;
                zarray_get( detections.z, i, &det );
                if( det->hamming )  // corrected bits mean a suspect decode -- our Python path drops these too
                    continue;

                apriltag_detection_info_t info { det, _tag_size, intrin.fx, intrin.fy, intrin.ppx, intrin.ppy };
                pose_holder pose;
                estimate_tag_pose( &info, &pose.p );
                if( ! pose.p.R || ! pose.p.t )
                    continue;

                double R[9], t[3];
                copy_rotation( pose.p.R, R );
                for( int r = 0; r < 3; ++r )
                    t[r] = MATD_EL( pose.p.t, r, 0 );
                docking_numbers const d = compute_docking( R, t );

                // The bounding box still gets computed: the text label is laid out against it, and
                // it is what any consumer that ignores object_geometry falls back to.
                float min_x = float( det->p[0][0] ), max_x = min_x;
                float min_y = float( det->p[0][1] ), max_y = min_y;
                for( int c = 1; c < 4; ++c )
                {
                    min_x = std::min( min_x, float( det->p[c][0] ) );
                    max_x = std::max( max_x, float( det->p[c][0] ) );
                    min_y = std::min( min_y, float( det->p[c][1] ) );
                    max_y = std::max( max_y, float( det->p[c][1] ) );
                }
                rs2::rect const quad { min_x, min_y, max_x - min_x, max_y - min_y };
                rs2::rect const normalized_bbox = quad.normalize( image_rect );

                std::ostringstream text;
                text << "#" << det->id << std::fixed
                     << std::setprecision( 2 )
                     << "\nxyz " << t[0] << " " << t[1] << " " << t[2]
                     << "\nlat " << d.lateral << " fwd " << d.forward
                     << std::setprecision( 1 )
                     // '?' marks a near-frontal view, where the perspective is sub-pixel and the
                     // angles cannot be trusted -- steer by 'lat' instead
                     << "\nhdg " << d.heading_deg << ( d.reliable_angle ? " " : "? " )
                     << "tilt " << d.tilt_deg;

                objects.emplace_back(
                    size_t( det->id ),
                    text.str(),
                    normalized_bbox,
                    normalized_bbox,  // no depth frame is used; keep the color box
                    float( d.forward ),
                    0.f,           // metadata_depth -- not provided
                    0.5f, 0.5f,    // com_rel_u/v -- not computed
                    0,             // score -- not provided
                    object_type::other
                );

                object_geometry & geom = objects.back().geometry;

                // The rotated quad, straight from the detector's corners, normalized the same way
                // the bounding box above is
                for( int c = 0; c < 4; ++c )
                {
                    geom.quad[c].x = rs2::normalizeT( float( det->p[c][0] ), image_rect.x, image_rect.x + image_rect.w );
                    geom.quad[c].y = rs2::normalizeT( float( det->p[c][1] ), image_rect.y, image_rect.y + image_rect.h );
                }
                geom.has_quad = true;

                // Pose axes: the tag-frame points (0,0,0) and (L,0,0)/(0,L,0)/(0,0,L) pushed through
                // the same (R,t) the docking numbers come from. Half the tag edge keeps the cross
                // inside the tag's own footprint, which is how the Python tool draws it.
                double const L = _tag_size / 2.;
                double const axis_pt[3][3] = { { L, 0., 0. }, { 0., L, 0. }, { 0., 0., L } };
                float ox = 0.f, oy = 0.f;
                if( project_tag_point( R, t, intrin, 0., 0., 0., ox, oy ) )
                {
                    geom.axis_origin.x = rs2::normalizeT( ox, image_rect.x, image_rect.x + image_rect.w );
                    geom.axis_origin.y = rs2::normalizeT( oy, image_rect.y, image_rect.y + image_rect.h );
                    for( int a = 0; a < 3; ++a )
                    {
                        float ax = 0.f, ay = 0.f;
                        // A point behind the camera projects to nonsense -- drop just that axis
                        if( ! project_tag_point( R, t, intrin, axis_pt[a][0], axis_pt[a][1], axis_pt[a][2], ax, ay ) )
                            continue;
                        geom.axis_end[a].x = rs2::normalizeT( ax, image_rect.x, image_rect.x + image_rect.w );
                        geom.axis_end[a].y = rs2::normalizeT( ay, image_rect.y, image_rect.y + image_rect.h );
                        geom.axis_valid[a] = true;
                        geom.has_axes = true;
                    }
                }

                double rpy[3];
                rpy_deg( R, rpy );
                LOG(DEBUG) << get_context( f ) << "tag36h11 #" << det->id
                           << " | x " << t[0] << " y " << t[1] << " z " << t[2]
                           << " roll " << rpy[0] << " pitch " << rpy[1] << " yaw " << rpy[2]
                           << " | lateral " << d.lateral << " vertical " << d.vertical
                           << " forward " << d.forward << " distance " << d.distance
                           << " approach " << d.approach_deg << " heading " << d.heading_deg
                           << " tilt " << d.tilt_deg
                           << " reliable " << ( d.reliable_angle ? 1 : 0 );
            }

            publish( objects );
        }
        catch( const std::exception & e )
        {
            LOG(ERROR) << get_context( f ) << e.what();
        }
        catch( ... )
        {
            LOG(ERROR) << get_context( f ) << "Unhandled exception caught in apriltag_docking_pose";
        }
    }

    void on_processing_block_enable( bool e ) override
    {
        post_processing_worker_filter::on_processing_block_enable( e );
        if( ! e  &&  _objects )
        {
            // Make sure all the objects go away!
            std::lock_guard< std::mutex > lock( _objects->mutex );
            _objects->clear();
        }
    }
};


static auto it_apriltag = post_processing_filters_list::register_filter< apriltag_docking_pose >( "AprilTag : Docking Pose" );
