/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/tracker.h
 * 
 * Header file for tracking people using the kinect sensor. Uses the OpenNI and NITE libraries.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_TRACKER_H_INCLUDED
#define WATERGUN_TRACKER_H_INCLUDED



/* INCLUDES */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <watergun/watergun_exception.h>
#include <XnCppWrapper.h>



/* MACRO DEFINITIONS */

/** XML local configuration file
 * 
 * The relative path to the local configuration file.
 * This is the first choice for configuration files.
 */
#ifndef WATERGUN_XML_LOCAL_CONFIG_PATH
    #define WATERGUN_XML_LOCAL_CONFIG_PATH "./config/config.xml"
#endif

/** XML global configuration file
 * 
 * The full path to the global configuration file.
 * This is second choice if the local file cannot be found.
 */
#ifndef WATERGUN_XML_GLOBAL_CONFIG_PATH
    #define WATERGUN_XML_GLOBAL_CONFIG_PATH "/etc/watergun/config.xml"
#endif

/** Maximum error message length
 * 
 * The maximum length of an error message from an EnumerationErrors class.
 */
#ifndef WATERGUN_MAX_ERROR_LENGTH
    #define WATERGUN_MAX_ERROR_LENGTH 1024
#endif

/** Maximum number of users
 * 
 * The maximum number of users the tracker can track.
 */
#ifndef WATERGUN_MAX_TRACKABLE_USERS
    #define WATERGUN_MAX_TRACKABLE_USERS 6
#endif



/* DECLARATIONS */

namespace watergun
{
    /** struct vector3d : XnVector3D
     * 
     * Wrapper for XnVector3D, but with arithmetic, component-wise operaions defined.
     */
    struct vector3d;

    /** class tracker
     * 
     * Creates a OpenNI/NITE context, and exposes human-tracking capabilities.
     */
    class tracker;
}



/* VECTOR3D DEFINITION */



/** struct vector3d : XnVector3D
 * 
 * Wrapper for XnVector3D, but with overloaded operations.
 */
struct watergun::vector3d : public XnVector3D
{
    /** @name default construction
     * 
     * @brief Initialize all components to 0.
     */
    vector3d () : XnVector3D { 0., 0., 0. } {}

    /** @name single components constructor
     * 
     * @brief Initialize all components to the same value.
     * @param v: The value to initialize the components to.
     */
    explicit vector3d ( XnFloat v ) : XnVector3D { v, v, v } {}

    /** @name three component constructor
     * 
     * @brief Initialize all components to separate values.
     * @param x: X value.
     * @param y: Y value.
     * @param z: Z value.
     */
    vector3d ( XnFloat x, XnFloat y, XnFloat z ) : XnVector3D { x, y, z } {}


    
    /* Default comparison operators */
    bool operator== ( const vector3d& other ) const { return X == other.X && Y == other.Y && Z == other.Z; }
    bool operator!= ( const vector3d& other ) const { return X != other.X || Y != other.Y || Z != other.Z; }

    /* Simple arithmetic operations */
    vector3d operator+ ( const vector3d& other ) const { return vector3d { X + other.X, Y + other.Y, Z + other.Z }; }
    vector3d operator- ( const vector3d& other ) const { return vector3d { X - other.X, Y - other.Y, Z - other.Z }; }
    vector3d operator* ( const vector3d& other ) const { return vector3d { X * other.X, Y * other.Y, Z * other.Z }; }
    vector3d operator/ ( const vector3d& other ) const { return vector3d { X / other.X, Y / other.Y, Z / other.Z }; }
    vector3d& operator+= ( const vector3d& other ) { return * this = * this + other; }
    vector3d& operator-= ( const vector3d& other ) { return * this = * this - other; }
    vector3d& operator*= ( const vector3d& other ) { return * this = * this * other; }
    vector3d& operator/= ( const vector3d& other ) { return * this = * this / other; }
    vector3d operator* ( XnFloat scalar ) const { return * this * vector3d { scalar }; }
    vector3d operator/ ( XnFloat scalar ) const { return * this / vector3d { scalar }; }
    vector3d& operator*= ( XnFloat scalar ) { return * this = * this * scalar; }
    vector3d& operator/= ( XnFloat scalar ) { return * this = * this / scalar; }
};



/* TRACKER DEFINITION */



/** class tracker
 * 
 * Creates a OpenNI/NITE context, and exposes human-tracking capabilities.
 */
class watergun::tracker
{
public:

    /** struct tracked_users
     * 
     * A structure to store information about a single user, including their ID and position in many formats.
     */
    struct tracked_user
    {
        /* The user ID */
        XnUserID id;

        /* The point in time that the position was taken */
        std::chrono::system_clock::time_point timestamp;

        /* The user's centre of mass in mixed polar coordinates.
         * X is an angle from the centre of the camera in radians, Y is the perpandicular height from the camera's center, Z is the distance from the camera. 
         */
        vector3d com;

        /* The rate of change of the COM */
        vector3d com_rate = { 0., 0., 0. };
    };



    /** @name constructor
     * 
     * @brief Sets up the context and configures OpenNI/NITE for human recognition.
     * @param camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
     * @param config: Path to a configuration file to use. If unspecified, the default local and global paths will be used.
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    explicit tracker ( vector3d camera_offset = vector3d {}, std::string config_path = "" );

    /** @name destructor
     * 
     * @brief Gracefully releases the OpenNI context and handles.
     */
    ~tracker ();



    /** @name  get_tracked_users
     * 
     * @brief  Immediately return an array of the currently tracked users.
     * @return Vector of users.
     */
    std::vector<tracked_user> get_tracked_users () const;

    /** @name  wait_tracked_users
     * 
     * @brief  Wait for data on tracked users to update, then return an array of them.
     * @return Vector of users.
     */
    std::vector<tracked_user> wait_tracked_users () const;



private:

    /* OpenNI context */
    xn::Context context;

    /* OpenNI script node */
    xn::ScriptNode script_node;
    
    /* OpenNI depth and user generators */
    xn::DepthGenerator depth_generator;
    xn::UserGenerator  user_generator;



    /* The offset of the camera from the origin */
    vector3d origin_offset;



    /* An array of the current tracked users */
    std::vector<tracked_user> tracked_users;

    /* A mutex and condition variable to protect tracked_users */
    mutable std::mutex tracked_users_mx;
    mutable std::condition_variable tracked_users_cv;

    /* The thread which is handling updating tracked_users */
    std::thread tracker_thread;

    /* Atomic bool telling the tracker thread when to quite */
    std::atomic_bool end_tracker_thread = false;



    /** @name  tracker_thread_function
     * 
     * @brief  Function run by tracker_thread. Runs in a loop, updating tracked_users as new frames come in.
     * @return Nothing.
     */
    void tracker_thread_function ();



    /** @name  check_status
     * 
     * @brief  Takes a returned status, and checks that is is XN_STATUS_OK. If not, throws with the supplied reason and status description after a colon.
     * @param  status: The status returned from an OpenNI call.
     * @param  error_msg: The error message to set the exception to contain.
     */
    static void check_status ( XnStatus status, const std::string& error_msg );

    /** @name  rate_of_change
     * 
     * @brief  Calculate the rate of change, given value and time deltas.
     * @param  delta_v: The change in value.
     * @param  delta_t: The change in time.
     * @return Rate of change as a double.
     */
    template<class T, class Rep, std::intmax_t Num, std::intmax_t Den>
    static auto rate_of_change ( T delta_v, std::chrono::duration<Rep, std::ratio<Num, Den>> delta_t ) 
        { return delta_v / ( static_cast<double> ( delta_t.count () ) * static_cast<double> ( Num ) / static_cast<double> ( Den ) ); }

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_TRACKER_H_INCLUDED */

