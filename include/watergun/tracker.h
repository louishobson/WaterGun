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
#include <XnPropNames.h>



/* MACRO DEFINITIONS */

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
    constexpr vector3d () : XnVector3D { 0., 0., 0. } {}

    /** @name single components constructor
     * 
     * @brief Initialize all components to the same value.
     * @param v: The value to initialize the components to.
     */
    explicit constexpr vector3d ( XnFloat v ) : XnVector3D { v, v, v } {}

    /** @name three component constructor
     * 
     * @brief Initialize all components to separate values.
     * @param x: X value.
     * @param y: Y value.
     * @param z: Z value.
     */
    constexpr vector3d ( XnFloat x, XnFloat y, XnFloat z ) : XnVector3D { x, y, z } {}


    
    /* Default comparison operators */
    constexpr bool operator== ( const vector3d& other ) const { return X == other.X && Y == other.Y && Z == other.Z; }
    constexpr bool operator!= ( const vector3d& other ) const { return X != other.X || Y != other.Y || Z != other.Z; }

    /* Simple arithmetic operations */
    constexpr vector3d operator+ ( const vector3d& other ) const { return vector3d { X + other.X, Y + other.Y, Z + other.Z }; }
    constexpr vector3d operator- ( const vector3d& other ) const { return vector3d { X - other.X, Y - other.Y, Z - other.Z }; }
    constexpr vector3d operator* ( const vector3d& other ) const { return vector3d { X * other.X, Y * other.Y, Z * other.Z }; }
    constexpr vector3d operator/ ( const vector3d& other ) const { return vector3d { X / other.X, Y / other.Y, Z / other.Z }; }
    constexpr vector3d& operator+= ( const vector3d& other ) { return * this = * this + other; }
    constexpr vector3d& operator-= ( const vector3d& other ) { return * this = * this - other; }
    constexpr vector3d& operator*= ( const vector3d& other ) { return * this = * this * other; }
    constexpr vector3d& operator/= ( const vector3d& other ) { return * this = * this / other; }
    constexpr vector3d operator* ( XnFloat scalar ) const { return * this * vector3d { scalar }; }
    constexpr vector3d operator/ ( XnFloat scalar ) const { return * this / vector3d { scalar }; }
    constexpr vector3d& operator*= ( XnFloat scalar ) { return * this = * this * scalar; }
    constexpr vector3d& operator/= ( XnFloat scalar ) { return * this = * this / scalar; }
};



/* TRACKER DEFINITION */



/** class tracker
 * 
 * Creates a OpenNI/NITE context, and exposes human-tracking capabilities.
 */
class watergun::tracker
{
public:

    /* Clock typedefs */
    typedef std::chrono::system_clock clock;

    /** struct tracked_users
     * 
     * A structure to store information about a single user, including their ID and position in many formats.
     */
    struct tracked_user
    {
        /* The user ID */
        XnUserID id;

        /* The point in time that the position was taken */
        clock::time_point timestamp;

        /* The user's center of mass in mixed polar coordinates.
         * X is an angle from the center of the camera in radians, Y is the perpandicular height from the camera's center, Z is the distance from the camera. 
         */
        vector3d com;

        /* The rate of change of the COM */
        vector3d com_rate;
    };



    /** @name constructor
     * 
     * @brief Sets up the context and configures OpenNI/NITE for human recognition.
     * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
     * @param _num_trackable_users: The max number of trackable users.
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    explicit tracker ( vector3d _camera_offset = vector3d {}, XnUInt16 _num_trackable_users = WATERGUN_MAX_TRACKABLE_USERS );

    /** @name destructor
     * 
     * @brief Gracefully releases the OpenNI context and handles.
     */
    ~tracker ();



    /** @name  get_tracked_users
     * 
     * @brief  Immediately return an array of the currently tracked users. The timestamp and positions of the tracked users are projected to now.
     * @return Vector of users.
     */
    std::vector<tracked_user> get_tracked_users () const;

    /** @name  wait_get_tracked_users
     * 
     * @brief  Wait for data on tracked users to update, then return an array of them. The timestamp and positions of the tracked users are projected to now.
     * @return Vector of users.
     */
    std::vector<tracked_user> wait_get_tracked_users () const;

    /** @name  get_average_generation_time
     * 
     * @brief  Get the average time taken to generate depth data.
     * @return The average duration.
     */
    clock::duration get_average_generation_time () const;



    /** @name  project_tracked_user
     * 
     * @brief  Update a user's position to match a new timestamp, given that they follow the same velocity.
     * @param  user: The user to update.
     * @param  timestamp: The new timestamp that their position should match. Defaults to now.
     * @return The updated tracked user.
     */
    tracked_user project_tracked_user ( const tracked_user& user, clock::time_point timestamp = clock::now () ) const;

    /** @name  dynamic_project_tracked_user
     * 
     * @brief  Same as project_tracked_user, except will be overridden in derived classes to take the rotation of the camera into account.
     * @param  user: The user to update.
     * @param  timestamp: The new timestamp that their position should match. Defaults to now.
     * @return The updated tracked user.
     */
    virtual tracked_user dynamic_project_tracked_user ( const tracked_user& user, clock::time_point timestamp = clock::now () ) const 
        { return project_tracked_user ( user, timestamp ); }



protected:

    /* The FOV and maximum depth of the camera */
    XnFieldOfView camera_fov;
    XnFloat camera_depth;

    /* The output mode of the camera */
    XnMapOutputMode camera_output_mode;

    /* The offset of the camera from the origin */
    vector3d camera_offset;

    /* The max number of trackabke users */
    XnUInt16 num_trackable_users;



    /* An arbitrarily large duration and duration */
    static constexpr clock::duration   large_duration   = std::chrono::hours { 24 };
    const            clock::time_point large_time_point = clock::now () + large_duration; 

    /* Zero duration and time point */
    static constexpr clock::duration   zero_duration   = clock::duration::zero ();
    static constexpr clock::time_point zero_time_point = clock::time_point {};



    /** @name  duration_to_seconds
     * 
     * @brief  Get a duration in seconds as a double.
     * @param  dur: The duration to cast.
     * @return The duration in seconds.
     */
    template<class Rep, class Ratio>
    static constexpr std::chrono::duration<double> duration_to_seconds ( std::chrono::duration<Rep, Ratio> dur )
        { return std::chrono::duration_cast<std::chrono::duration<double>> ( dur ); }

    /** @name  rate_of_change
     * 
     * @brief  Calculate the rate of change, given value and time deltas.
     * @param  delta_v: The change in value.
     * @param  delta_t: The change in time.
     * @return Rate of change as a double.
     */
    template<class T, class Rep, class Ratio>
    static constexpr T rate_of_change ( T delta_v, std::chrono::duration<Rep, Ratio> delta_t ) 
        { return delta_v / duration_to_seconds ( delta_t ).count (); }



private:

    /* OpenNI context */
    xn::Context context;

    /* OpenNI script node */
    xn::ScriptNode script_node;
    
    /* OpenNI depth and user generators */
    xn::DepthGenerator depth_generator;
    xn::UserGenerator  user_generator;

    /* System and OpenNI timestamps at the start of the program */
    clock::time_point system_timestamp;
    XnUInt64 openni_timestamp;



    /* The minimum rate of change of COM for it to not be considered 0 */
    static constexpr vector3d min_com_rate { M_PI / 240. /* 0.75 degrees */, 0.100 /* 10 cm */, 0.050 /* 5 cm */ };

    /* The clock sync period in frames */
    static constexpr int clock_sync_period = 2 * 30;



    /* An array of the current tracked users */
    std::vector<tracked_user> tracked_users;

    /* The average computation time for the user generator */
    clock::duration average_generation_time { 0 };

    /* A mutex and condition variable to protect tracked_users */
    mutable std::mutex tracked_users_mx;
    mutable std::condition_variable tracked_users_cv;



    /* The thread which is handling updating tracked_users */
    std::thread tracker_thread;

    /* Atomic bool telling the tracker thread when to quit */
    std::atomic_bool end_threads = false;



    /** @name  tracker_thread_function
     * 
     * @brief  Function run by tracker_thread. Runs in a loop, updating tracked_users as new frames come in.
     * @return Nothing.
     */
    void tracker_thread_function ();



    /** @name  sync_clocks
     * 
     * @brief  Synchronize the OpenNI and system timestamps.
     * @return Nothing.
     */
    void sync_clocks ();

    /** @name  openni_to_system_timestamp
     * 
     * @brief  Change an OpenNI timestamp to a system timestamp.
     * @param  timestamp: The OpenNI timestamp.
     * @return A system timestamp.
     */
    clock::time_point openni_to_system_timestamp ( XnUInt64 timestamp ) const { return system_timestamp + std::chrono::microseconds { timestamp - openni_timestamp }; }



    /** @name  check_status
     * 
     * @brief  Takes a returned status, and checks that is is XN_STATUS_OK. If not, throws with the supplied reason and status description after a colon.
     * @param  status: The status returned from an OpenNI call.
     * @param  error_msg: The error message to set the exception to contain.
     */
    static void check_status ( XnStatus status, const std::string& error_msg );

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_TRACKER_H_INCLUDED */

