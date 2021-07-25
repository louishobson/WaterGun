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
#include <mutex>
#include <NiTE2/NiTE.h>
#include <string>
#include <thread>
#include <vector>
#include <watergun/watergun_exception.h>



/* DECLARATIONS */

namespace watergun
{
    /** struct vector3d : nite::Point3f
     * 
     * Wrapper for nite::Point3f, but with overloaded operations.
     */
    struct vector3d;

    /** class tracker : nite::UserTracker::NewFrameListener
     * 
     * Creates a OpenNI/NITE context, and exposes human-tracking capabilities.
     */
    class tracker;
}



/* VECTOR3D DEFINITION */



/** struct vector3d : nite::Point3f
 * 
 * Wrapper for nite::Point3f, but with overloaded operations.
 */
struct watergun::vector3d : public nite::Point3f
{
    /** @name default construction
     * 
     * @brief Initialize all components to 0.
     */
    vector3d () noexcept : Point3f { 0., 0., 0. } {}

    /** @name single components constructor
     * 
     * @brief Initialize all components to the same value.
     * @param v: The value to initialize the components to.
     */
    explicit vector3d ( float v ) noexcept : Point3f { v, v, v } {}

    /** @name three component constructor
     * 
     * @brief Initialize all components to separate values.
     * @param x: X value.
     * @param y: Y value.
     * @param z: Z value.
     */
    vector3d ( float x, float y, float z ) noexcept : Point3f { x, y, z } {}

    /** @name Point3f constructor
     * 
     * @brief Construct from nite::Point3f object.
     * @param v: The Point3f object.
     */
    vector3d ( const Point3f& v ) noexcept : Point3f { v } {}



    /* Simple arithmetic operations */
    vector3d operator+ ( const vector3d& other ) const noexcept { return vector3d { x + other.x, y + other.y, z + other.z }; }
    vector3d operator- ( const vector3d& other ) const noexcept { return vector3d { x - other.x, y - other.y, z - other.z }; }
    vector3d operator* ( const vector3d& other ) const noexcept { return vector3d { x * other.x, y * other.y, z * other.z }; }
    vector3d operator/ ( const vector3d& other ) const noexcept { return vector3d { x / other.x, y / other.y, z / other.z }; }
    vector3d& operator+= ( const vector3d& other ) noexcept { return * this = * this + other; }
    vector3d& operator-= ( const vector3d& other ) noexcept { return * this = * this - other; }
    vector3d& operator*= ( const vector3d& other ) noexcept { return * this = * this * other; }
    vector3d& operator/= ( const vector3d& other ) noexcept { return * this = * this / other; }
    vector3d operator* ( float scalar ) const noexcept { return * this * vector3d { scalar }; }
    vector3d operator/ ( float scalar ) const noexcept { return * this / vector3d { scalar }; }
    vector3d& operator*= ( float scalar ) noexcept { return * this = * this * scalar; }
    vector3d& operator/= ( float scalar ) noexcept { return * this = * this / scalar; }
};



/* TRACKER DEFINITION */



/** class tracker
 * 
 * Creates a OpenNI/NITE context, and exposes human-tracking capabilities.
 */
class watergun::tracker : private nite::UserTracker::NewFrameListener
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
        nite::UserId id;

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
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    explicit tracker ( vector3d _camera_offset = vector3d {} );

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
    float camera_h_fov, camera_v_fov;
    float camera_depth;

    /* The output mode of the camera */
    openni::VideoMode camera_output_mode;

    /* The offset of the camera from the origin */
    vector3d camera_offset;



    /* An arbitrarily large duration and duration */
    static const clock::duration   large_duration;
    static const clock::time_point large_time_point;

    /* Zero duration and time point */
    static const clock::duration   zero_duration;
    static const clock::time_point zero_time_point;



    /** @name  duration_to_seconds
     * 
     * @brief  Get a duration in seconds as a double.
     * @param  dur: The duration to cast.
     * @return The duration in seconds.
     */
    template<class Rep, class Ratio>
    static constexpr std::chrono::duration<double> duration_to_seconds ( std::chrono::duration<Rep, Ratio> dur ) noexcept
        { return std::chrono::duration_cast<std::chrono::duration<double>> ( dur ); }

    /** @name  rate_of_change
     * 
     * @brief  Calculate the rate of change, given value and time deltas.
     * @param  delta_v: The change in value.
     * @param  delta_t: The change in time.
     * @return Rate of change as a double.
     */
    template<class T, class Rep, class Ratio>
    static constexpr T rate_of_change ( T delta_v, std::chrono::duration<Rep, Ratio> delta_t ) noexcept
        { return delta_v / duration_to_seconds ( delta_t ).count (); }



private:

    /* The OpenNI device handle */
    openni::Device device;

    /* OpenNI video stream object */
    openni::VideoStream depth_stream;

    /* NiTE user tracker */
    nite::UserTracker user_tracker;

    /* System and OpenNI timestamps at the start of the program */
    clock::time_point system_timestamp;
    std::uint64_t openni_timestamp;



    /* The minimum rate of change of COM for it to not be considered 0 */
    const vector3d min_com_rate { M_PI / 240. /* 0.75 degrees */, 0.100 /* 10 cm */, 0.050 /* 5 cm */ };

    /* The clock sync period in frames */
    const int clock_sync_period { 30 * 30 };



    /* An array of the current tracked users */
    std::vector<tracked_user> tracked_users;

    /* The average computation time for the user generator */
    clock::duration average_generation_time { 0 };

    /* A counter for re-syncing the clock in frames */
    int clock_sync_counter { clock_sync_period };

    /* A mutex and condition variable to protect tracked_users */
    mutable std::mutex tracked_users_mx;
    mutable std::condition_variable tracked_users_cv;



    /** @name  onNewFrame
     * 
     * @brief  Overload of pure virtual method, which will be called when new frame data is availible.
     * @param  [unnamed]: The user tracker for which new data is availible.
     * @return Nothing.
     */
    void onNewFrame ( nite::UserTracker& ) override final;



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
    clock::time_point openni_to_system_timestamp ( std::uint64_t timestamp ) const noexcept { return system_timestamp + std::chrono::microseconds { timestamp - openni_timestamp }; }



    /** @name  check_status
     * 
     * @brief  Takes a OpenNI or NiTE status, and checks that is is okay. If not, throws with the supplied reason and status description after a colon.
     * @param  status: The status returned from an OpenNI or NiTE call.
     * @param  error_msg: The error message to set the exception to contain.
     */
    static void check_status ( openni::Status status, const std::string& error_msg );
    static void check_status ( nite::Status status, const std::string& error_msg );

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_TRACKER_H_INCLUDED */

