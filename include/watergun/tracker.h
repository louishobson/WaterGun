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
#include <watergun/utility.h>
#include <watergun/watergun_exception.h>



/* DECLARATIONS */

namespace watergun
{
    /** struct vector3d
     * 
     * 3D vector class.
     */
    struct vector3d;

    /** class tracker : nite::UserTracker::NewFrameListener
     * 
     * Creates a OpenNI/NITE context, and exposes human-tracking capabilities.
     */
    class tracker;
}



/* VECTOR3D DEFINITION */



/** struct vector3d
 * 
 * 3D vector class.
 */
struct watergun::vector3d
{
    /* X, Y and Z components */
    double x, y, z;



    /** @name default construction
     * 
     * @brief Initialize all components to 0.
     */
    constexpr vector3d () noexcept : x { 0. }, y { 0. }, z { 0. } {}

    /** @name single components constructor
     * 
     * @brief Initialize all components to the same value.
     * @param v: The value to initialize the components to.
     */
    explicit constexpr vector3d ( double v ) noexcept : x { v }, y { v }, z { v } {}

    /** @name three component constructor
     * 
     * @brief Initialize all components to separate values.
     * @param x: X value.
     * @param y: Y value.
     * @param z: Z value.
     */
    constexpr vector3d ( double _x, double _y, double _z ) noexcept : x { _x }, y { _y }, z { _z } {}

    /** @name Point3f constructor
     * 
     * @brief Construct from nite::Point3f object.
     * @param v: The Point3f object.
     */
    constexpr vector3d ( const nite::Point3f& v ) noexcept : x { v.x }, y { v.y }, z { v.z } {}

    /** @name Point3f explicit conversion
     * 
     * @brief Construct from nite::Point3f object.
     * @param v: The Point3f object.
     */
    explicit operator nite::Point3f () noexcept { return nite::Point3f { x, y, z }; }



    /* Simple arithmetic operations */
    constexpr vector3d operator+ ( const vector3d& other ) const noexcept { return vector3d { x + other.x, y + other.y, z + other.z }; }
    constexpr vector3d operator- ( const vector3d& other ) const noexcept { return vector3d { x - other.x, y - other.y, z - other.z }; }
    constexpr vector3d operator* ( const vector3d& other ) const noexcept { return vector3d { x * other.x, y * other.y, z * other.z }; }
    constexpr vector3d operator/ ( const vector3d& other ) const noexcept { return vector3d { x / other.x, y / other.y, z / other.z }; }
    constexpr vector3d& operator+= ( const vector3d& other ) noexcept { return * this = * this + other; }
    constexpr vector3d& operator-= ( const vector3d& other ) noexcept { return * this = * this - other; }
    constexpr vector3d& operator*= ( const vector3d& other ) noexcept { return * this = * this * other; }
    constexpr vector3d& operator/= ( const vector3d& other ) noexcept { return * this = * this / other; }
    constexpr vector3d operator* ( double scalar ) const noexcept { return * this * vector3d { scalar }; }
    constexpr vector3d operator/ ( double scalar ) const noexcept { return * this / vector3d { scalar }; }
    constexpr vector3d& operator*= ( double scalar ) noexcept { return * this = * this * scalar; }
    constexpr vector3d& operator/= ( double scalar ) noexcept { return * this = * this / scalar; }

    /* Comparison operators */
    constexpr bool operator== ( const vector3d& other ) const noexcept = default;
    constexpr bool operator!= ( const vector3d& other ) const noexcept = default;

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



    /** @name  get_num_tracked_users
     * 
     * @brief  Immediately return the number of tracked users.
     * @return Integer.
     */
    int get_num_tracked_users () const;

    /** @name  get_tracked_users
     * 
     * @brief  Immediately return an array of the currently tracked users. The timestamp and positions of the tracked users are projected to now.
     * @return Vector of users.
     */
    std::vector<tracked_user> get_tracked_users () const;

    /** @name  get_average_generation_time
     * 
     * @brief  Get the average time taken to generate depth data.
     * @return The average duration.
     */
    clock::duration get_average_generation_time () const;

    /** @name  wait_for_tracked_users
     * 
     * @brief  Wait on a timeout for new tracked users to become availible.
     * @param  timeout: The duration to wait for or time point to wait until.
     * @param  stoken: A stop token to cause a stop to waiting.
     * @param  frameid: A pointer to the ID of the last frame recieved by the caller. If there is already a more recent frame, this function will return immediately.
     *                  Frameid will also be updated to the ID of the frame just received.
     * @return True if new tracked users are availible, false otherwise.
     */
    bool wait_for_tracked_users ( std::stop_token stoken = std::stop_token {}, int * frameid = nullptr ) const;
    bool wait_for_tracked_users ( clock::duration timeout, std::stop_token stoken = std::stop_token {}, int * frameid = nullptr ) const;
    bool wait_for_tracked_users ( clock::time_point timeout, std::stop_token stoken = std::stop_token {}, int * frameid = nullptr ) const;

    /** @name  wait_for_detected_tracked_users
     * 
     * @brief  Wait on a timeout for new tracked users to become availible and where there is at least one user detected.
     * @param  timeout: The duration to wait for or time point to wait until.
     * @param  stoken: A stop token to cause a stop to waiting.
     * @param  frameid: A pointer to the ID of the last frame recieved by the caller. If there is already a more recent frame, this function will return immediately.
     *                  Frameid will also be updated to the ID of the frame just received.
     * @return True if new tracked users are availible, false otherwise.
     */
    bool wait_for_detected_tracked_users ( std::stop_token stoken = std::stop_token {}, int * frameid = nullptr ) const;
    bool wait_for_detected_tracked_users ( clock::duration timeout, std::stop_token stoken = std::stop_token {}, int * frameid = nullptr ) const;
    bool wait_for_detected_tracked_users ( clock::time_point timeout, std::stop_token stoken = std::stop_token {}, int * frameid = nullptr ) const;



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
    double camera_h_fov, camera_v_fov;
    double camera_depth;

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

    /* The global and detected frameid */
    int global_frameid { 1 }, detected_frameid { 1 };

    /* A mutex and condition variable to protect tracked_users */
    mutable std::mutex tracked_users_mx;
    mutable std::condition_variable_any tracked_users_cv;
    mutable std::condition_variable_any detected_tracked_users_cv;



    /** @name  onNewFrame
     * 
     * @brief  Overload of pure virtual method, which will be called when new frame data is available.
     * @param  [unnamed]: The user tracker for which new data is available.
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

