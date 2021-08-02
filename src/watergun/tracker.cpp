/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * src/watergun/tracker.cpp
 * 
 * Implementation of include/watergun/tracker.h
 * 
 */



/* INCLUDES */
#include <watergun/tracker.h>



/* TRACKER STATIC MEMBER DEFINITION */

/* An arbitrarily large duration and duration */
const watergun::tracker::clock::duration   watergun::tracker::large_duration   { std::chrono::hours { 24 } };
const watergun::tracker::clock::time_point watergun::tracker::large_time_point { clock::now () + large_duration };

/* Zero duration and time point */
const watergun::tracker::clock::duration   watergun::tracker::zero_duration   { clock::duration::zero () };
const watergun::tracker::clock::time_point watergun::tracker::zero_time_point { clock::time_point {} };



/* TRACKER IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Sets up the context and configures OpenNI/NITE for human recognition.
 * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::tracker::tracker ( const vector3d _camera_offset )
    : camera_offset { _camera_offset }
{
    /* Initialize OpenNI and NiTE */
    check_status ( openni::OpenNI::initialize (), "Failed to initialize OpenNI" );
    check_status ( nite::NiTE::initialize (), "Failed to initialize NiTE" );

    /* Open the Kinect device then create a video stream and user tracker */
    check_status ( device.open ( openni::ANY_DEVICE ), "Failed to open device" );
    check_status ( depth_stream.create ( device, openni::SensorType::SENSOR_DEPTH ), "Failed to open depth stream" );
    check_status ( user_tracker.create ( &device ), "Failed to create user tracker" );

    /* Set the protected camera properties */
    camera_h_fov = depth_stream.getHorizontalFieldOfView ();
    camera_v_fov = depth_stream.getVerticalFieldOfView ();
    camera_depth = depth_stream.getMaxPixelValue ();
    camera_output_mode = depth_stream.getVideoMode ();

    /* Sync clocks */
    sync_clocks ();

    /* Start listening to the user tracker */
    user_tracker.addNewFrameListener ( this );
}



/** @name destructor
 * 
 * @brief Gracefully releases the OpenNI context and handles.
 */
watergun::tracker::~tracker ()
{
    /* Remove user tracker listener */
    user_tracker.removeNewFrameListener ( this );

    /* Destrroy user tracker, depth stream and device */
    user_tracker.destroy ();
    depth_stream.destroy ();
    device.close ();
    
    /* Shutdown NiTE and OpenNI */
    nite::NiTE::shutdown ();
    openni::OpenNI::shutdown ();
}



/** @name  get_num_tracked_users
 * 
 * @brief  Immediately return the number of tracked users.
 * @return Integer.
 */
int watergun::tracker::get_num_tracked_users () const
{
    /* Lock the mutex then return the number of tracked users */
    std::unique_lock<std::mutex> lock { tracked_users_mx };
    return tracked_users.size ();
}



/** @name  get_tracked_users
 * 
 * @brief  Immediately return an array of the currently tracked users. The timestamp and positions of the tracked users are projected to now.
 * @return Vector of users.
 */
std::vector<watergun::tracker::tracked_user> watergun::tracker::get_tracked_users () const
{
    /* Lock the mutex, copy the tracked users, then unlock */
    std::unique_lock<std::mutex> lock { tracked_users_mx };
    auto tracked_users_copy = tracked_users;
    lock.unlock ();

    /* Update their positions */
    for ( tracked_user& user : tracked_users_copy ) user = dynamic_project_tracked_user ( user );
    
    /* Return the tracked users */
    return tracked_users_copy;
}



/** @name  get_average_generation_time
 * 
 * @brief  Get the average time taken to generate depth data.
 * @return The average duration.
 */
watergun::tracker::clock::duration watergun::tracker::get_average_generation_time () const
{
    /* Lock the mutex and return the value */
    std::unique_lock<std::mutex> lock { tracked_users_mx };
    return average_generation_time;
}




/** @name  wait_for_tracked_users
 * 
 * @brief  Wait on a timeout for new tracked users to become availible.
 * @param  timeout: The duration to wait for or time point to wait until.
 * @param  stoken: A stop token to cause a stop to waiting.
 * @param  frameid: A pointer to the ID of the last frame recieved by the caller. If there is already a more recent frame, this function will return immediately.
 *                  Frameid will also be updated to the ID of the frame just received.
 * @return True if new tracked users are availible, false otherwise.
 */
bool watergun::tracker::wait_for_tracked_users ( std::stop_token stoken, int * frameid ) const
{
    /* Call the timepoint version */
    return wait_for_tracked_users ( clock::time_point::max (), std::move ( stoken ), frameid );
}
bool watergun::tracker::wait_for_tracked_users ( clock::duration timeout, std::stop_token stoken, int * frameid ) const
{
    /* Call the timepoint version */
    return wait_for_tracked_users ( clock::now () + timeout, std::move ( stoken ), frameid );
}
bool watergun::tracker::wait_for_tracked_users ( clock::time_point timeout, std::stop_token stoken, int * frameid ) const
{
    /* Lock the mutex */
    std::unique_lock<std::mutex> lock { tracked_users_mx };

    /* If frameid is null, create a new variable for it to point to, which is equal to the current frameid */
    int alt_frameid = global_frameid;
    if ( !frameid ) frameid = &alt_frameid;

    /* Wait for a new frame to become availible */
    tracked_users_cv.wait_until ( lock, stoken, timeout, [ this, &stoken, frameid ] { return ( frameid && * frameid < global_frameid ) || stoken.stop_requested (); } );

    /* Update the frameid and return */
    if ( * frameid < global_frameid ) return ( * frameid = global_frameid ); else return false;
}



/** @name  wait_for_detected_tracked_users
 * 
 * @brief  Wait on a timeout for new tracked users to become availible and where there is at least one user detected.
 * @param  timeout: The duration to wait for or time point to wait until.
 * @param  stoken: A stop token to cause a stop to waiting.
 * @param  frameid: A pointer to the ID of the last frame recieved by the caller. If there is already a more recent frame, this function will return immediately.
 *                  Frameid will also be updated to the ID of the frame just received.
 * @return True if new tracked users are availible, false otherwise.
 */
bool watergun::tracker::wait_for_detected_tracked_users ( std::stop_token stoken, int * frameid ) const
{
    /* Call the timepoint version */
    return wait_for_tracked_users ( clock::time_point::max (), std::move ( stoken ), frameid );
}
bool watergun::tracker::wait_for_detected_tracked_users ( clock::duration timeout, std::stop_token stoken, int * frameid ) const
{
    /* Call the timepoint version */
    return wait_for_tracked_users ( clock::now () + timeout, std::move ( stoken ), frameid );
}
bool watergun::tracker::wait_for_detected_tracked_users ( clock::time_point timeout, std::stop_token stoken, int * frameid ) const
{
    /* Lock the mutex */
    std::unique_lock<std::mutex> lock { tracked_users_mx };

    /* If frameid is null, create a new variable for it to point to, which is equal to the current frameid */
    int alt_frameid = detected_frameid;
    if ( !frameid ) frameid = &alt_frameid;

    /* Wait for a new frame to become availible */
    detected_tracked_users_cv.wait_until ( lock, stoken, timeout, [ this, &stoken, frameid ] { return * frameid < detected_frameid || stoken.stop_requested (); } );

    /* Update the frameid and return */
    if ( * frameid < detected_frameid ) return ( * frameid = detected_frameid ); else return false;
}



/** @name  project_tracked_user
 * 
 * @brief  Update a user's position to match a new timestamp, given that they follow the same velocity.
 * @param  user: The user to update.
 * @param  timestamp: The new timestamp that their position should match.
 * @return The updated tracked user.
 */
watergun::tracker::tracked_user watergun::tracker::project_tracked_user ( const tracked_user& user, const clock::time_point timestamp ) const
{
    /* Return a tracked user with updated timestamp and position */
    return tracked_user
    {
        user.id, timestamp,
        user.com + user.com_rate * duration_to_seconds ( timestamp - user.timestamp ).count (),
        user.com_rate,
    };
}



/** @name  onNewFrame
 * 
 * @brief  Overload of pure virtual method, which will be called when new frame data is available.
 * @param  [unnamed]: The user tracker for which new data is available.
 * @return Nothing.
 */
void watergun::tracker::onNewFrame ( nite::UserTracker& ) 
{
    /* Read the new frame */
    nite::UserTrackerFrameRef frame;
    check_status ( user_tracker.readFrame ( &frame ), "Failed to read user tracker frame" );
       
    /* Lock the mutex */
    std::unique_lock<std::mutex> lock { tracked_users_mx };

    /* Get the timestamp that the frame became available */
    clock::time_point frame_timestamp = openni_to_system_timestamp ( frame.getTimestamp () );

    /* Recompute average computation time */
    average_generation_time = std::chrono::duration_cast<clock::duration> ( average_generation_time * 0.95 + ( clock::now () - frame_timestamp ) * 0.05 );

    /* Get the users */
    const auto& users = frame.getUsers ();

    /* Create a new tracked users array and iterate through the tracked users to populate it */
    std::vector<tracked_user> new_tracked_users;
    for ( int i = 0; i < users.getSize (); ++i )
    {
        /* Create the new user */
        tracked_user user { users [ i ].getId (), frame_timestamp, users [ i ].getCenterOfMass (), vector3d {} };

        /* If the Z-coord is 0 (the user is lost), ignore this user. Else change to meters and add the camera offset. */
        if ( user.com.z == 0. ) continue; user.com = user.com / 1000. + camera_offset;

        /* Replace the COM with polar coordinates */
        user.com = { std::atan ( user.com.x / user.com.z ), user.com.y, std::sqrt ( user.com.x * user.com.x + user.com.z * user.com.z ) };

        /* See if a user of the same ID can be found in the last frame's tracked users */
        auto it = std::find_if ( tracked_users.begin (), tracked_users.end (), [ &user ] ( const tracked_user& u ) { return u.id == user.id; } );
        
        /* If they were tracked in the last frame, dynamically project the user position back to the time of the last frame to calculate the COM rate. */
        if ( it != tracked_users.end () ) user.com_rate = it->com_rate * 0.5 + rate_of_change ( dynamic_project_tracked_user ( user, it->timestamp ).com - it->com, user.timestamp - it->timestamp ) * 0.5;

        /* Use the minimum COM rate values to reduce noise */
        if ( std::abs ( user.com_rate.x ) < min_com_rate.x ) user.com_rate.x = 0;
        if ( std::abs ( user.com_rate.y ) < min_com_rate.y ) user.com_rate.y = 0;
        if ( std::abs ( user.com_rate.z ) < min_com_rate.z ) user.com_rate.z = 0;

        /* Add the tracked user to the new array */
        new_tracked_users.push_back ( user );
    }

    /* Update the tracked users with the new array */
    tracked_users = std::move ( new_tracked_users );

    /* Increment the frame IDs and possibly resync clocks */
    if ( ++global_frameid % clock_sync_period == 0 ) sync_clocks ();
    if ( tracked_users.size () ) ++detected_frameid;

    /* Notify the condition variables */
    tracked_users_cv.notify_all ();
    if ( tracked_users.size () ) detected_tracked_users_cv.notify_all ();
}



/** @name  sync_clocks
 * 
 * @brief  Synchronize the OpenNI and system timestamps.
 * @return Nothing.
 */
void watergun::tracker::sync_clocks ()
{
    /* Start the depth stream */
    depth_stream.start ();

    /* Read the depth frame */
    openni::VideoFrameRef frame;
    depth_stream.readFrame ( &frame );
    depth_stream.readFrame ( &frame );

    /* Set clocks */
    system_timestamp = clock::now ();
    openni_timestamp = frame.getTimestamp ();

    /* Stop the depth stream */
    depth_stream.stop ();
}



/** @name  check_status
 * 
 * @brief  Takes a OpenNI or NiTE status, and checks that is is okay. If not, throws with the supplied reason and status description after a colon.
 * @param  status: The status returned from an OpenNI or NiTE call.
 * @param  error_msg: The error message to set the exception to contain.
 */
void watergun::tracker::check_status ( const openni::Status status, const std::string& error_msg )
{
    /* If status != STATUS_OK, throw an exception with error_msg as the message */
    if ( status != openni::STATUS_OK ) throw watergun_exception { error_msg };
}
void watergun::tracker::check_status ( nite::Status status, const std::string& error_msg )
{
    /* If status != STATUS_OK, throw an exception with error_msg as the message */
    if ( status != nite::STATUS_OK ) throw watergun_exception { error_msg };
}