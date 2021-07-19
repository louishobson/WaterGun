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



/* TRACKER IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Sets up the context and configures OpenNI/NITE for human recognition.
 * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @param _num_trackable_users: The max number of trackable users.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::tracker::tracker ( const vector3d _camera_offset, const XnUInt16 _num_trackable_users )
    : camera_offset { _camera_offset }
    , num_trackable_users { _num_trackable_users }
{
    /* Initialize the context */
    check_status ( context.Init (), "Failed to init context" );

    /* Create and start the depth generator */
    check_status ( depth_generator.Create ( context ), "Failed to init depth generator" );
    depth_generator.StartGenerating ();

    /* Wait for new depth data, then timestamps can be set accordingly */
    depth_generator.WaitAndUpdateData ();
    system_timestamp = clock::now ();
    openni_timestamp = depth_generator.GetTimestamp (); 

    /* Create and start the user generator. */
    check_status ( user_generator.Create ( context ), "Failed to init user generator" );
    user_generator.StartGenerating ();

    /* Set the protected camera properties */
    depth_generator.GetFieldOfView ( camera_fov );
    camera_depth = depth_generator.GetDeviceMaxDepth () / 1000.;

    /* Start the tracking thread */
    tracker_thread = std::thread { &tracker::tracker_thread_function, this };
}



/** @name destructor
 * 
 * @brief Gracefully releases the OpenNI context and handles.
 */
watergun::tracker::~tracker ()
{
    /* Set the end threads flag to true */
    end_threads = true;

    /* Join the tracker thread, if running */
    if ( tracker_thread.joinable () ) tracker_thread.join ();

    /* Stop generation */
    context.StopGeneratingAll ();

    /* Release contexts and handles */
    script_node.Release ();
    depth_generator.Release ();
    user_generator.Release ();
    context.Release ();
}



/** @name  get_tracked_users
 * 
 * @brief  Immediately return an array of the currently tracked users. The timestamp and positions of the tracked users are projected to now.
 * @return Vector of users.
 */
std::vector<watergun::tracker::tracked_user> watergun::tracker::get_tracked_users () const
{
    /* Lock the mutex, copy the tracked users, then unlock */
    std::unique_lock lock { tracked_users_mx };
    auto tracked_users_copy = tracked_users;
    lock.unlock ();

    /* Update their positions */
    for ( tracked_user& user : tracked_users_copy ) user = dynamic_project_tracked_user ( user );
    
    /* Return the tracked users */
    return tracked_users_copy;
}

/** @name  wait_get_tracked_users
 * 
 * @brief  Wait for data on tracked users to update, then return an array of them. The timestamp and positions of the tracked users are projected to now.
 * @return Vector of users.
 */
std::vector<watergun::tracker::tracked_user> watergun::tracker::wait_get_tracked_users () const
{
    /* Lock the mutex, wait on the condition variable, copy the tracked users, then unlock */
    std::unique_lock lock { tracked_users_mx };
    tracked_users_cv.wait ( lock );
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
    std::unique_lock lock { tracked_users_mx };
    return average_generation_time;
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



/** @name  tracker_thread_function
 * 
 * @brief  Function run by tracker_thread. Runs in a loop, updating tracked_users as new frames come in.
 * @return Nothing.
 */
void watergun::tracker::tracker_thread_function ()
{
    /* Loop while waiting for user data */
    while ( context.WaitOneUpdateAll ( user_generator ) == XN_STATUS_OK && !end_threads )
    {
        /* Lock the mutex */
        std::unique_lock lock { tracked_users_mx };

        /* Get the timestamp that the frame became availible */
        clock::time_point frame_timestamp = openni_to_system_timestamp ( depth_generator.GetTimestamp () );

        /* Recompute average computation time */
        average_generation_time = std::chrono::duration_cast<clock::duration> ( average_generation_time * 0.9 + ( clock::now () - frame_timestamp ) * 0.1 );

        std::cout << "!!!" << ( clock::now () - frame_timestamp ).count () << std::endl;

        /* Get the number of users availible and populate an array with those users' IDs */
        XnUInt16 num_users = num_trackable_users; std::vector<XnUserID> user_ids { num_trackable_users };
        user_generator.GetUsers ( user_ids.data (), num_users );

        /* Create a new tracked users array and iterate through the tracked users to populate it */
        std::vector<tracked_user> new_tracked_users;
        for ( const XnUserID user_id : user_ids )
        {
            /* Create the new user */
            tracked_user user { user_id, frame_timestamp };

            /* Get the COM for this user in cartesian coordinates. If the Z-coord is 0 (the user is lost), ignore this user. Else change to m/s and add the camera offset. */
            user_generator.GetCoM ( user_id, user.com );
            if ( user.com.Z == 0. ) continue; user.com = user.com / 1000. + camera_offset;

            /* Replace the COM with polar coordinates */
            user.com = { std::atan ( user.com.X / user.com.Z ), user.com.Y, std::sqrt ( user.com.X * user.com.X + user.com.Z * user.com.Z ) };

            /* See if a user of the same ID can be found in the last frame's tracked users */
            auto it = std::find_if ( tracked_users.begin (), tracked_users.end (), [ user_id ] ( const tracked_user& u ) { return u.id == user_id; } );
            
            /* If they were tracked in the last frame, dynamically project the user position back to the time of the last frame to calculate the COM rate. */
            if ( it != tracked_users.end () ) user.com_rate = rate_of_change ( dynamic_project_tracked_user ( user, it->timestamp ).com - it->com, user.timestamp - it->timestamp );

            /* Add the tracked user to the new array */
            new_tracked_users.push_back ( user );
        }

        /* Update the tracked users with the new array and notify the condition variable */
        tracked_users = std::move ( new_tracked_users );
        tracked_users_cv.notify_all ();
    }
}



/** @name  check_status
 * 
 * @brief  Takes a returned status, and checks that is is XN_STATUS_OK. If not, throws with the supplied reason and status description after a colon.
 * @param  status: The status returned from an OpenNI call.
 * @param  error_msg: The error message to set the exception to contain.
 */
void watergun::tracker::check_status ( const XnStatus status, const std::string& error_msg )
{
    /* If status != XN_STATUS_OK, throw an exception with error_msg as the message */
    if ( status != XN_STATUS_OK ) throw watergun_exception { error_msg + ": " + xnGetStatusString ( status ) };
}