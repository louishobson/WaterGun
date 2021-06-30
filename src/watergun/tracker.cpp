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
 * @param config: Path to a configuration file to use. If unspecified, the default local and global paths will be used.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::tracker::tracker ( const vector3d _camera_offset, const XnUInt16 _num_trackable_users, std::string config_path )
    : camera_offset { _camera_offset }
    , num_trackable_users { _num_trackable_users }
{
    /* Return value and error storage */
    XnStatus status; xn::EnumerationErrors errors;

    /* If a config path is given, solely use that */
    if ( config_path.size () )
    {
        /* If does not exist, throw */
        if ( !std::filesystem::exists ( config_path ) ) throw watergun_exception { "Could not find user-supplied config file: " + config_path };
    } else
    
    /* Else use use the default paths */
    {
        /* Chose local over global, or throw if neither is found */
        if ( std::filesystem::exists ( WATERGUN_XML_LOCAL_CONFIG_PATH  ) ) config_path = WATERGUN_XML_LOCAL_CONFIG_PATH; else
        if ( std::filesystem::exists ( WATERGUN_XML_GLOBAL_CONFIG_PATH ) ) config_path = WATERGUN_XML_GLOBAL_CONFIG_PATH;
        else throw watergun_exception { "Could not find local or global default config files" };
    }

    /* Initialize the context */
    status = context.InitFromXmlFile ( config_path.c_str (), script_node, &errors );

    /* Handle case where could not create node (non-file error) */
    if ( status == XN_STATUS_NO_NODE_PRESENT )
    {
        /* Extract error and throw */
        XnChar error_string [ WATERGUN_MAX_ERROR_LENGTH ]; errors.ToString ( error_string, WATERGUN_MAX_ERROR_LENGTH );
        throw watergun_exception { std::string { "Failed to init context: " } + error_string };
    }

    /* Handle any other case where the status is not okay */
    else check_status ( status, "Failed to init context" );

    /* Set up depth and user generators */
    check_status ( context.FindExistingNode ( XN_NODE_TYPE_DEPTH, depth_generator ), "Failed to init depth generator" );
    check_status ( context.FindExistingNode ( XN_NODE_TYPE_USER,  user_generator  ), "Failed to init user generator"  );

    /* Set the protected camera properties */
    depth_generator.GetFieldOfView ( camera_fov );
    camera_depth = depth_generator.GetDeviceMaxDepth () / 1000.;

    /* Start generation */
    context.StartGeneratingAll ();

    /* Set the timestamps */
    system_timestamp = clock::now ();
    xnOSGetTimeStamp ( &openni_timestamp );

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
    /* Loop while updating depth buffers */
    while ( user_generator.WaitAndUpdateData () == XN_STATUS_OK && !end_threads )
    {
        /* Get the timestamp that the depth data became availible */
        clock::time_point timestamp = openni_to_system_timestamp ( depth_generator.GetTimestamp () );

        /* Get the number of users availible and populate an array with those users' IDs */
        XnUInt16 num_users = num_trackable_users; std::vector<XnUserID> user_ids { num_trackable_users };
        user_generator.GetUsers ( user_ids.data (), num_users );

        /* Create a new tracked users array */
        std::vector<tracked_user> new_tracked_users;

        /* Lock the mutex and loop through the detected users */
        std::unique_lock lock { tracked_users_mx };
        for ( const XnUserID user_id : user_ids )
        {
            /* Create the new user */
            tracked_user user { user_id, timestamp };

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