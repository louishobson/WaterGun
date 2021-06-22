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
 * @param camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @param config: Path to a configuration file to use. If unspecified, the default local and global paths will be used.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::tracker::tracker ( const vector3d camera_offset, std::string config_path )
    : origin_offset { camera_offset }
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

    /* Set up user generator */
    check_status ( context.FindExistingNode ( XN_NODE_TYPE_USER, user_generator ), "Failed to init user generator"  );

    /* Start generation */
    context.StartGeneratingAll ();

    /* Start the tracking thread */
    tracker_thread = std::thread { &tracker::tracker_thread_callback, this };
}



/** @name destructor
 * 
 * @brief Gracefully releases the OpenNI context and handles.
 */
watergun::tracker::~tracker ()
{
    /* If the tracker thread is running, stop and join it */
    if ( tracker_thread.joinable () ) { end_tracker_thread = true; tracker_thread.join (); }

    /* Release contexts and handles */
    script_node.Release ();
    user_generator.Release ();
    context.Release ();
}



/** @name  tracker_thread_callback
 * 
 * @brief  Function run by tracker_thread. Runs in a loop, updating tracked_users as new frames come in.
 * @return Nothing.
 */
void watergun::tracker::tracker_thread_callback ()
{
    /* Loop while updating OpenNI buffers */
    while ( context.WaitOneUpdateAll ( user_generator ) == XN_STATUS_OK && !end_tracker_thread )
    {
        /* Get the timestamp */
        auto timestamp = std::chrono::system_clock::now ();

        /* Get the number of users availible and populate an array with those users' IDs */
        XnUInt16 num_users = WATERGUN_MAX_TRACKABLE_USERS; XnUserID user_ids [ WATERGUN_MAX_TRACKABLE_USERS ];
        user_generator.GetUsers ( user_ids, num_users );

        /* Create a new tracked users array */
        std::vector<tracked_user> new_tracked_users;

        /* Lock the mutex */
        std::unique_lock lock { tracked_users_mx };

        /* Loop through the users */
        for ( XnUInt16 i = 0; i < num_users; ++i )
        {
            /* Create the new user */
            tracked_user user { user_ids [ i ], timestamp };

            /* Get the COM for this user. If the Z-coord is 0 (the user is lost), ignore this user. Else change to m/s and add the origin offset. */
            user_generator.GetCoM ( user_ids [ i ], user.com );
            if ( user.com.Z == 0. ) continue; user.com = user.com / 1000. + origin_offset;

            /* Calculate the polar COM */
            user.polar_com = { std::atan ( user.com.X / user.com.Z ), user.com.Y, std::sqrt ( user.com.X * user.com.X + user.com.Z * user.com.Z ) };

            /* See if a user of the same ID can be found in the last frame's tracked users */
            auto it = std::find_if ( tracked_users.begin (), tracked_users.end (), [ id = user_ids [ i ] ] ( const tracked_user& u ) { return u.id == id; } );

            /* If the user was tracked in the last frame, update their rates of change */
            if ( it != tracked_users.end () )
            {
                user.com_rate = rate_of_change ( user.com - it->com, timestamp - it->timestamp );
                user.polar_com_rate = rate_of_change ( user.polar_com - it->polar_com, timestamp - it->timestamp );
            }

            /* Add the new tracked user */
            new_tracked_users.push_back ( user );
        }

        /* Move over the new tracked users */
        tracked_users = std::move ( new_tracked_users );

        /* Notify the condition variable */
        tracked_users_cv.notify_all ();
    }

    /* Notify the condition variable one last time */
    tracked_users_cv.notify_all ();
}



/** @name  get_tracked_users
 * 
 * @brief  Immediately return an array of the currently tracked users.
 * @return Vector of users.
 */
std::vector<watergun::tracker::tracked_user> watergun::tracker::get_tracked_users () const
{
    /* Lock the mutex */
    std::unique_lock lock { tracked_users_mx };
    
    /* Return the tracked users */
    return tracked_users;
}

/** @name  wait_tracked_users
 * 
 * @brief  Wait for data on tracked users to update, then return an array of them.
 * @return Vector of users.
 */
std::vector<watergun::tracker::tracked_user> watergun::tracker::wait_tracked_users () const
{
    /* Lock the mutex */
    std::unique_lock lock { tracked_users_mx };

    /* Wait on the condition variable */
    tracked_users_cv.wait ( lock );

    /* Return the tracked users */
    return tracked_users;
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