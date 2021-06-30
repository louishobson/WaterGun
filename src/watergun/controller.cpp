/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * src/watergun/controller.cpp
 * 
 * Implementation of include/watergun/controller.h
 * 
 */



/* INCLUDES */
#include <watergun/controller.h>



/* CONTROLLER IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Sets up controller, then begins controlling the servos.
 * @param _servo_period: The period to update servo positions.
 * @param _search_yaw_velocity: The yaw angular velocity in radians per second when searching for a user.
 * @param _water_rate: The velocity of the water leaving the watergun (depends on psi etc).
 * @param _max_yaw_velocity: Maximum yaw angular velocity in radians per second.
 * @param _aim_period: The period of time in seconds with which to spire to be correctly aimed within.
 * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @param _num_trackable_users: The max number of trackable users.
 * @param config_path: Path to a configuration file to use. If unspecified, the default local and global paths will be used.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::controller::controller ( const clock::duration _servo_period, const XnFloat _search_yaw_velocity, const XnFloat _water_rate, const XnFloat _max_yaw_velocity, const clock::duration _aim_period, const vector3d _camera_offset, const XnUInt16 _num_trackable_users, const std::string& config_path )
    : aimer ( _water_rate, _max_yaw_velocity, _aim_period, _camera_offset, _num_trackable_users, config_path )
    , servo_period { _servo_period }
    , search_yaw_velocity { _search_yaw_velocity }
    , num_future_movements { static_cast<int> ( std::chrono::seconds { 1 } / _aim_period ) }
{
    /* Push a non-movement from the beginning of all time to the movement plan. The duration will be updated on the movement planner thread's start.
     * Also push a search movement for the rest of all time to the movement plan. It's start point will also be updated on the same thread's start.
     */
    movement_plan.emplace_back ( zero_duration,  zero_time_point,  0., 0. );
    movement_plan.emplace_back ( large_duration, large_time_point, search_yaw_velocity, 0. );

    /* Set the current movement */
    current_movement = std::next ( movement_plan.begin () );

    /* Start the servo thread */
    servo_controller_thread = std::thread ( &watergun::controller::servo_controller_thread_function, this );

    /* Sleep for a short time */
    std::this_thread::sleep_for ( std::chrono::milliseconds { 100 } );

    /* Start the movement planner thread */
    movement_planner_thread = std::thread ( &watergun::controller::movement_planner_thread_function, this );
}



/** @name destructor
 * 
 * @brief Gracefully releases control of the servos.
 */
watergun::controller::~controller ()
{
    /* Set the end thread flag to true */
    end_threads = true;

    /* If any threads are running, join them */
    if ( movement_planner_thread.joinable () ) movement_planner_thread.join ();
    if ( servo_controller_thread.joinable () ) servo_controller_thread.join ();
}



/** @name  get_movement_plan.
 * 
 * @brief  Immediately returns the current movement plan.
 * @return The movement plan.
 */
std::list<watergun::controller::single_movement> watergun::controller::get_movement_plan () const
{
    /* Aquire the lock, then return the list */
    std::unique_lock lock { movement_mx };
    return movement_plan;
}



/** @name  wait_get_movement_plan
 * 
 * @brief  Waits for the movement plan to update, then returns it.
 * @return The movement plan.
 */
std::list<watergun::controller::single_movement> watergun::controller::wait_get_movement_plan () const
{
    /* Aquire the lock, wait on the condition variable, then return the list */
    std::unique_lock lock { movement_mx };
    movement_cv.wait ( lock );
    return movement_plan;
}



/** @name  get_current_movement
 * 
 * @brief  Immediately returns the current movement.
 * @return The single movement.
 */
watergun::controller::single_movement watergun::controller::get_current_movement () const
{
    /* Lock the mutex */
    std::unique_lock lock { movement_mx };

    /* While the current movement has a large timestamp, wait for it to update */
    while ( current_movement->timestamp == large_time_point ) { lock.unlock (); std::this_thread::sleep_for ( servo_period ); lock.lock (); }

    /* Return the movement */
    return * current_movement;
}



/** @name  wait_get_current_movement
 * 
 * @brief  Waits for the movement plan to update, then returns the current movement.
 * @return The single movement.
 */
watergun::controller::single_movement watergun::controller::wait_get_current_movement () const
{
    /* Lock the mutex */
    std::unique_lock lock { movement_mx };

    /* Wait on the condition variable */
    movement_cv.wait ( lock );

    /* While the current movement has a large timestamp, wait for it to update */
    while ( current_movement->timestamp == large_time_point ) { lock.unlock (); std::this_thread::sleep_for ( servo_period ); lock.lock (); }

    /* Return the movement */
    return * current_movement;
}



/** @name  dynamic_project_tracked_user
 * 
 * @brief  Override which compensates for camera movement when projecting a tracked user.
 * @param  user: The user to update.
 * @param  timestamp: The new timestamp that their position should match. Defaults to now. Timestamps into the future are likely to lose accuracy.
 * @return The updated tracked user.
 */
watergun::controller::tracked_user watergun::controller::dynamic_project_tracked_user ( const tracked_user& user, const clock::time_point timestamp ) const
{
    /* Find the early and late timestamps */
    const clock::time_point early_timestamp = std::min ( user.timestamp, timestamp ), late_timestamp = std::max ( user.timestamp, timestamp );

    /* Lock the mutex */
    std::unique_lock lock { movement_mx };

    /* Iterate backwards through the movement plan to find a movement that started before the early timestamp */
    auto movement_it = current_movement; while ( movement_it->timestamp > early_timestamp ) --movement_it;

    /* Get the timestamp of the current movement. Iterate over the movements, adding up the change in yaw, until the late timestamp is met. */
    XnFloat delta_yaw = 0.;
    for ( clock::time_point movement_timestamp = movement_it->timestamp; movement_timestamp < late_timestamp; movement_timestamp += movement_it++->duration )
    {
        /* Get the duration within the early and late times, that this movement occured */
        const clock::duration movement_duration = 
            std::clamp ( late_timestamp,  movement_timestamp, movement_timestamp + movement_it->duration ) - 
            std::clamp ( early_timestamp, movement_timestamp, movement_timestamp + movement_it->duration );

        /* Add to the delta yaw */
        delta_yaw += movement_it->yaw_rate * duration_to_seconds ( movement_duration ).count ();
    }

    /* Unlock the mutex */
    lock.unlock ();

    /* Project the user */
    tracked_user proj_user = project_tracked_user ( user, timestamp );

    /* Make up for the delta yaw */
    if ( timestamp == late_timestamp ) proj_user.com.X -= delta_yaw; else proj_user.com.X += delta_yaw;

    /* Return the projected user */
    return proj_user;
}




/** @name  movement_planner_thread_function
 * 
 * @brief  Function run by movement_planner_thread. Continuously updates movement_plan, and notifies the condition variable.
 * @return Nothing.
 */
void watergun::controller::movement_planner_thread_function ()
{
    /* Loop while not signalled to end */
    while ( !end_threads )
    {
        /* Get tracked users and choose a target. If there is no target, notify and continue. */
        tracked_user target = choose_target ( wait_get_tracked_users () );
        if ( target.com == vector3d {} ) { movement_cv.notify_all (); continue; }

        /* Lock the mutex */
        std::unique_lock lock { movement_mx };

        /* Erase movements not yet started */
        movement_plan.erase ( std::next ( current_movement ), movement_plan.end () );

        /* Add new future movements */
        movement_plan.splice ( movement_plan.end (), calculate_future_movements ( target, num_future_movements ) );

        /* Add a search movement to the end of the plan */
        movement_plan.emplace_back ( large_duration, large_time_point, std::copysign ( search_yaw_velocity, movement_plan.back ().yaw_rate ), 0. );

        /* Increment the current movement */
        std::advance ( current_movement, 1 );

        /* Signal the condition variable */
        movement_cv.notify_all ();
    }
}



/** @name  servo_controller_thread_function
 * 
 * @brief  Function run by servo_controller_thread. Continuously updates the servos based on the motion plan made by the movement planner thread.
 * @return Nothing.
 */
void watergun::controller::servo_controller_thread_function ()
{
    /* Create a permanent mutex lock, while not waiting */
    std::unique_lock lock { movement_mx };

    /* Loop while not signalled to end */
    while ( !end_threads )
    {
        /* Possibly increment the current movement, if it should finish now */
        if ( clock::now () > current_movement->timestamp + current_movement->duration ) ++current_movement;

        /* If just arrived at a new movement, update the timestamp and correct the previous movement's duration */
        if ( current_movement->timestamp == large_time_point ) { current_movement->timestamp = clock::now (); std::prev ( current_movement )->duration = current_movement->timestamp - std::prev ( current_movement )->timestamp; }

        /* Do servos */

        /* Wait on the condition variable or until the next servo update */
        movement_cv.wait_until ( lock, std::min ( clock::now () + servo_period, current_movement->timestamp + current_movement->duration ) );
    }
}