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
 * @brief Sets up controller, then begins controlling the motors.
 * @param _yaw_stepper: The yaw stepper motor to use.
 * @param _pitch_stepper: The pitch stepper motor to use.
 * @param _search_yaw_velocity: The yaw angular velocity in radians per second when searching for a user.
 * @param _water_rate: The velocity of the water leaving the watergun (depends on psi etc).
 * @param _air_resistance: Horizontal deceleration of the water, to model small amounts of air resistance.
 * @param _max_yaw_velocity: Maximum yaw angular velocity in radians per second.
 * @param _max_yaw_acceleration: Maximum yaw angular acceleration in radians per second squared.
 * @param _aim_period: The period of time in seconds with which to aspire to be correctly aimed within.
 * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::controller::controller ( pwm_stepper& _yaw_stepper, gpio_stepper& _pitch_stepper, const double _search_yaw_velocity, const double _water_rate, const double _air_resistance, const double _max_yaw_velocity, const double _max_yaw_acceleration, const clock::duration _aim_period, const vector3d _camera_offset )
    : aimer ( _water_rate, _air_resistance, _max_yaw_velocity, _max_yaw_acceleration, _aim_period, _camera_offset )
    , yaw_stepper { _yaw_stepper }
    , pitch_stepper { _pitch_stepper }
    , search_yaw_velocity { _search_yaw_velocity }
    , num_future_movements { static_cast<int> ( std::chrono::seconds { 1 } / _aim_period ) }
{
    /* Push a non-movement from the beginning of all time to the movement plan. The duration will be updated on the movement planner thread's start.
     * Also push a search movement for the rest of all time to the movement plan. It's start point will also be updated on the same thread's start.
     */
    movement_plan.push_back ( single_movement { zero_duration,  zero_time_point,  0., 0. } );
    movement_plan.push_back ( single_movement { large_duration, large_time_point, search_yaw_velocity, 0. } );

    /* Set the current movement */
    current_movement = std::next ( movement_plan.begin () );

    /* Sleep for a short time */
    std::this_thread::sleep_for ( std::chrono::milliseconds { 100 } );

    /* Start the movement planner thread */
    controller_thread = std::jthread { [ this ] ( std::stop_token stoken ) { movement_planner_thread_function ( std::move ( stoken ) ); } };
}



/** @name destructor
 * 
 * @brief Gracefully releases control of the motors.
 */
watergun::controller::~controller ()
{
    /* Join the thread */
    if ( controller_thread.joinable () ) { controller_thread.request_stop (); controller_thread.join (); }
}



/** @name  get_current_movement
 * 
 * @brief  Immediately returns the current movement.
 * @return The single movement.
 */
watergun::controller::single_movement watergun::controller::get_current_movement () const
{
    /* Lock the mutex and return the current movement */
    std::unique_lock<std::mutex> lock { movement_mx };
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
    std::unique_lock<std::mutex> lock { movement_mx };

    /* Iterate backwards through the movement plan to find a movement that started before the early timestamp */
    auto movement_it = current_movement; while ( movement_it->timestamp > early_timestamp ) --movement_it;

    /* Iterate over the movements, adding up the change in yaw, until the late timestamp is met. */
    double delta_yaw = 0.; do
    {
        /* Get the duration within the early and late times, that this movement occured */
        const clock::duration movement_duration = 
            watergun::clamp ( late_timestamp,  movement_it->timestamp, movement_it->timestamp + movement_it->duration ) - 
            watergun::clamp ( early_timestamp, movement_it->timestamp, movement_it->timestamp + movement_it->duration );

        /* Add to the delta yaw */
        delta_yaw += movement_it->yaw_rate * duration_to_seconds ( movement_duration ).count ();
    } while ( ( ++movement_it )->timestamp < late_timestamp );

    /* Unlock the mutex */
    lock.unlock ();

    /* Project the user */
    tracked_user proj_user = project_tracked_user ( user, timestamp );

    /* Make up for the delta yaw */
    if ( timestamp == late_timestamp ) proj_user.com.x -= delta_yaw; else proj_user.com.x += delta_yaw;

    /* Return the projected user */
    return proj_user;
}



/** @name  movement_planner_thread_function
 * 
 * @brief  Function run by controller_thread. Continuously updates movement_plan, and notifies the condition variable.
 * @param  stoken: The stop token for the jthread.
 * @return Nothing.
 */
void watergun::controller::movement_planner_thread_function ( std::stop_token stoken )
{
    /* The last frameid */
    int frameid = 0;

    /* Wait for detected tracked users */
    wait_for_detected_tracked_users ( stoken, &frameid );

    /* Loop while not signalled to end */
    while ( !stoken.stop_requested () )
    {
        /* Get tracked users and choose a target. If there is no target, continue. */
        tracked_user target = choose_target ( get_tracked_users () );
        if ( target.com == vector3d {} ) continue;

        /* Calculate future movements */
        std::list<single_movement> future_movements = calculate_future_movements ( target, * current_movement, num_future_movements );

        /* Lock the mutex then erase movements not yet started */
        std::unique_lock<std::mutex> lock { movement_mx };
        movement_plan.erase ( std::next ( current_movement ), movement_plan.end () );

        /* Add new future movements */
        movement_plan.splice ( movement_plan.end (), std::move ( future_movements ) );

        /* Add a search movement to the end of the plan */
        movement_plan.push_back ( single_movement { large_duration, large_time_point, std::copysign ( search_yaw_velocity, movement_plan.back ().yaw_rate ), 0. } );

        /* Update the motors for every new movement */
        do {
            /* Lock the mutex if not already locked */
            if ( !lock.owns_lock () ) lock.lock ();

            /* Increment the current movement */
            std::advance ( current_movement, 1 );

            /* Set the start time and duration of previous movement */
            current_movement->timestamp = clock::now ();
            std::prev ( current_movement )->duration = current_movement->timestamp - std::prev ( current_movement )->timestamp;

            /* Set stepper velocities and positions */
            yaw_stepper.set_velocity ( current_movement->yaw_rate );
            pitch_stepper.set_position ( current_movement->ending_pitch, current_movement->duration );

            /* Unlock the mutex */
            lock.unlock ();

            /* Break if new tracked user data is availible */
        } while ( !wait_for_detected_tracked_users ( current_movement->duration, stoken, &frameid ) && !stoken.stop_requested () );
    }
}