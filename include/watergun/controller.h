/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/controller.h
 * 
 * Header file for controlling the watergun motors.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_CONTROLLER_H_INCLUDED
#define WATERGUN_CONTROLLER_H_INCLUDED



/* INCLUDES */
#include <list>
#include <watergun/aimer.h>
#include <watergun/solenoid.h>
#include <watergun/stepper.h>



/* DECLARATIONS */

namespace watergun
{
    /** class controller : aimer
     * 
     * Controlls the signals to the motors.
     */
    class controller;    
}



/* CONTROLLER DEFINITION */

/** class controller : aimer
 * 
 * Controlls the signals to the motors.
 */
class watergun::controller : public aimer
{
    
public:
    
    /** @name constructor
     * 
     * @brief Sets up controller, then begins controlling the motors.
     * @param _yaw_stepper: The yaw stepper motor to use.
     * @param _pitch_stepper: The pitch stepper motor to use.
     * @param _solenoid_valve: The solenoid valve to use.
     * @param _search_yaw_velocity: The yaw angular velocity in radians per second when searching for a user.
     * @param _water_rate: The velocity of the water leaving the watergun (depends on psi etc).
     * @param _air_resistance: Horizontal deceleration of the water, to model small amounts of air resistance.
     * @param _max_yaw_velocity: Maximum yaw angular velocity in radians per second.
     * @param _max_yaw_acceleration: Maximum yaw angular acceleration in radians per second squared.
     * @param _aim_period: The period of time in seconds with which to aspire to be correctly aimed within.
     * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    controller ( pwm_stepper& _yaw_stepper, gpio_stepper& _pitch_stepper, solenoid& _solenoid_valve, double _search_yaw_velocity, double _water_rate, double _air_resistance, double _max_yaw_velocity, double _max_yaw_acceleration, clock::duration _aim_period = clock::duration { 0 }, vector3d _camera_offset = vector3d {} );

    /** @name destructor
     * 
     * @brief Gracefully releases control of the motors.
     */
    ~controller ();



    /** @name  get_current_movement
     * 
     * @brief  Immediately returns the current movement.
     * @return The single movement.
     */
    single_movement get_current_movement () const;



    /** @name  dynamic_project_tracked_user
     * 
     * @brief  Override which compensates for camera movement when projecting a tracked user.
     * @param  user: The user to update.
     * @param  timestamp: The new timestamp that their position should match. Defaults to now. Timestamps into the future are likely to lose accuracy.
     * @return The updated tracked user.
     */
    tracked_user dynamic_project_tracked_user ( const tracked_user& user, clock::time_point timestamp = clock::now () ) const;



protected:

    /* The angular velocity when searching for users */
    double search_yaw_velocity;



private:

    /* The stepper motors */
    pwm_stepper&  yaw_stepper;
    gpio_stepper& pitch_stepper;

    /* Solenoid valve */
    solenoid& solenoid_valve;



    /* A double ended queue of single movements, representing past and future movements */
    std::list<single_movement> movement_plan;

    /* An iterator to the current movement being applied */
    std::list<single_movement>::iterator current_movement;

    /* A mutex to protect the movement plan and iterator */
    mutable std::mutex movement_mx;

    /* The number of future single movements to store in the movement plan */
    int num_future_movements;



    /* A thread to handle the updating of the movement plan */
    std::jthread controller_thread;



    /** @name  movement_planner_thread_function
     * 
     * @brief  Function run by controller_thread. Continuously updates movement_plan, and notifies the condition variable.
     * @param  stoken: The stop token for the jthread.
     * @return Nothing.
     */
    void movement_planner_thread_function ( std::stop_token stoken );

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_CONTROLLER_H_INCLUDED */


