/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/controller.h
 * 
 * Header file for controlling the watergun servos.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_CONTROLLER_H_INCLUDED
#define WATERGUN_CONTROLLER_H_INCLUDED



/* INCLUDES */
#include <list>
#include <watergun/aimer.h>



/* DECLARATIONS */

namespace watergun
{
    /** class controller : aimer
     * 
     * Controlls the signals to the servos.
     */
    class controller;    
}



/* CONTROLLER DEFINITION */

/** class controller : aimer
 * 
 * Controlls the signals to the servos.
 */
class watergun::controller : public aimer
{
    
public:
    
    /** @name constructor
     * 
     * @brief Sets up controller, then begins controlling the servos.
     * @param _servo_period: The period to update servo positions.
     * @param _search_yaw_velocity: The yaw angular velocity in radians per second when searching for a user.
     * @param _water_rate: The velocity of the water leaving the watergun (depends on psi etc).
     * @param _air_resistance: Horizontal deceleration of the water, to model small amounts of air resistance.
     * @param _max_yaw_velocity: Maximum yaw angular velocity in radians per second.
     * @param _aim_period: The period of time in seconds with which to spire to be correctly aimed within.
     * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    controller ( clock::duration _servo_period, float _search_yaw_velocity, float _water_rate, float _air_resistance, float _max_yaw_velocity, clock::duration _aim_period, vector3d _camera_offset = vector3d {} );

    /** @name destructor
     * 
     * @brief Gracefully releases control of the servos.
     */
    ~controller ();



    /** @name  get_movement_plan.
     * 
     * @brief  Immediately returns the current movement plan.
     * @return The movement plan.
     */
    std::list<single_movement> get_movement_plan () const;

    /** @name  wait_get_movement_plan
     * 
     * @brief  Waits for the movement plan to update, then returns it.
     * @return The movement plan.
     */
    std::list<single_movement> wait_get_movement_plan () const;



    /** @name  get_current_movement
     * 
     * @brief  Immediately returns the current movement.
     * @return The single movement.
     */
    single_movement get_current_movement () const;

    /** @name  wait_get_current_movement
     * 
     * @brief  Waits for the movement plan to update, then returns the current movement.
     * @return The single movement.
     */
    single_movement wait_get_current_movement () const;



    /** @name  dynamic_project_tracked_user
     * 
     * @brief  Override which compensates for camera movement when projecting a tracked user.
     * @param  user: The user to update.
     * @param  timestamp: The new timestamp that their position should match. Defaults to now. Timestamps into the future are likely to lose accuracy.
     * @return The updated tracked user.
     */
    virtual tracked_user dynamic_project_tracked_user ( const tracked_user& user, clock::time_point timestamp = clock::now () ) const;



protected:

    /* The servo period */
    clock::duration servo_period;

    /* The angular velocity when searching for users */
    float search_yaw_velocity;



private:

    /* A double ended queue of single movements, representing past and future movements */
    std::list<single_movement> movement_plan;

    /* An iterator to the current movement being applied */
    std::list<single_movement>::iterator current_movement;

    /* A mutex and condition variable to protect and notify for the movement plan and iterator */
    mutable std::mutex movement_mx;
    mutable std::condition_variable movement_cv;

    /* The number of future single movements to store in the movement plan */
    int num_future_movements;



    /* A thread to handle the updating of the movement plan */
    std::thread movement_planner_thread;

    /* A thread to handle controlling the servos */
    std::thread servo_controller_thread;

    /* An atomic boolean telling threads when to end */
    std::atomic_bool end_threads { false };



    /** @name  movement_planner_thread_function
     * 
     * @brief  Function run by movement_planner_thread. Continuously updates movement_plan, and notifies the condition variable.
     * @return Nothing.
     */
    void movement_planner_thread_function ();

    /** @name  servo_controller_thread_function
     * 
     * @brief  Function run by servo_controller_thread. Continuously updates the servos based on the motion plan made by the movement planner thread.
     * @return Nothing.
     */
    void servo_controller_thread_function ();

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_CONTROLLER_H_INCLUDED */


