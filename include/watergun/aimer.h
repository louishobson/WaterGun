/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/aimer.h
 * 
 * Header file for choosing the user to aim at, and producing motion by which the watergun should follow.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_AIMER_H_INCLUDED
#define WATERGUN_AIMER_H_INCLUDED



/* INCLUDES */
#include <array>
#include <complex>
#include <list>
#include <utility>
#include <watergun/tracker.h>



/* DECLARATIONS */

namespace watergun
{
    /** class aimer : tracker
     * 
     * Extenstion to tracker which adds aiming capabilities to tracking.
     */
    class aimer;
}



/* AIMER DEFINITION */

/** class aimer : tracker
 * 
 * Extenstion to tracker which adds aiming capabilities to tracking.
 */
class watergun::aimer : public tracker
{
public:

    /** struct gun_position
     * 
     * The position of the watergun in terms of yaw and pitch in radians.
     */
    struct gun_position { XnFloat yaw, pitch; };

    /** struct single_movement
     * 
     * Describes an amount of constant movement starting at a given point.
     * A movement plan is a list of single movements.
     */
    struct single_movement
    {
        /* The duration for which the movement should last, or has lasted for */
        clock::duration duration;

        /* The time at which the movement was started, or maximum if not started */
        clock::time_point timestamp;

        /* The rate of change of yaw during this movement */
        XnFloat yaw_rate;

        /* The exact pitch to end with at the end of this movement */
        XnFloat ending_pitch;
    };



    /** @name constructor
     * 
     * @brief Sets up tracker, then begins processing aim data.
     * @param _water_rate: The velocity of the water leaving the watergun (depends on psi etc).
     * @param _max_yaw_velocity: Maximum yaw angular velocity in radians per second.
     * @param _aim_period: The period of time in seconds with which to spire to be correctly aimed within.
     * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
     * @param _num_trackable_users: The max number of trackable users.
     * @param config_path: Path to a configuration file to use. If unspecified, the default local and global paths will be used.
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    aimer ( XnFloat _water_rate, XnFloat _max_yaw_velocity, clock::duration _aim_period, vector3d _camera_offset = vector3d {}, XnUInt16 _num_trackable_users = WATERGUN_MAX_TRACKABLE_USERS, const std::string& config_path = "" )
        : tracker { _camera_offset, _num_trackable_users, config_path }
        , water_rate { _water_rate }
        , max_yaw_velocity { _max_yaw_velocity }
        , aim_period { _aim_period }
    {}

    /** @name destructor
     * 
     * @brief Stops processing then destructs tracker.
     */
    ~aimer () = default;



    /** @name  calculate_aim
     * 
     * @brief  From a tracked user, find the yaw and pitch the watergun must shoot to hit the user for the given water velocity.
     * @param  user: The user to aim at.
     * @return A gun position, or NaN for both yaw and pitch if it is not possible to hit the user.
     */
    gun_position calculate_aim ( const tracked_user& user ) const;

    /** @name  choose_target
     * 
     * @brief  Choose a user to aim at from the given list.
     * @param  users: The users to aim at.
     * @return The tracked user the gun has chosen to aim for. The tracked user will be updated to represent the user's projected current position.
     */
    tracked_user choose_target ( const std::vector<tracked_user>& users ) const;

    /** @name  calculate_future_movements
     * 
     * @brief  Over the next n lots of aim periods, create a list of single movements to follow to keep on track with hitting a tracked user.
     *         The output list will be shorter than n elements, if it becomes not possible to hit the targeted user.
     * @param  user: The tracked user to aim for.
     * @param  n: The number of aim periods to single movements plans for.
     * @return The list of single movements forming a movement plan.
     */
    std::list<single_movement> calculate_future_movements ( const tracked_user& user, int n ) const;



protected:

    /* The water velocity */
    XnFloat water_rate;

    /* Maximum yaw angular velocity in radians per second */
    XnFloat max_yaw_velocity;

    /* The period of time with which the gun should aspire to be aiming at a user within */
    clock::duration aim_period;



private:

    /** @name  solve_quartic
     * 
     * @brief  Solves a quartic equation with given coeficients in decreasing power order.
     *         Special thanks to Sidney Cadot (https://github.com/sidneycadot) for the function implementation.
     * @param  c0: The first coeficient (x^4)...
     * @param  c4: The last coeficient (x^0).
     * @return Array of four (possibly complex) solutions.
     */
    static std::array<std::complex<double>, 4> solve_quartic ( const std::complex<double>& c0, const std::complex<double>& c1, const std::complex<double>& c2, const std::complex<double>& c3, const std::complex<double>& c4 );

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_AIMER_H_INCLUDED */


