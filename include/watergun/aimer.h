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

    /** @name constructor
     * 
     * @brief Sets up tracker, then begins processing aim data.
     * @param water_velocity: The velocity of the water leaving the watergun (depends on psi etc).
     * @param camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
     * @param config: Path to a configuration file to use. If unspecified, the default local and global paths will be used.
     * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
     */
    aimer ( XnFloat water_velocity, vector3d camera_offset = vector3d {}, std::string config_path = "" )
        : tracker { camera_offset, config_path }
        , water_rate { water_velocity }
    {}

    /** @name destructor
     * 
     * @brief Stops processing then destructs tracker.
     */
    ~aimer () = default;



    /** @name  get_target
     * 
     * @brief  Immediately get the next target to hit, based off of the data currently availible.
     * @return The chosen tracked user. The timestamp will have time_since_epoch () == 0, if no user is found or can be hit.
     */
    tracked_user get_target () { return choose_target ( get_tracked_users () ); }

    /** @name  wait_get_target
     * 
     * @brief  Wait for the data on tracked users to update, then choose the next target to hit.
     * @return The chosen tracked user. The timestamp will have time_since_epoch () == 0, if no user is found or can be hit.
     */
    tracked_user wait_get_target () { return choose_target ( wait_get_tracked_users () ); }



    /** @name  calculate_aim
     * 
     * @brief  From a tracked user, find the yaw and pitch the watergun must shoot to hit the user for the given water velocity.
     * @param  user: The user to aim at.
     * @return A pair, containing the yaw and pitch in radians, or NaN for both if it is not possible to hit the user.
     */
    std::pair<XnFloat, XnFloat> calculate_aim ( const tracked_user& user ) const;



private:

    /* The water velocity */
    XnFloat water_rate;



    /** @name  choose_target
     * 
     * @brief  Choose a user to aim at from the given list.
     * @param  users: The users to aim at.
     * @return The tracked user the gun has chosen to aim for. The tracked user will be updated to represent the user's projected current position.
     */
    tracked_user choose_target ( const std::vector<tracked_user>& users );

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


