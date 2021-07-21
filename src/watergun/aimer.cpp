/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * src/watergun/aimer.cpp
 * 
 * Implementation of include/watergun/aimer.h
 * 
 */



/* INCLUDES */
#include <watergun/aimer.h>



/* AIMER IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Sets up tracker, then begins processing aim data.
 * @param _water_rate: The velocity of the water leaving the watergun (depends on psi etc).
 * @param _air_resistance: Horizontal deceleration of the water, to model small amounts of air resistance.
 * @param _max_yaw_velocity: Maximum yaw angular velocity in radians per second.
 * @param _aim_period: The period of time in seconds with which to aspire to be correctly aimed within. Defaults to the length of a frame.
 * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @param _num_trackable_users: The max number of trackable users.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::aimer::aimer ( const XnFloat _water_rate, const XnFloat _air_resistance, const XnFloat _max_yaw_velocity, const clock::duration _aim_period, const vector3d _camera_offset, const XnUInt16 _num_trackable_users )
    : tracker { _camera_offset, _num_trackable_users }
    , water_rate { _water_rate }
    , air_resistance { _air_resistance }
    , max_yaw_velocity { _max_yaw_velocity }
    , aim_period { _aim_period }
{
    /* If the aim period is 0, update it to the length of a frame */
    if ( aim_period == clock::duration { 0 } ) aim_period = std::chrono::milliseconds { 1000 } / camera_output_mode.nFPS; 
}



/** @name  calculate_aim
 * 
 * @brief  From a tracked user, find the yaw and pitch the watergun must shoot to hit the user for the given water velocity.
 * @param  user: The user to aim at.
 * @return A gun position, or NaN for both yaw and pitch if it is not possible to hit the user.
 */
watergun::aimer::gun_position watergun::aimer::calculate_aim ( const tracked_user& user ) const
{
    /* Solve the time quartic to test whether it is possible to hit the user */
    auto roots = solve_quartic
    (
        ( air_resistance * air_resistance * 0.25 ) + ( 9.81 * 9.81 * 0.25 ),
        ( air_resistance * user.com_rate.Z ) + ( 9.81 * user.com_rate.Y ),
        ( air_resistance * user.com.Z ) + ( user.com_rate.Z * user.com_rate.Z ) + ( 9.81 * user.com.Y ) + ( user.com_rate.Y * user.com_rate.Y ) - ( water_rate * water_rate ),
        ( user.com.Z * user.com_rate.Z * 2. ) + ( user.com.Y * user.com_rate.Y * 2. ),
        ( user.com.Z * user.com.Z ) + ( user.com.Y * user.com.Y )
    );

    /* Look for two real positive roots */
    XnFloat time = INFINITY;
    for ( const auto& root : roots ) if ( std::abs ( root.imag () ) < 1e-6 && root.real () > 0. && root.real () < time ) time = root.real ();

    /* If time is still infinity, there are no solutions, so return NaN */
    if ( time == INFINITY ) return { std::nanf ( "" ), std::nanf ( "" ) };

    /* Else produce the angles */
    return { user.com.X + user.com_rate.X * time, std::asin ( ( user.com.Y + user.com_rate.Y * time + 4.905f * time * time ) / ( water_rate * time ) ) };
}



/** @name  choose_target
 * 
 * @brief  Immediately choose a user to aim at from the currently availible data.
 * @param  users: The users to aim at.
 * @return The tracked user the gun has chosen to aim for. The tracked user will be updated to represent the user's projected current position.
 */
watergun::aimer::tracked_user watergun::aimer::choose_target ( const std::vector<tracked_user>& users ) const
{
    /* Score which user to hit, the user with the highest score is chosen.
     * The required yaw to hit the user being at the center camera scores 1, at the edge of the FOV scores -1.
     * Being 0m away from the camera scores 1, being the maximum distance away scores -1.
     * Moving towards the camera at 7m/s scores 1, while away scores -1.
     */

    /* Set a minimum best score and store the best user to aim for */
    double best_score = -100; tracked_user best_user;

    /* Loop through the users */
    for ( const tracked_user& user : users )
    {
        /* Calculate aim and continue if it is not possible to hit the user */
        gun_position aim = calculate_aim ( user ); if ( std::isnan ( aim.yaw ) ) continue;

        /* Get their score */
        double score = ( std::abs ( aim.yaw ) / ( camera_fov.fHFOV / 2. ) ) * -2. + 1. + ( user.com.Z / camera_depth ) * -2. + 1. + ( user.com_rate.Z / 7. ) * -1.;

        /* If they have a new best score, update the best score and best user */
        if ( score > best_score ) { best_score = score; best_user = user; }
    }

    /* Return the best user to aim for */
    return best_user;
}



/** @name  calculate_future_movements
 * 
 * @brief  Over the next n lots of aim periods, create a list of single movements to follow to keep on track with hitting a tracked user.
 * @param  user: The tracked user to aim for.
 * @param  n: The number of aim periods to single movements plans for.
 * @return The list of single movements forming a movement plan.
 */
std::list<watergun::aimer::single_movement> watergun::aimer::calculate_future_movements ( const tracked_user& user, int n ) const
{
    /* Create the list of future movements */
    std::list<single_movement> future_movements;

    /* Store the change in yaw over the loop */
    XnFloat delta_yaw = 0.;

    /* Loop through n */
    for ( int i = 0; i < n; ++i )
    {
        /* Project the user */
        tracked_user proj_user = project_tracked_user ( user, user.timestamp + aim_period * ( i + 1 ) );

        /* Get the aim for the user. */
        gun_position aim = calculate_aim ( proj_user );

        /* If it is not possible to hit the user, break, else take away the delta yaw */
        if ( std::isnan ( aim.yaw ) ) break; aim.yaw -= delta_yaw;

        /* Calculate the yaw rate */
        XnFloat yaw_rate = clamp ( rate_of_change ( aim.yaw, aim_period ), -max_yaw_velocity, +max_yaw_velocity );

        /* Add the single movement */
        future_movements.push_back ( single_movement { aim_period, large_time_point, yaw_rate, aim.pitch } );

        /* Add to the delta yaw */
        delta_yaw += yaw_rate * duration_to_seconds ( aim_period ).count ();
    }

    /* Return the list */
    return future_movements;
}



/** @name  solve_quartic
 * 
 * @brief  Solves a quartic equation with given coeficients in decreasing power order.
 *         Special thanks to Sidney Cadot (https://github.com/sidneycadot) for the function implementation.
 * @param  c0: The first coeficient (x^4)...
 * @param  c4: The last coeficient (x^0).
 * @return Array of four (possibly complex) solutions.
 */
std::array<std::complex<double>, 4> watergun::aimer::solve_quartic ( const std::complex<double>& c0, const std::complex<double>& c1, const std::complex<double>& c2, const std::complex<double>& c3, const std::complex<double>& c4 ) noexcept
{
    const std::complex<double> a = c0;
    const std::complex<double> b = c1 / a;
    const std::complex<double> c = c2 / a;
    const std::complex<double> d = c3 / a;
    const std::complex<double> e = c4 / a;

    const std::complex<double> Q1 = c * c - 3. * b * d + 12. * e;
    const std::complex<double> Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d + 27. * b * b * e - 72. * c * e;
    const std::complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
    const std::complex<double> Q4 = 3. * b * b - 8. * c;

    const std::complex<double> Q5 = std::pow ( Q2 / 2. + std::sqrt ( Q2 * Q2 / 4. - Q1 * Q1 * Q1 ), 1. / 3. );
    const std::complex<double> Q6 = ( Q1 / Q5 + Q5 ) / 3.;
    const std::complex<double> Q7 = 2. * std::sqrt ( Q4 / 12. + Q6 );

    return
    {
        ( -b - Q7 - std::sqrt ( 4. * Q4 / 6. - 4. * Q6 - Q3 / Q7 ) ) / 4.,
        ( -b - Q7 + std::sqrt ( 4. * Q4 / 6. - 4. * Q6 - Q3 / Q7 ) ) / 4.,
        ( -b + Q7 - std::sqrt ( 4. * Q4 / 6. - 4. * Q6 + Q3 / Q7 ) ) / 4.,
        ( -b + Q7 + std::sqrt ( 4. * Q4 / 6. - 4. * Q6 + Q3 / Q7 ) ) / 4.
    };
}
