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
 * @param _max_yaw_acceleration: Maximum yaw angular acceleration in radians per second squared.
 * @param _aim_period: The period of time in seconds with which to aspire to be correctly aimed within. Defaults to the length of a frame.
 * @param _camera_offset: The position of the camera relative to a custom origin. Defaults to the camera being the origin.
 * @throw watergun_exception, if configuration cannot be completed (e.g. config file or denice not found).
 */
watergun::aimer::aimer ( const double _water_rate, const double _air_resistance, const double _max_yaw_velocity, const double _max_yaw_acceleration, const clock::duration _aim_period, const vector3d _camera_offset )
    : tracker { _camera_offset }
    , water_rate { _water_rate }
    , air_resistance { _air_resistance }
    , max_yaw_velocity { _max_yaw_velocity }
    , max_yaw_acceleration { _max_yaw_acceleration }
    , aim_period { _aim_period }
{
    /* If the aim period is 0, update it to the length of a frame */
    if ( aim_period == clock::duration { 0 } ) aim_period = std::chrono::milliseconds { 1000 } / camera_output_mode.getFps (); 
}



/** @name  calculate_aim
 * 
 * @brief  From a tracked user, find the yaw and pitch the watergun must shoot to hit the user for the given water velocity.
 * @param  user: The user to aim at.
 * @return A gun position. If the user cannot be hit, yaw is set to the user's angle, and pitch is set to 45 degrees.
 */
watergun::aimer::gun_position watergun::aimer::calculate_aim ( const tracked_user& user ) const
{
    /* If the user is at the camera, return their angle for the yaw, and 0 degrees for the pitch */
    if ( ( user.com.z * user.com.z ) + ( user.com.y * user.com.y ) == 0. ) return { user.com.x, 0. };

    /* Solve the time quartic to test whether it is possible to hit the user */
    auto roots = solve_quartic
    (
        ( air_resistance * air_resistance * 0.25 ) + ( 9.81 * 9.81 * 0.25 ),
        ( air_resistance * user.com_rate.z ) + ( 9.81 * user.com_rate.y ),
        ( air_resistance * user.com.z ) + ( user.com_rate.z * user.com_rate.z ) + ( 9.81 * user.com.y ) + ( user.com_rate.y * user.com_rate.y ) - ( water_rate * water_rate ),
        ( user.com.z * user.com_rate.z * 2. ) + ( user.com.y * user.com_rate.y * 2. ),
        ( user.com.z * user.com.z ) + ( user.com.y * user.com.y )
    );

    /* Look for two real positive roots */
    double time = INFINITY;
    for ( const auto& root : roots ) if ( std::abs ( root.imag () ) < 1e-6 && root.real () > 0. && root.real () < time ) time = root.real ();

    /* If time is still infinity, there are no solutions, so return the user's position and 45 degrees */
    if ( time == INFINITY ) return { user.com.x, M_PI / 4. };

    /* Else produce the angles */
    return { user.com.x + user.com_rate.x * time, std::asin ( std::clamp ( ( user.com.y + user.com_rate.y * time + 4.905 * time * time ) / ( water_rate * time ), -1., 1. ) ) };
}



/** @name  choose_target
 * 
 * @brief  Immediately choose a user to aim at from the currently available data.
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
        double score = ( std::abs ( aim.yaw ) / ( camera_h_fov / 2. ) ) * -2. + 1. + ( user.com.z / camera_depth ) * -2. + 1. + ( user.com_rate.z / 7. ) * -1.;

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
 * @param  current_movement: The current movement of the gun.
 * @param  n: The number of aim periods to single movements plans for.
 * @return The list of single movements forming a movement plan.
 */
std::list<watergun::aimer::single_movement> watergun::aimer::calculate_future_movements ( const tracked_user& user, const single_movement& current_movement, int n ) const
{
    /* We will slowly increment the number of aim_periods it will take for the gun to aim at the user.
     * Each time, consider the gun accelerating from its initial position, and decelerating from its final position at max_yaw_acceleration rads/sec^2.
     * If it is not possible to cover enough distance to reach the target, then increment the number of aim periods it will take, until it is deemed possible.
     */

    /* Get a lower bound for the number of aim periods to intercept the user */
    int target_aim_periods = 150; //aim_periods_lower_bound ( user, current_movement );

    /* Get the maximum delta velocity */
    double max_delta_velocity = max_yaw_acceleration * duration_to_seconds ( aim_period ).count ();

    /* Create the tableaux */
    CoinPackedMatrix tableaux;

    /* Set the initial tableux size */
    tableaux.setDimensions ( target_aim_periods * 3 + 2, target_aim_periods * 2 );
    
    /* Create the constraint bounds */
    std::vector<double> constraint_lb; constraint_lb.reserve ( target_aim_periods * 3 + 2 );
    std::vector<double> constraint_ub; constraint_ub.reserve ( target_aim_periods * 3 + 2 );
        
    /* Set proj_user initially to user, and proj_user will always refer to the user one period after */
    tracked_user proj_user = user, proj_user_ext = project_tracked_user ( proj_user, proj_user.timestamp + aim_period );

    /* Get the aimings */
    gun_position aim = calculate_aim ( proj_user ), aim_ext = calculate_aim ( proj_user_ext );

    /* Get the rate of change of the yaw of the aim */
    double aim_yaw_rate = rate_of_change ( aim_ext.yaw - aim.yaw, aim_period );

    /* Set up the constraints which define the absolute variables and the total angle covered */
    for ( int i = 0; i < target_aim_periods; ++i )
    {
        /* Set the constraint: t[i] >= x[i] - aim_yaw_rate   <=>   -x[i] + t[i] >= -aim_yaw_rate */
        for ( int j = 0; j < target_aim_periods; ++j ) 
        { 
            tableaux.modifyCoefficient ( i * 2 + 0, j, i == j ? -1. : 0. );
            tableaux.modifyCoefficient ( i * 2 + 0, j + target_aim_periods, i == j ? 1. : 0. );
        }

        /* Set the bounds for the constraint */
        constraint_lb.push_back ( -aim_yaw_rate ); constraint_ub.push_back ( COIN_DBL_MAX );

        /* Set the constraint: t[i] >= aim_yaw_rate - x[i]   <=>   x[i] + t[i] >= aim_yaw_rate */
        for ( int j = 0; j < target_aim_periods; ++j )
        {
            tableaux.modifyCoefficient ( i * 2 + 1, j, i == j ? 1. : 0. );
            tableaux.modifyCoefficient ( i * 2 + 1, j + target_aim_periods, i == j ? 1. : 0. );
        }

        /* Set the bounds for the constraint */
        constraint_lb.push_back ( aim_yaw_rate ); constraint_ub.push_back ( COIN_DBL_MAX );

        /* Update the projections */
        proj_user = proj_user_ext; proj_user_ext = project_tracked_user ( proj_user, proj_user.timestamp + aim_period );

        /* Update the aimings */
        aim = aim_ext; aim_ext = calculate_aim ( proj_user_ext );

        /* Update the rate of change of the yaw of the aim */
        aim_yaw_rate = rate_of_change ( aim_ext.yaw - aim.yaw, aim_period );
    }

    /* Set up the constraints which enforce the maximum velocity change between periods */
    for ( int i = 0; i < target_aim_periods + 1; ++i ) 
    {
        /* Set constraint: -max_delta_velocity <= x[i-1] - x[i] <= max_delta_velocity */
        for ( int j = 0; j < target_aim_periods; ++j ) 
        {
            tableaux.modifyCoefficient ( i + target_aim_periods * 2, j, j == i - 1 ? 1. : ( j == i ? -1. : 0. ) );
            tableaux.modifyCoefficient ( i + target_aim_periods * 2, j + target_aim_periods, 0. );
        }

        /* Set bounds. Initial and final constraint should have special bounds */
        if ( i == 0                  ) { constraint_lb.push_back ( -max_delta_velocity - current_movement.yaw_rate ); constraint_ub.push_back ( max_delta_velocity - current_movement.yaw_rate ); } else
        if ( i == target_aim_periods ) { constraint_lb.push_back ( -max_delta_velocity + aim_yaw_rate              ); constraint_ub.push_back ( max_delta_velocity + aim_yaw_rate              ); } else
        { constraint_lb.push_back ( -max_delta_velocity ); constraint_ub.push_back ( max_delta_velocity ); }
    }

    /* Set up the total angle covered constraint: ( SUM x ) * aim_period == total angle covered */
    for ( int j = 0; j < target_aim_periods; ++j ) 
    { 
        tableaux.modifyCoefficient ( target_aim_periods * 3 + 1, j, 1. ); 
        tableaux.modifyCoefficient ( target_aim_periods * 3 + 1, j + target_aim_periods, 0. ); 
    }

    /* Set the bounds for the constraint */
    constraint_lb.push_back ( aim.yaw / duration_to_seconds ( aim_period ).count () ); constraint_ub.push_back ( aim.yaw / duration_to_seconds ( aim_period ).count () ); 

    /* Create the variable bounds */
    std::vector<double> variable_lb; variable_lb.reserve ( target_aim_periods * 2 );
    std::vector<double> variable_ub; variable_ub.reserve ( target_aim_periods * 2 );

    /* Set the variable bounds */
    for ( int i = 0; i < target_aim_periods; ++i ) { variable_lb.push_back ( -max_yaw_velocity ); variable_ub.push_back ( max_yaw_velocity ); }
    for ( int i = 0; i < target_aim_periods; ++i ) { variable_lb.push_back ( 0. ); variable_ub.push_back ( COIN_DBL_MAX ); }

    /* Create the objective row */
    std::vector<double> objective_row ( target_aim_periods * 2, 0. );

    /* Populate the objective row */
    for ( int i = 0; i < target_aim_periods; ++i ) objective_row.at ( i + target_aim_periods ) = 1000. * ( i + 1 );

    /* Create the simplex solver */
    ClpSimplex clp_simplex;

    /* Set log level */
    clp_simplex.setLogLevel ( 0 );

    /* Load the tableaux */
    clp_simplex.loadProblem ( tableaux, variable_lb.data (), variable_ub.data (), objective_row.data (), constraint_lb.data (), constraint_ub.data () );

    /* Attempt to solve the problem */
    clp_simplex.dual ();

    /* List of future movements */
    std::list<single_movement> future_movements;

    /* Populate the list of future movements */
    for ( int i = 0; i < target_aim_periods && i < n; ++i ) future_movements.push_back ( single_movement 
    { 
        aim_period, user.timestamp + aim_period * i, 
        clp_simplex.getColSolution () [ i ], 
        current_movement.ending_pitch + ( aim.pitch - current_movement.ending_pitch ) * ( i + 1 ) / target_aim_periods
    } );

    /* Return the future movements */
    return future_movements;
}



/** @name  aim_periods_lower_bound
 * 
 * @brief  Get a lower bound on the number of aim periods to go from a current yaw rate, and intercept a tracked user.
 * @param  user: The user to intercept.
 * @param  current_movement: The current movement.
 * @return A lower bound on the number of aim periods.
 */
int watergun::aimer::aim_periods_lower_bound ( const tracked_user& user, const single_movement& current_movement ) const
{
    /* Ignoring the max velocity, get the time to accelerate to the user at the maximum acceleration value (choose the +ve time) */
    double intercept_time = solve_quadratic ( 0.5 * std::copysign ( max_yaw_acceleration, user.com.x ), current_movement.yaw_rate - user.com_rate.x, -user.com.x ).at ( 0 ).real ();

    /* If the maximum velocity was exceeded, consider the cumulative time of accelerating and then travelling at the max velocity until the user is intercepted */
    if ( std::abs ( current_movement.yaw_rate + std::copysign ( max_yaw_acceleration, user.com.x ) * intercept_time ) > max_yaw_velocity ) 
    {
        /* Calculate the time taken to accelerate to full velocity in the direction of the user */
        double acceleration_time = std::abs ( std::copysign ( max_yaw_velocity, user.com.x ) - current_movement.yaw_rate ) / max_yaw_acceleration;

        /* Calculate the distance covered over that acceleration */
        double acceleration_distance = current_movement.yaw_rate * acceleration_time + 0.5 * std::copysign ( max_yaw_acceleration, user.com.x ) * acceleration_time * acceleration_time;

        /* Calculate the time at max velocity to intercept the user, from the gun angle after the acceleration time */
        double max_velocity_time = ( user.com.x - acceleration_distance ) / ( max_yaw_velocity - user.com_rate.x );

        /* Change the intercept time to be the sum of the acceleration and max velocity times */
        intercept_time = acceleration_time + max_velocity_time;
    }

    /* Return the minimum number of periods */
    return std::max ( std::ceil ( intercept_time / duration_to_seconds ( aim_period ).count () ), 1. ); 
}



/** @name  solve_quadratic
 * 
 * @brief  Solves a quadratic equation with given coeficients in decreasing power order.
 * @param  c0: The first coeficient (x^2).
 * @param  c1: The first coeficient (x^1).
 * @param  c2: The first coeficient (x^0).
 * @return An array of two (possibly complex) solutions.
 */
std::array<std::complex<double>, 2> watergun::aimer::solve_quadratic ( const std::complex<double>& c0, const std::complex<double>& c1, const std::complex<double>& c2 )
{
    const std::complex<double> sqrt_part = std::sqrt ( c1 * c1 - 4. * c0 * c2 );

    return { ( -c1 + sqrt_part ) / ( 2. * c0 ), ( -c1 - sqrt_part ) / ( 2. * c0 ) };
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
