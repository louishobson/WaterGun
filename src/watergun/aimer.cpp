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
    , aim_period_s { duration_to_seconds ( aim_period ).count () }
{
    /* If the aim period is 0, update it to the length of a frame */
    if ( aim_period == clock::duration { 0 } ) aim_period = std::chrono::milliseconds { 1000 } / camera_output_mode.getFps (); 

    /* Set the log level of the movement model */
    movement_model.setLogLevel ( 0 );

    /* Create the initial basic movement model */
    movement_model = create_basic_movement_model ( movement_model_size_multiple );
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
    if ( time == INFINITY ) return { user.com.x, M_PI / 4., true };

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
    /* If n is larger than the current model size, increase the current model size */
    if ( n > movement_model.getNumCols () / 2 ) movement_model = create_basic_movement_model ( n );

    /* Specialize the model */
    auto gun_positions = specialize_movement_model ( movement_model, user, current_movement );

    /* Attempt to solve the problem */
    movement_model.dual ();

    /* If it failed, increase the model size and try again */
    while ( movement_model.isProvenPrimalInfeasible () )
    {
        /* Increase the model size */
        movement_model = create_basic_movement_model ( movement_model.getNumCols () / 2 + movement_model_size_multiple );

        /* Respecialize the model */
        gun_positions = specialize_movement_model ( movement_model, user, current_movement );

        /* Attempt to solve again */
        movement_model.dual ();
    }

    /* List of future movements */
    std::list<single_movement> future_movements;

    /* Populate the list of future movements */
    for ( int i = 0; i < n; ++i ) future_movements.push_back ( single_movement 
    { 
        aim_period, user.timestamp + aim_period * i, 
        movement_model.getColSolution () [ i ], 
        gun_positions.at ( i ).pitch, 
        movement_model.getColSolution () [ i + n ] < on_target_threshold && !gun_positions.at ( i ).out_of_range 
    } );

    /* Return the future movements */
    return future_movements;
}



/** @name  create_basic_movement_model
 * 
 * @brief  Create a linear programming model for n future movements into the future. The constraint bounds will need to be modified later for the model to work.
 * @param  n: The number of movements in the model.
 * @return ClpModel object.
 */
ClpModel watergun::aimer::create_basic_movement_model ( const int n ) const
{
    /* Tableaux contains variables x[n] and t[n], where x[i] is the velocity at the i'th period,
     * and t[i] is at least the absolute difference between x[i] and the on-target angle at that period.
     * The acceleration between periods, as well as the finishing angle are constrained.
     * The model is optimal, when t[0...n) are minimised, where t[i+1] is more desireable to minimise than t[i].
     */

    /* Create the tableaux */
    CoinPackedMatrix tableaux;

    /* Set the initial tableux size */
    tableaux.setDimensions ( n * 3 + 1, n * 2 );

    /* Create the constraint bounds */
    std::vector<double> constraint_lb ( n * 3 + 1 ), constraint_ub ( n * 3 + 1 );

    /* Set up the constraints which force t[i] >= | aim_period * x[i] - target angle | */
    for ( int i = 0; i < n; ++i ) for ( int j = 0; j < n * 2; ++j ) 
    {
        /* Set the 1st constraint: t[i] >= aim_period *  SUM x[0...i] - target angle    <=>   aim_period * -SUM x[0...i] + t[i] >= -target angle */
        tableaux.modifyCoefficient ( i * 2 + 0, j, j < n && j <= i ? -aim_period_s : ( j - n == i ? 1. : 0. ) );
            
        /* Set the 2nd constraint: t[i] >= aim_period * -SUM x[0...i] + target angle    <=>   aim_period *  SUM x[0...i] + t[i] >=  target angle */
        tableaux.modifyCoefficient ( i * 2 + 1, j, j < n && j <= i ?  aim_period_s : ( j - n == i ? 1. : 0. )  );

        /* Set the upper bounds to the maximum. Lower bound is set during specialization. */
        constraint_ub.at ( i * 2 ) = COIN_DBL_MAX; constraint_ub.at ( i * 2 + 1 ) = COIN_DBL_MAX;
    }

    /* Set up the constraints which enforce the maximum acceleration */
    for ( int i = 0; i < n + 1; ++i ) for ( int j = 0; j < n * 2; ++j ) 
    {
        /* Set up the constraint: -max acceleration <= ( x[i-1] - x[i] ) / aim_period <= max acceleration */
        tableaux.modifyCoefficient ( i + n * 2, j, j == i - 1 ? 1. / aim_period_s : ( j == i && j != n ? -1. / aim_period_s : 0. ) );

        /* Set the bounds */
        constraint_lb.at ( i + n * 2 ) = -max_yaw_acceleration; constraint_ub.at ( i + n * 2 ) = max_yaw_acceleration;
    }

    /* Create the variable bounds */
    std::vector<double> variable_lb ( n * 2 ), variable_ub ( n * 2 );
    std::fill_n ( variable_lb.begin (), n, -max_yaw_velocity ); std::fill_n ( variable_lb.begin () + n, n, 0. );
    std::fill_n ( variable_ub.begin (), n, +max_yaw_velocity ); std::fill_n ( variable_ub.begin () + n, n, COIN_DBL_MAX );
    variable_ub.back () = 0.; /* t[n-1] should only be 0, as this will force the gun to be aimed at the user by the end of the last period */

    /* Create the objective row */
    std::vector<double> objective_row ( n * 2, 0. );
    for ( int i = 0; i < n; ++i ) objective_row.at ( i + n ) = std::pow ( 1.1, i );

    /* Create the model and populate it */
    ClpModel clp_model; clp_model.loadProblem ( tableaux, variable_lb.data (), variable_ub.data (), objective_row.data (), constraint_lb.data (), constraint_ub.data () );

    /* Return the model */
    return clp_model;
}



/** @name  specialize_movement_model
 * 
 * @brief  Make a basic movement model specific to a given tracked user.
 * @param  clp_model: A reference to the model to refine.
 * @param  user: The tracked user to aim for.
 * @param  current_movement: The current movement of the gun.
 * @return An array of the gun positions at each period of the mode.
 */
std::vector<watergun::aimer::gun_position> watergun::aimer::specialize_movement_model ( ClpModel& clp_model, const tracked_user& user, const single_movement& current_movement ) const
{
    /* Get the number of variables in the model */
    const int n = clp_model.getNumCols () / 2;

    /* The return array of gun positions at the end of each period */
    std::vector<gun_position> gun_positions ( n );

    /* Modify the lower bounds on all the constraints defining all t[0...n) */
    for ( int i = 0; i < n; ++i )
    {
        /* Project the user and get the aim */
        tracked_user proj_user = project_tracked_user ( user, user.timestamp + aim_period * ( i + 1 ) );
        gun_positions.at ( i ) = calculate_aim ( proj_user );

        /* Set the bounds for the constraint */
        clp_model.setRowLower ( i * 2, -gun_positions.at ( i ).yaw ); clp_model.setRowLower ( i * 2 + 1, +gun_positions.at ( i ).yaw );        
    }

    /* Calculate the rate of change of the aiming yaw at the end of the periods. Correct for the off-chance that the user becomes unhittable between the two aimings. */
    gun_position aim_ext = calculate_aim ( project_tracked_user ( user, user.timestamp + aim_period * ( n + 1 ) ) );
    double aim_yaw_rate; if ( gun_positions.back ().out_of_range || aim_ext.out_of_range ) aim_yaw_rate = user.com_rate.x; else aim_yaw_rate = rate_of_change ( aim_ext.yaw - gun_positions.back ().yaw, aim_period );

    /* Modify the bounds on the first and last constraint that enforce the maximum acceleration */
    clp_model.setRowBounds ( n * 2, -max_yaw_acceleration - current_movement.yaw_rate / aim_period_s, +max_yaw_acceleration - current_movement.yaw_rate / aim_period_s );
    clp_model.setRowBounds ( n * 3, -max_yaw_acceleration +              aim_yaw_rate / aim_period_s, +max_yaw_acceleration +             aim_yaw_rate / aim_period_s );

    /* Return the gun positions */
    return gun_positions;
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
