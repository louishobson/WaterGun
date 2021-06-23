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



/** @name  calculate_aim
 * 
 * @brief  From a tracked user, find the yaw and pitch the watergun must shoot to hit the user for the given water velocity.
 * @param  user: The user to aim at.
 * @return A pair, containing the yaw and pitch in radians, or NaN for both if it is not possible to hit the user.
 */
std::pair<XnFloat, XnFloat> watergun::aimer::calculate_aim ( const tracked_user& user ) const
{
    /* Solve the time quartic to test whether it is possible to hit the user */
    auto roots = solve_quartic
    (
        ( 24.059025 ),
        ( 9.81 * user.com_rate.Y ),
        ( 9.81 * user.com.Y ) + ( user.com_rate.Y * user.com_rate.Y ) + ( user.com_rate.Z * user.com_rate.Z ) - ( water_rate * water_rate ),
        ( 2. * user.com.Y * user.com_rate.Y ) + ( 2. * user.com.Z * user.com_rate.Z ),
        ( user.com.Z * user.com.Z ) + ( user.com.Y * user.com.Y )
    );

    /* Look for two real positive times */
    XnFloat time = INFINITY;
    for ( const auto& root : roots ) if ( std::abs ( root.imag () ) < 1e-6 && root.real () > 0. && root.real () < time ) time = root.real ();

    /* If time is still infinity, there are no solutions, so return NaN */
    if ( time == INFINITY ) return { NAN, NAN };

    /* Else produce the angles */
    return { user.com.X + user.com_rate.X * time, std::asin ( ( user.com.Y + user.com_rate.Y * time + 4.905 * time * time ) / ( water_rate * time ) ) };
}



/** @name  solve_quartic
 * 
 * @brief  Solves a quartic equation with given coeficients in decreasing power order.
 *         Special thanks to sidneycadot (https://github.com/sidneycadot) for the function definition.
 * @param  c0: The first coeficient (x^4)...
 * @param  c4: The last coeficient (x^0).
 * @return Array of four (possibly complex) solutions.
 */
std::array<std::complex<double>, 4> watergun::aimer::solve_quartic ( const std::complex<double> c0, const std::complex<double> c1, const std::complex<double> c2, const std::complex<double> c3, const std::complex<double> c4 )
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
