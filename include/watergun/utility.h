/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/stepper.h
 * 
 * Abstraction of stepper motor control, assuming stepper is controlled by DRV8825.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_UTILITY_H_INCLUDED
#define WATERGUN_UTILITY_H_INCLUDED



/* INCLUDES */

#include <chrono>


/* METHOD DEFINITIONS */

namespace watergun
{
    /** @name  duration_to_seconds
     * 
     * @brief  Get a duration in seconds as a double.
     * @param  dur: The duration to cast.
     * @return The duration in seconds.
     */
    template<class Rep, class Ratio>
    constexpr std::chrono::duration<double> duration_to_seconds ( std::chrono::duration<Rep, Ratio> dur ) noexcept
        { return std::chrono::duration_cast<std::chrono::duration<double>> ( dur ); }

    /** @name  rate_of_change
     * 
     * @brief  Calculate the rate of change, given value and time deltas.
     * @param  delta_v: The change in value.
     * @param  delta_t: The change in time.
     * @return Rate of change as a double.
     */
    template<class T, class Rep, class Ratio>
    constexpr T rate_of_change ( T delta_v, std::chrono::duration<Rep, Ratio> delta_t ) noexcept
        { return delta_v / duration_to_seconds ( delta_t ).count (); }

    /** @name  clamp
     * 
     * @brief  Clamps a number type between an upper and lower bound.
     * @param  value: The value to clamp.
     * @param  lower: The lower bound.
     * @param  upper: The upper bound.
     * @return A reference to value, lower or upper.
     */
    template<class T> const T& clamp ( const T& value, const T& lower, const T& upper ) noexcept { return std::max ( lower, std::min ( value, upper ) ); }

    
}


/* HEADER GUARD */
#endif /* #ifndef WATERGUN_UTILITY_H_INCLUDED */