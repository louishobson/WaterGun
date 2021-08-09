/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/solenoid.h
 * 
 * Abstraction of solenoid valve control.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_SOLENOID_H_INCLUDED
#define WATERGUN_SOLENOID_H_INCLUDED



/* INCLUDES */
#include <mraa/gpio.hpp>
#include <watergun/watergun_exception.h>



/* DECLARATIONS */

namespace watergun
{
    /** class solenoid
     *
     * Class to abstract the control of a solenoid valve.
     */
    class solenoid;
}



/* SOLENOID DEFINITION */

/** class solenoid
 *
 * Class to abstract the control of a solenoid valve.
 */
class watergun::solenoid
{
public:

    /** @name constructor
     * 
     * @brief Give a pin number to set up the solenoid
     * @param _solenoid_pin: The pin number to use for the solenoid.
     */
    solenoid ( int _solenoid_pin );



    /** @name  power_on
     * 
     * @brief  Set the solenoid to be powered on.
     * @return Nothing.
     */
    void power_on () { if ( !solenoid_state ) solenoid_gpio.write ( solenoid_state = 1 ); }

    /** @name  power_off
     * 
     * @brief  Set the solenoid valve to be powered off.
     * @return Nothing.
     */
    void power_off () { if ( solenoid_state ) solenoid_gpio.write ( solenoid_state = 0 ); }

    /** @name  is_powered
     * 
     * @brief  Get if the solenoid is powered.
     * @return True if powered, false if not.
     */
    bool is_powered () noexcept { return solenoid_state; }



private:

    /* The pin number for the solenoid */
    int solenoid_pin;

    /* The GPIO object for the pin */
    mraa::Gpio solenoid_gpio;

    /* Whether the solenoid is currently powered */
    int solenoid_state { 0 };

};




/* HEADER GUARD */
#endif /* #ifndef WATERGUN_SOLENOID_H_INCLUDED */