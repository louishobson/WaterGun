/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * src/watergun/solenoid.cpp
 * 
 * Implementation of include/watergun/solenoid.h
 * 
 */



/* INCLUDES */
#include <watergun/solenoid.h>



/* SOLENOID IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Give a pin number to set up the solenoid
 * @param _solenoid_pin: The pin number to use for the solenoid.
 */
watergun::solenoid::solenoid ( const int _solenoid_pin ) try
    : solenoid_pin { _solenoid_pin }
    , solenoid_gpio { solenoid_pin }
{
    /* Set the pin to output, and off */
    solenoid_gpio.dir ( mraa::DIR_OUT );
    solenoid_gpio.write ( 0 );
} catch ( const std::exception& e )
{
    /* Rethrow, stating that solenoid setup failed */
    throw watergun_exception { std::string { "Solenoid setup failed: " } + e.what () };
}