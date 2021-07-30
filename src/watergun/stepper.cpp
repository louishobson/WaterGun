/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * src/watergun/stepper.cpp
 * 
 * Implementation of include/watergun/stepper.h
 * 
 */



/* INCLUDES */
#include <watergun/stepper.h>



/* IMPLEMENTATION OF STEPPER_BASE */



/** @name constructor
 * 
 * @brief Set the motor stepping angle, controlling GPIO pins and min PWM frequency.
 * @param _step_size: The number of radians per whole step of the motor.
 * @param _min_step_freq: The minimum PWM frequency before microstepping is increased.
 * @param _step_pin: The pin number for the step control.
 * @param _dir_pin: The pin number for direction control.
 * @param _microstep_pin_0: The first pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _microstep_pin_1: The second pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _microstep_pin_2: The third pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _sleep_pin: The pin number for motor sleep control, or -1 for not present.
 */
watergun::stepper_base::stepper_base ( const double _step_size, const double _min_step_freq, const int _step_pin, const int _dir_pin, const int _microstep_pin_0, const int _microstep_pin_1, const int _microstep_pin_2, const int _sleep_pin ) try
    : step_size { step_size }
    , min_step_freq { _min_step_freq }
    , step_pin { _step_pin }
    , dir_pin { _dir_pin }
    , microstep_pin_0 { _microstep_pin_0 }
    , microstep_pin_1 { _microstep_pin_1 }
    , microstep_pin_2 { _microstep_pin_2 }
    , sleep_pin { _sleep_pin }
{
    /* Open pins and consider the implications of each pin number being -1 */
    if ( step_pin <  0 ) throw watergun_exception { "Stepper step pin cannot be always off" };
    if ( dir_pin  >= 0 ) dir_gpio = create_output_gpio ( dir_pin ); else throw watergun_exception { "Stepper dir pin cannot be always off" };
    if ( microstep_pin_2 >= 0 ) microstep_gpio_2 = create_output_gpio ( microstep_pin_2 );
    if ( microstep_pin_1 >= 0 ) microstep_gpio_1 = create_output_gpio ( microstep_pin_1 );
    if ( microstep_pin_0 >= 0 ) microstep_gpio_0 = create_output_gpio ( microstep_pin_0 );
    if ( sleep_pin >= 0 ) sleep_gpio = create_output_gpio ( sleep_pin );

    /* Sort out availible microstepping numbers */
    if ( microstep_pin_0 == -1 ) availible_microstep_numbers.remove_if ( [] ( int m ) { return  m & 1; } ); else
    if ( microstep_pin_0 == -2 ) availible_microstep_numbers.remove_if ( [] ( int m ) { return ~m & 1; } );
    if ( microstep_pin_1 == -1 ) availible_microstep_numbers.remove_if ( [] ( int m ) { return  m & 2; } ); else
    if ( microstep_pin_1 == -2 ) availible_microstep_numbers.remove_if ( [] ( int m ) { return ~m & 2; } );
    if ( microstep_pin_2 == -1 ) availible_microstep_numbers.remove_if ( [] ( int m ) { return  m & 4; } ); else
    if ( microstep_pin_2 == -2 ) availible_microstep_numbers.remove_if ( [] ( int m ) { return ~m & 4; } );
} catch ( const std::exception& e )
{
    /* Rethrow, stating that stepper motor setup failed */
    throw watergun_exception { std::string { "Stepper motor setup failed: " } + e.what () };
}



/** @name  choose_microstep_number
 * 
 * @brief  Choose the best microstep number for a given angular velocity.
 * @param  velocity: The angular velocity to choose based on.
 * @return The microstep number (one of availible_microstep_numbers).
 */
int watergun::stepper_base::choose_microstep_number ( const double velocity ) const
{
    /* If there is only one availible, return it immediately */
    if ( availible_microstep_numbers.size () == 1 ) return availible_microstep_numbers.front ();

    /* Get the desired microstep number to keep the step rate just above the min_step_freq */
    int microstep_number = std::ceil ( std::log2 ( ( step_size * min_step_freq ) / std::abs ( velocity ) ) );

    /* Find the best availible match for the microstepping number */
    auto match = std::find_if ( availible_microstep_numbers.begin (), availible_microstep_numbers.end (), [ microstep_number ] ( int m ) { return m >= microstep_number; } );
    
    /* Return the best availible number */
    return ( match == availible_microstep_numbers.end () ? * --match : * match );
}



/** @name  create_pwm
 * 
 * @brief  Create a PWM pin
 * @param  pin: The pin to create the PWM object on.
 * @return The PWM object.
 */
mraa::Pwm watergun::stepper_base::create_pwm ( int pin )
{
    /* Create object from pin number */
    mraa::Pwm pwm { pin };
    
    /* Disable the pin, and set the duty cycle to a half */
    pwm.enable ( false );
    pwm.write ( 0.5 );
    
    /* Return the object */
    return pwm;
}



/** @name  create_output_gpio
 * 
 * @brief  Create an output GPIO pin.
 * @param  pin: The pin to create the GPIO object on.
 * @return The GPIO object.
 */
mraa::Gpio watergun::stepper_base::create_output_gpio ( int pin )
{
    /* Create object from the pin number */
    mraa::Gpio gpio { pin };

    /* Set to output and write a 0 */
    gpio.dir ( mraa::Dir::DIR_OUT );
    gpio.write ( 0 );

    /* Return the object */
    return gpio;
}



/* PWM_STEPPER IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Set the motor stepping angle, controlling GPIO pins and min PWM frequency.
 * @param _step_size: The number of radians per whole step of the motor.
 * @param _min_step_freq: The minimum PWM frequency before microstepping is increased.
 * @param _step_pin: The pin number for the step control.
 * @param _dir_pin: The pin number for direction control.
 * @param _microstep_pin_0: The first pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _microstep_pin_1: The second pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _microstep_pin_2: The third pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _sleep_pin: The pin number for motor sleep control, or -1 for not present.
 */
watergun::pwm_stepper::pwm_stepper ( const double _step_size, const double _min_step_freq, const int _step_pin, const int _dir_pin, const int _microstep_pin_0, const int _microstep_pin_1, const int _microstep_pin_2, const int _sleep_pin ) try
    : stepper_base { _step_size, _min_step_freq, _step_pin, _dir_pin, _microstep_pin_0, _microstep_pin_1, _microstep_pin_2, _sleep_pin }
{
    /* Initialize the PWM pin */
    step_pwm.open ( step_pin );    
} catch ( const std::exception& e )
{
    /* Rethrow, stating that stepper motor setup failed */
    throw watergun_exception { std::string { "Stepper motor setup failed: " } + e.what () };
}



/** @name pure virtual destructor
 * 
 * @brief A stepper_base object should be abstract.
 */
watergun::stepper_base::~stepper_base () {};



/** @name  set_velocity
 * 
 * @brief  Set a new rotation velocity.
 * @param  velocity: The new angular velocity in rad/sec, positive meaning clockwise and vice versa.
 * @return Nothing.
 */
void watergun::pwm_stepper::set_velocity ( double velocity )
{
    /* If the velocity is 0, disable the motor */
    if ( velocity == 0. )
    {
        /* Set the sleep pin to on */
        if ( sleep_gpio.isValid () ) sleep_gpio.write ( 1 );

        /* Disable all microstepping pins */
        if ( microstep_gpio_0.isValid () ) microstep_gpio_0.write ( 0 );
        if ( microstep_gpio_1.isValid () ) microstep_gpio_1.write ( 0 );
        if ( microstep_gpio_2.isValid () ) microstep_gpio_2.write ( 0 );

        /* Set the direction pin to off */
        dir_gpio.write ( 0 );

        /* Disable the PWM pin and return */
        step_pwm.enable ( false );
        return;
    }

    /* Else begin calculations for the motor */

    /* Get the microstep number to keep the PWM frequency over the minimum */
    int microstep_number = choose_microstep_number ( velocity );

    /* Get the microstep size */
    double microstep_size = step_size / std::exp2 ( microstep_number );

    /* Get the PWM period */
    double pwm_period = microstep_size / std::abs ( velocity ); 

    /* Set the sleep pin to off */
    if ( sleep_gpio.isValid () ) sleep_gpio.write ( 0 );

    /* Set the microstep pin values */
    if ( microstep_gpio_0.isValid () ) microstep_gpio_0.write ( microstep_number & 1 );
    if ( microstep_gpio_1.isValid () ) microstep_gpio_1.write ( microstep_number & 2 );
    if ( microstep_gpio_2.isValid () ) microstep_gpio_2.write ( microstep_number & 4 );

    /* Set the direction pin */
    dir_gpio.write ( velocity < 0. );

    /* Set the PWM pin */
    step_pwm.period ( pwm_period );
    step_pwm.enable ( true );
}



/* GPIO_STEPPER IMPLEMENTATION */



/** @name constructor
 * 
 * @brief Set the motor stepping angle, controlling GPIO pins and min PWM frequency.
 * @param _step_size: The number of radians per whole step of the motor.
 * @param _min_step_freq: The minimum PWM frequency before microstepping is increased.
 * @param _step_pin: The pin number for the step control.
 * @param _dir_pin: The pin number for direction control.
 * @param _microstep_pin_0: The first pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _microstep_pin_1: The second pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _microstep_pin_2: The third pin for microstepping control, or -1 for always off, or -2 for always on.
 * @param _sleep_pin: The pin number for motor sleep control, or -1 for not present.
 * @param _position_pin: The pin number which provides stepper positioning capabilities, or -1 for not present.
 */
watergun::gpio_stepper::gpio_stepper ( const double _step_size, const double _min_step_freq, const int _step_pin, const int _dir_pin, const int _microstep_pin_0, const int _microstep_pin_1, const int _microstep_pin_2, const int _sleep_pin, const int _position_pin ) try
    : stepper_base { _step_size, _min_step_freq, _step_pin, _dir_pin, _microstep_pin_0, _microstep_pin_1, _microstep_pin_2, _sleep_pin }
{
    /* Initialize the step and position GPIOs */
    step_gpio.open ( step_pin );    
    position_gpio.open ( position_pin );
} catch ( const std::exception& e )
{
    /* Rethrow, stating that stepper motor setup failed */
    throw watergun_exception { std::string { "Stepper motor setup failed: " } + e.what () };
}



/** @name destructor
 * 
 * @brief Close and join the stepper thread.
 */
watergun::gpio_stepper::~gpio_stepper ()
{
    /* Aquire lock */
    std::unique_lock<std::mutex> lock { stepper_mx };

    /* Signal the thread to end, notify and unlock */
    end_thread = true; stepper_cv.notify_all (); lock.unlock ();

    /* Join the thread */
    if ( stepper_thread.joinable () ) stepper_thread.join ();
}



/** @name  set_position
 * 
 * @brief  Set a desired position for the stepper, and a duration over which the transition to that position will be made.
 *         Note that if the transition period is deemed too small, it will be increased appropriately.
 * @param  angle: The desired finishing angle.
 * @param  duration: The duration of the transition.
 * @return Nothing.
 */
void watergun::gpio_stepper::set_position ( const double angle, const clock::duration duration )
{
    /* If duration is negative, throw */
    if ( duration.count () < 0 ) throw watergun_exception { "GPIO stepper transition duration cannot be negative" };

    /* Aquire lock */
    std::unique_lock<std::mutex> lock { stepper_mx };

    /* Modify variables */
    target_angle = angle;
    target_transition_time = duration;

    /* Notify and return */
    stepper_cv.notify_all ();
}



/** @name  stepper_thread_function
 * 
 * @brief  The function which the stepper thread runs to control the motor movement.
 * @return Nothing.
 */
void watergun::gpio_stepper::stepper_thread_function ()
{
    /* Create a lock on the mutex */
    std::unique_lock<std::mutex> lock { stepper_mx };

    /* Put the motor into sleep mode */
    sleep_gpio.write ( 1 );

    /* Wait for a new angle */
    stepper_cv.wait ( lock );

    /* Loop while should not be ending the thread */
    while ( !end_thread )
    {
        /* Calculate the required velocity */
        double velocity = watergun::clamp ( rate_of_change ( target_angle - current_angle, target_transition_time ), -max_motor_velocity, +max_motor_velocity );

        /* Get the microstepping number */
        int microstep_number = choose_microstep_number ( velocity );

        /* Get the microstep size */
        double microstep_size = step_size / std::exp2 ( microstep_number );

        /* Get the period */
        double period = std::max ( microstep_size / std::abs ( velocity ), min_step_period );

        /* Get the number of steps required */
        int required_steps = std::abs ( target_angle - current_angle ) / microstep_size;
    
        /* Set the sleep pin to off */
        if ( sleep_gpio.isValid () ) sleep_gpio.write ( 0 );

        /* Set the microstep pin values */
        if ( microstep_gpio_0.isValid () ) microstep_gpio_0.write ( microstep_number & 1 );
        if ( microstep_gpio_1.isValid () ) microstep_gpio_1.write ( microstep_number & 2 );
        if ( microstep_gpio_2.isValid () ) microstep_gpio_2.write ( microstep_number & 4 );

        /* Set the direction pin */
        dir_gpio.write ( velocity < 0. );

        /* Loop until all required steps are made, or the condition variable is notified */
        do {
            /* Turn on the step GPIO, sleep for half the minimum period, then turn it back on */
            step_gpio.write ( 1 );
            std::this_thread::sleep_for ( std::chrono::duration<double> { min_step_period / 2. } );
            step_gpio.write ( 0 );

            /* Change the current angle */
            if ( velocity > 0. ) current_angle += microstep_size; else current_angle -= microstep_size;

            /* Decrement required steps, and break if complete */
            if ( --required_steps == 0 ) break;

            /* Sleep for the rest of the period, but break if interrupted by the condition variable */
        } while ( stepper_cv.wait_for ( lock, std::chrono::duration<double> { period - min_step_period / 2. } ) == std::cv_status::timeout );

        /* Wait on condition variable if completed all required steps */
        if ( required_steps == 0 ) stepper_cv.wait ( lock );
    }
}




