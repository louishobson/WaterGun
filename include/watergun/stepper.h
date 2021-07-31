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
#ifndef WATERGUN_STEPPER_H_INCLUDED
#define WATERGUN_STEPPER_H_INCLUDED



/* INCLUDES */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <list>
#include <mraa/gpio.hpp>
#include <mraa/pwm.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <watergun/utility.h>
#include <watergun/watergun_exception.h>



/* DECLARATIONS */

namespace watergun
{
    /** class stepper_base
     * 
     * Stepper motor controller base class.
     */
    class stepper_base;

    /** class pwm_stepper
     * 
     * Stepper motor controller, where the step pin is controlled by PWM.
     */
    class pwm_stepper;

    /** class gpio_stepper
     * 
     * Stepper motor controller, where the step pin is controlled by GPIO.
     */
    class gpio_stepper;
}



/* STEPPER_BASE DEFINITION */



/** class stepper_base
 * 
 * Stepper motor controller base class.
 */
class watergun::stepper_base
{
public:

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
    stepper_base ( double _step_size, double _min_step_freq, int _step_pin, int _dir_pin, int _microstep_pin_0, int _microstep_pin_1, int _microstep_pin_2, int _sleep_pin );

    /** @name pure virtual destructor
     * 
     * @brief A stepper_base class should be abstract.
     */
    virtual ~stepper_base () = 0;



    /* The clock the stepper uses */
    typedef std::chrono::steady_clock clock;



protected:

    /* The number of radians per whole step of the motor */
    double step_size;

    /* The minimum PWM frequency before microstepping is increased */
    double min_step_freq;

    /* Pin numbers */
    int step_pin, dir_pin, microstep_pin_0, microstep_pin_1, microstep_pin_2, sleep_pin;

    /* The availible microstepping numbers (0 for full step, 1 for 1/2, etc.) */
    std::list<int> availible_microstep_numbers { 0, 1, 2, 3, 4, 5 };

    /* The Gpio objects, not including the step pin */
    mraa::Gpio dir_gpio, microstep_gpio_0, microstep_gpio_1, microstep_gpio_2, sleep_gpio;



    /** @name  choose_microstep_number
     * 
     * @brief  Choose the best microstep number for a given angular velocity.
     * @param  velocity: The angular velocity to choose based on.
     * @return The microstep number (one of availible_microstep_numbers).
     */
    int choose_microstep_number ( double velocity ) const;



    /** @name  disable_motor
     * 
     * @brief  Put the motor to sleep and turn off all direction and microstepping and direction pins.
     * @return Nothing.
     */
    void disable_motor ();

    /** @name  enable_motor
     * 
     * @brief  Take the motor out of sleep, and set up the microstepping and direction pins.
     * @param  microstep_number: The microstep number to use.
     * @param  direction: True for clockwise, false for anticlockwise.
     * @return Nothing.
     */
    void enable_motor ( int microstep_number, bool direction );



    /** @name  create_pwm
     * 
     * @brief  Create a PWM pin.
     * @param  pin: The pin to create the PWM object on.
     * @return The PWM object.
     */
    static mraa::Pwm create_pwm ( int pin );

    /** @name  create_output_gpio
     * 
     * @brief  Create an output GPIO pin.
     * @param  pin: The pin to create the GPIO object on.
     * @return The GPIO object.
     */
    static mraa::Gpio create_output_gpio ( int pin );

};



/* PWM_STEPPER DEFINITION */



/** class pwm_stepper
 * 
 * Stepper motor controller, where the step pin is controlled by PWM.
 */
class watergun::pwm_stepper : public stepper_base
{
public:

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
    pwm_stepper ( double _step_size, double _min_step_freq, int _step_pin, int _dir_pin, int _microstep_pin_0, int _microstep_pin_1, int _microstep_pin_2, int _sleep_pin );



    /** @name  set_velocity
     * 
     * @brief  Set a new rotation velocity.
     * @param  velocity: The new angular velocity in rad/sec, positive meaning clockwise and vice versa.
     * @return Nothing.
     */
    void set_velocity ( double velocity );



private:

    /* Step PWM pin */
    mraa::Pwm step_pwm;

};



/** GPIO_STEPPER DEFINITION */



/** class gpio_stepper
 * 
 * Stepper motor controller, where the step pin is controlled by GPIO.
 */
class watergun::gpio_stepper : public stepper_base
{
public:

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
    gpio_stepper ( double _step_size, double _min_step_freq, int _step_pin, int _dir_pin, int _microstep_pin_0, int _microstep_pin_1, int _microstep_pin_2, int _sleep_pin, int _position_pin );

    /** @name destructor
     * 
     * @brief Close and join the stepper thread.
     */
    ~gpio_stepper ();



    /** @name  set_position
     * 
     * @brief  Set a desired position for the stepper, and a duration over which the transition to that position will be made.
     *         Note that if the transition period is deemed too small, it will be increased appropriately.
     * @param  angle: The desired finishing angle.
     * @param  duration: The duration of the transition.
     * @return Nothing.
     */
    void set_position ( double angle, clock::duration duration );



private:

    /* Position pin */
    int position_pin;

    /* Step and position GPIO pins */
    mraa::Gpio step_gpio, position_gpio;



    /* The maximum motor velocity in radians per second */
    const double max_motor_velocity { 3 * 2 * M_PI };

    /* The minumum step period */
    const double min_step_period { 100e-6 };



    /* The current angle of the stepper motor */
    double current_angle { 0. };

    /* The angle the stepper should aim for */
    double target_angle { 0. };

    /* The amount of time to make the transition */
    clock::duration target_transition_time { 0 };

    /* Mutex and condition variable for protecting the stepper variables */
    std::mutex stepper_mx;
    std::condition_variable stepper_cv;

    /* Thread for controlling stepper position */
    std::thread stepper_thread;

    /* An atomic boolean telling the thread when to end */
    std::atomic_bool end_thread { false };



    /** @name  stepper_thread_function
     * 
     * @brief  The function which the stepper thread runs to control the motor movement.
     * @return Nothing.
     */
    void stepper_thread_function ();

};



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_STEPPER_H_INCLUDED */