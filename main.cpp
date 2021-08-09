/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * main.cpp
 * 
 * Main entry code for the watergun.
 */



/* INCLUDES */
#include <signal.h>
#include <iostream>
#include <watergun/controller.h>



/* SIGNAL HANDLING */



/** @name  wait_for_interrupt
 * 
 * @brief  Block the thread until an interrupt signal is received.
 * @return Nothing.
 */
void wait_for_interrupt ()
{
    /* Create the sigset */
    sigset_t interrupt_set;
    sigemptyset ( &interrupt_set );
    sigaddset ( &interrupt_set, SIGINT );

    /* Block the signal set */
    sigprocmask ( SIG_BLOCK, &interrupt_set, NULL );

    /* Wait for it to become pending */
    int signo; sigwait ( &interrupt_set, &signo );

    /* Unlock the signal set */
    sigprocmask ( SIG_UNBLOCK, &interrupt_set, NULL );
}



int main ()
{
    /* Set up the stepper motors */
    watergun::pwm_stepper yaw_stepper { 1.8, 1000, 1, 2, 3, 4, 5, 6 };
    watergun::gpio_stepper pitch_stepper { 0.9, 1000, 3 * 2 * M_PI, 1, 2, 3, 4, 5, 6, 7 };

    /* Set up the solenoid valve */
    watergun::solenoid solenoid_valve { 1 };

    /* Create the controller in a new block */
    {
        /* Create the controller */
        watergun::controller controller { yaw_stepper, pitch_stepper, solenoid_valve, M_PI / 2., M_PI / 4., 10., 0., M_PI };

        /* Wait for interrupt signal */
        wait_for_interrupt ();
    }
}