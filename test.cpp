/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * test.cpp
 * 
 * Test file.
 * 
 */



/* INCLUDES */
#include <csignal>
#include <iostream>
#include <watergun/controller.h>



/* Global quit boolean */
std::atomic_bool quit = false;



/* MAIN */

int main ()
{
    /* Set quit to true when interrupted */
    std::signal ( SIGINT, [] ( int signum ) { quit = true; } );

    /* Create the tracker */
    watergun::tracker controller {};

    /* Loop until signalled to quit */
    while ( !quit )
    {
        /* Get the next info on tracked users */
        auto tracked_users = controller.wait_get_tracked_users ();

        /* Print info */
        if ( tracked_users.size () ) 
        {
            auto t = tracked_users.front ();
            //auto aim = controller.calculate_aim ( t );
            //std::cout << t.com.X * 180 / M_PI << "    " << std::atan ( t.com.Y / t.com.Z ) * 180 / M_PI << "    " << aim.yaw * 180 / M_PI << "    " << aim.pitch * 180 / M_PI << std::endl;
            std::cout << t.com.X * 180 / M_PI << "    " << t.com_rate.X * 180 / M_PI << std::endl;
            std::cout << t.com.Y << "    " << t.com_rate.Y << std::endl;
            std::cout << t.com.Z << "    " << t.com_rate.Z << std::endl;
        }
    }

    /* Return success */
    return 0;
}