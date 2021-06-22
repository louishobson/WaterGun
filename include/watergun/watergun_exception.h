/*
 * Copyright (C) 2021 Louis Hobson <louis-hobson@hotmail.co.uk>. All Rights Reserved.
 * 
 * Distributed under MIT licence as a part of a self aiming watergun project.
 * For details, see: https://github.com/louishobson/WaterGun/blob/master/LICENSE
 * 
 * include/watergun/watergun_exception.h
 * 
 * Defines exceptions for errors in watergun processing.
 * 
 */



/* HEADER GUARD */
#ifndef WATERGUN_EXCEPTION_H_INCLUDED
#define WATERGUN_EXCEPTION_H_INCLUDED



/* INCLUDES */
#include <stdexcept>



/* DECLARATIONS */

namespace watergun
{
    /** class watergun_exception
     * 
     * General excpetion thrown on watergun errors.
     */
    class watergun_exception : public std::runtime_error { using std::runtime_error::runtime_error; };
}



/* HEADER GUARD */
#endif /* #ifndef WATERGUN_EXCEPTION_H_INCLUDED */