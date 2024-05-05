#ifndef UTILS_H
#define UTILS_H

/**
 * @file utils.h
 * @brief Utils fuction to node initialization
 *
 * Authors: Francisco Affonso Pinto
 *          
 */

/* Bibs */
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include "ilqr/dynamics.h"
#include "ilqr/cost.h"

// ANSI color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

bool init_dynamics(Dynamics& dynamic, ros::NodeHandle& nh);
bool init_cost(Cost& cost, ros::NodeHandle& nh);


#endif // UTILS_H