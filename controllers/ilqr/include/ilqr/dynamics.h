#ifndef DYNAMICS_H
#define DYNAMICS_H

/**
 * @file dynamics.h
 * @brief Implementation of mobile robot dynamics
 *
 * This file contains the implementation of 4WD mobile robot
 * dynamics, including discretization and dynamics derivatives.
 *
 * Authors: Francisco Affonso Pinto
 *          
 */

/* Bibs */
#include <iostream> 
#include <string>

#include <ros/console.h>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace casadi;
using namespace Eigen;

typedef struct f_prime {
    MatrixXd f_x; // df/dx
    MatrixXd f_u; // df/du
} f_prime_t;

class Dynamics
{
    protected:
        float _dt;              // Sampling time [s]     
        std::string _model;     // Dynamic model (standard or error-tracking)
        f_prime_t _f_prime;     // Contains dynamic jacobians
        
        Function _f, _f_x, _f_u, _ns;    // CasADi function

        static constexpr int _Nx=3, // State vector size
                             _Nu=2; // Action control vector size

    public:
        /**
         * @brief Construct a new Dynamics object
         * 
         */
        Dynamics();

        /**
         * @brief Construct a new Dynamics object
         * 
         * @param dt Sampling time [s]
         * @param model Dynamic model (standard or error-tracking)
         */
        Dynamics(double dt, std::string model);

        /**
         * @brief Get the next state using kinmematic unicycle model
         * 
         * @param input State and Action control {x, u}
         * @return MatrixXd 
         */
        MatrixXd get_ns(std::vector<DM> input);

        /**
         * @brief Get the next step of error-tracking model
         * 
         * @param input State and Action control {x, u, ur}
         * @return std::vector<casadi::DM> 
         */        
        MatrixXd get_f(std::vector<DM> input);

        /**
         * @brief Get the f prime object (dynamics derivatives)
         * 
         * @param input State and Action control {x, u}
         * @return f_prime_t 
         */
        f_prime_t  get_f_prime(std::vector<DM> input);
};

#endif // DYNAMICS_H