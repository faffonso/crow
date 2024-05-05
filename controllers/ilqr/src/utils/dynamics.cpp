#include "ilqr/dynamics.h"

Dynamics::Dynamics()
{
    _dt = 0.1;
    _model = "";
}

Dynamics::Dynamics(double dt, std::string model)
{
    // Saving classes atributes
    _dt = dt;
    _model = model;

    // State space
    SX x = SX::sym("x", _Nx);
    SX u = SX::sym("u", _Nu);
    SX ur = SX::sym("ur", _Nu);

    SX f, ns;

    SX x1_dot = u(0) * cos(x(2));
    SX x2_dot = u(0) * sin(x(2));
    SX x3_dot = u(1);

    SX x_dot = vertcat(x1_dot, x2_dot, x3_dot);

    ns = x + x_dot * dt;


    // Unicycle Kinematic Model
    if (model == "standard") 
    {

        SX x1_dot = x(0) + (u(0)) * cos(x(2)) * dt;
        SX x2_dot = x(1) + (u(0)) * sin(x(2)) *  dt;
        SX x3_dot = x(2) + u(1) * dt;
        SX x4_dot = u(0);
        SX x5_dot = u(1);

        SX x_dot = vertcat(x1_dot, x2_dot, x3_dot, x4_dot, x5_dot);
        
        f = x_dot;
    }

    // Error-Tracking (Mobile point)
    else if (model == "error-tracking") 
    {

        SX e1_dot =  u(1) * x(1) - u(0) + ur(0) * cos(x(2));
        SX e2_dot = -u(1) * x(0) + ur(0) * sin(x(2));
        SX e3_dot = -u(1) + ur(1);

        SX e_dot = vertcat(e1_dot, e2_dot, e3_dot);

        f = x + e_dot * dt;
    }

    else {
        ROS_ERROR_STREAM("Not exist this dynamic model" << model);
    }

    // Dynamics derivatives (Aproxximation for A and B --> x_dot = Ax + Bu)
    SX f_x = jacobian(f, x);
    SX f_u = jacobian(f, u);

    // Dynamics Functions
    _ns = Function("ns", {x, u}, {ns});

    _f = Function("f", {x, u, ur}, {f});
    _f_x = Function("f_x", {x, u, ur}, {f_x});
    _f_u = Function("f_u", {x, u, ur}, {f_u});
}

MatrixXd Dynamics::get_ns(std::vector<DM> input) {
    return Eigen::Matrix<double, 1, _Nx>::Map(DM::densify(_ns(input).at(0)).nonzeros().data(), 1, _Nx);
}

MatrixXd Dynamics::get_f(std::vector<DM> input) {
    return Eigen::Matrix<double, 1, _Nx>::Map(DM::densify(_f(input).at(0)).nonzeros().data(), 1, _Nx);
}

f_prime_t Dynamics::get_f_prime(std::vector<DM> input)
{
    _f_prime.f_x = Eigen::Matrix<double, _Nx, _Nx>::Map(DM::densify(_f_x(input).at(0)).nonzeros().data(), _Nx, _Nx);
    _f_prime.f_u = Eigen::Matrix<double, _Nx, _Nu>::Map(DM::densify(_f_u(input).at(0)).nonzeros().data(), _Nx, _Nu);
    
    return _f_prime;
}