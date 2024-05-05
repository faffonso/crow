#include "ilqr/utils.h"

bool init_dynamics(Dynamics& dynamic, ros::NodeHandle& nh) {
    float dt;
    std::string model;

    if (!nh.getParam("/controller/dynamics/dt", dt)) {
        ROS_ERROR_STREAM("Sampling time (dt) could not be read.");
        return false;
    }
    if (!nh.getParam("/controller/dynamics/model", model)) {
        ROS_ERROR_STREAM("Dynamic model (model) could not be read.");
        return false;
    }
    dynamic = Dynamics(dt, model);

    ROS_INFO_STREAM(GREEN << "Dynamic Model Object Created!" << RESET);
    return true;
}

bool init_cost(Cost& cost, ros::NodeHandle& nh) {
    int N, eps, t;
    float Qf_x, Qf_y, Qf_theta, Q_x, Q_y, Q_theta, R_v, R_omega, v_max, omega_max;

    if(!nh.getParam("/controller/cost/N", N))
    {
        ROS_ERROR_STREAM("Prediction Horizon (N) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/Qf_x", Qf_x))
    {
        ROS_ERROR_STREAM("Final state x cost (Qf_x) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/Qf_y", Qf_y))
    {
        ROS_ERROR_STREAM("Final state y cost (Qf_y) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/Qf_theta", Qf_theta))
    {
        ROS_ERROR_STREAM("Final state theta cost (Qf_theta) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/Q_x", Q_x))
    {
        ROS_ERROR_STREAM("State x cost (Q_x) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/Q_y", Q_y))
    {
        ROS_ERROR_STREAM("State y cost (Q_y) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/Q_theta", Q_theta))
    {
        ROS_ERROR_STREAM("State theta cost (Q_theta) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/R_v", R_v))
    {
        ROS_ERROR_STREAM("Linear velocity cost (R_v) could not be read.");
        return 0;
    }
    if(!nh.getParam("/controller/cost/R_omega", R_omega))
    {
        ROS_ERROR_STREAM("Angular velocity cost (R_omega) could not be read.");
        return 0;
    }

    if(!nh.getParam("/controller/cost/v_max", v_max))
    {
        ROS_ERROR_STREAM("Max linear speed (v_max) could not be read.");
        return 0;
    }

        if(!nh.getParam("/controller/cost/omega_max", omega_max))
    {
        ROS_ERROR_STREAM("Max angular spped (omega_max) could not be read.");
        return 0;
    }

        if(!nh.getParam("/controller/cost/eps", eps))
    {
        ROS_ERROR_STREAM("Barrier Function eps could not be read.");
        return 0;
    }

        if(!nh.getParam("/controller/cost/t", t))
    {
        ROS_ERROR_STREAM("Barrier Function t could not be read.");
        return 0;
    }
    cost = Cost(N, Qf_x, Qf_y, Qf_theta, Q_x, Q_y, Q_theta, R_v, R_omega, v_max, omega_max, eps, t);
    
    ROS_INFO_STREAM(GREEN << "Cost Object Created!" << RESET);
    return true;
}
