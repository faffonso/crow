#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

#include "ilqr/dynamics.h"
#include "ilqr/cost.h"
#include "ilqr/ilqr.h"
#include "ilqr/utils.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ilqr_node");
    ros::NodeHandle nh;   
    
    int N;
    float dt;

    Dynamics dynamic;
    Cost cost;

    ros::Rate rate(12);

    if (!init_dynamics(dynamic, nh)) 
        return 1;

    if (!init_cost(cost, nh))
        return 1; 

    nh.getParam("/controller/dynamics/dt", dt);
    nh.getParam("/controller/cost/N", N);

    iLQR control(nh, &dynamic, &cost,
                dt, N);
    
    ROS_INFO_STREAM(GREEN << "iLQR Created!" << RESET);

    while (ros::ok()) 
    {
        control.run();

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}