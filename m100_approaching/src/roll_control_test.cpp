#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>

#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/GlobalPosition.h>

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/GimbalSpeedControl.h>
#include <dji_sdk/VelocityControl.h>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>
#include <iterator>
#include <string>
#include <iostream>
#include <fstream>

ros::ServiceClient gimbal_velocity_control_service;
ros::ServiceClient sdk_permission_control_service;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roll_control_test");
    ros::NodeHandle nh;

    gimbal_velocity_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("/dji_sdk/gimbal_speed_control");
    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control");

    dji_sdk::SDKPermissionControl sdk_permission_control;
    dji_sdk::GimbalSpeedControl gimbal_speed_control;
    
    sdk_permission_control.request.control_enable = 1;
    bool control_requested = false;
    while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
    {
        ROS_ERROR("Velocity controller: request control failed!");
    }

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        gimbal_speed_control.request.roll_rate = -100;// * 0.1 度每秒
        gimbal_speed_control.request.pitch_rate = 0;
        gimbal_speed_control.request.yaw_rate = 0;

        if(!(gimbal_velocity_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result))
        {
            ROS_ERROR("Velocity controller: gimbal speed control failed!");
        }
        loop_rate.sleep();
    }
}
