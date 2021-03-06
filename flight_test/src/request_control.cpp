#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

#include <dji_sdk/SDKPermissionControl.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "request_control");
    ros::NodeHandle nh;

    ros::ServiceClient sdk_permission_control_service;
    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control");
    dji_sdk::SDKPermissionControl sdk_permission_control;
    sdk_permission_control.request.control_enable = 1;
    bool control_requested = false;
    while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
    {
        ROS_ERROR("request control failed!");
    }
}
