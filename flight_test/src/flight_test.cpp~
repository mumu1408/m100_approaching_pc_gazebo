#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

#include <dji_sdk/SDKPermissionControl.h>
#include <dji_sdk/VelocityControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/dji_drone.h>

double f_speed; 
int f_time;

ros::ServiceClient sdk_permission_control_service;
ros::ServiceClient drone_velocity_control_service;
ros::ServiceClient drone_task_service;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flight_test");
    ros::NodeHandle nh;
    ros::NodeHandle node_priv("~");
    node_priv.param("f_speed", f_speed, 0.3);
    node_priv.param("f_time", f_time, 5);

    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control");
    drone_velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("/dji_sdk/velocity_control");
    drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    
    
    dji_sdk::SDKPermissionControl sdk_permission_control;
    dji_sdk::VelocityControl velocity_control;
    dji_sdk::DroneTaskControl drone_task_control;
    
    DJIDrone* drone = new DJIDrone(nh);

    sdk_permission_control.request.control_enable = 1;
    bool control_requested = false;
    while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
    {
        ROS_ERROR("request control failed!");
    }

/*
    drone_task_control.request.task = 4;
    while(!(drone_task_service.call(drone_task_control) && drone_task_control.response.result))
    {
        ROS_ERROR("takeoff failed!");
    }
*/
    drone->takeoff();
    //sleep(5);
    ros::Time begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(8))
    {
    }
    begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(f_time))
    {
	/*******************************前*************************************/
        velocity_control.request.frame = 0;//body frame(FRU)
        velocity_control.request.vx = f_speed;
        velocity_control.request.vy = 0;
        velocity_control.request.vz = 0;
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }
    }
    begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(f_time))
    {    
        /*********************************右**************************************/
	velocity_control.request.frame = 0;//body frame(FRU)
        velocity_control.request.vx = 0;
        velocity_control.request.vy = f_speed;
        velocity_control.request.vz = 0;
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }
     }
        
    begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(f_time))
    {    
	/*********************************后**************************************/
	velocity_control.request.frame = 0;//body frame(FRU)
        velocity_control.request.vx = -1 * f_speed;
        velocity_control.request.vy = 0;
        velocity_control.request.vz = 0;
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }
    }
        
    begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(f_time))
    {          
	/*********************************左**************************************/
	velocity_control.request.frame = 0;//body frame(FRU)
        velocity_control.request.vx = 0;
        velocity_control.request.vy = -1 * f_speed;
        velocity_control.request.vz = 0;
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }
    }
    
    begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(f_time))
    {   
	/*********************************上**************************************/
	velocity_control.request.frame = 0;//body frame(FRU)
        velocity_control.request.vx = 0;
        velocity_control.request.vy = 0;
        velocity_control.request.vz = f_speed;
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }
    }

    begin = ros::Time::now();
    while((ros::Time::now() - begin) < ros::Duration(f_time))
    {             
	/*********************************下**************************************/
	velocity_control.request.frame = 0;//body frame(FRU)
        velocity_control.request.vx = 0;
        velocity_control.request.vy = 0;
        velocity_control.request.vz = -1 * f_speed;
        velocity_control.request.yawRate = 0;

        if(!(drone_velocity_control_service.call(velocity_control) && velocity_control.response.result))
        {
            ROS_ERROR("Velocity controller: drone velocity control failed!");
        }
    }
     
    drone_task_control.request.task = 6;
    while(!(drone_task_service.call(drone_task_control) && drone_task_control.response.result))
    {
        ROS_ERROR("landing failed!");
    }

    sdk_permission_control.request.control_enable = 0;
    sdk_permission_control_service.call(sdk_permission_control);

}
