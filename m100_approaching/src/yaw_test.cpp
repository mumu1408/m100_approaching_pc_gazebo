#include <ros/ros.h>

#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/Gimbal.h>

#include <Eigen/Geometry>

static double drone_yaw;

static double gimbal_roll;
static double gimbal_pitch;
static double gimbal_yaw;

static double gimbal_yaw_wrt_drone;

static void drone_quaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr& drone_quaternion_msg)
{
    Eigen::Vector3d my_euler;
	Eigen::Matrix3d my_rotation;
	Eigen::Quaternion<double> quaternion = Eigen::Quaternion<double>(drone_quaternion_msg->q0, drone_quaternion_msg->q1, drone_quaternion_msg->q2, drone_quaternion_msg->q3);
 
 	my_euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
    //std::cout << "around x axis = " << my_euler[2] / M_PI * 180 << std::endl;
    //std::cout << "around y axis = " << my_euler[1] / M_PI * 180 << std::endl;
    //std::cout << "around z axis = " << my_euler[0] / M_PI * 180 << std::endl;
    drone_yaw = atan2(2*(quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()), 1 - 2 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));
    drone_yaw = drone_yaw / 3.1415926 * 180;
    std::cout << "drone_yaw: " << drone_yaw << std::endl;
}

static void gimbal_callback(const dji_sdk::Gimbal::ConstPtr& gimbal_ori_msg)
{
    gimbal_roll = gimbal_ori_msg->roll;
    gimbal_yaw = gimbal_ori_msg->yaw;
    gimbal_pitch = gimbal_ori_msg->pitch;
    std::cout << "gimbal_yaw: " << gimbal_yaw << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yaw_test");
    ros::NodeHandle nh;

    ros::Subscriber attitude_quaternion_sub;
    ros::Subscriber gimbal_sub;
    attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion", 5, drone_quaternion_callback);
    gimbal_sub = nh.subscribe("/dji_sdk/gimbal", 5, gimbal_callback);

    ros::Rate loop_rate(10);
    ros::spinOnce();


    while (ros::ok())
    {
        ros::spinOnce(); 
        if (drone_yaw < 0)
            drone_yaw = 360 + drone_yaw;
        std::cout << "drone_yaw modified: " << drone_yaw << std::endl;
        if (gimbal_yaw < 0)
            gimbal_yaw = 360 + gimbal_yaw;
        std::cout << "gimbal_yaw modified: " << gimbal_yaw << std::endl;
        gimbal_yaw_wrt_drone = gimbal_yaw - drone_yaw;
        std::cout << "gimbal_yaw - drone_yaw: " << gimbal_yaw_wrt_drone << std::endl;
        if (gimbal_yaw_wrt_drone <= 360 && gimbal_yaw_wrt_drone > 180)
        {
            gimbal_yaw_wrt_drone = -(360 - gimbal_yaw_wrt_drone);
            std::cout << "180.0 < gimbal_yaw_wrt_drone <= 360.0" << std::endl;
        }

        else if (gimbal_yaw_wrt_drone <= -180 && gimbal_yaw_wrt_drone >= -360)
        {
            gimbal_yaw_wrt_drone = 360 + gimbal_yaw_wrt_drone;
            std::cout << "-360.0 <= gimbal_yaw_wrt_drone <= -180.0" << std::endl;
        }
        std::cout << "gimbal_yaw_wrt_drone: " << gimbal_yaw_wrt_drone << std::endl;
        loop_rate.sleep();
    }

}
