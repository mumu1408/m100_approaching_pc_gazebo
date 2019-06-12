#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>

#include <dji_sdk/Gimbal.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/GlobalPosition.h>

#include <apriltags/AprilTagDetections.h>

#include <cmath>
#include <Eigen/Geometry>

#include <vector>
#include <iterator>
#include <algorithm>
#include <fstream>

ros::Subscriber apriltags_36h11_sub;

ros::Publisher setpoint_target_x_incameraframe_pub;
ros::Publisher setpoint_target_y_incameraframe_pub;
ros::Publisher setpoint_target_z_incameraframe_pub;
ros::Publisher setpoint_target_roll_incameraframe_pub;
ros::Publisher setpoint_target_pitch_incameraframe_pub;
ros::Publisher setpoint_target_yaw_incameraframe_pub;

bool found_36h11 = false;
std::string tag_36h11_detection_topic;
Eigen::Matrix4d tag_to_camera_transformation;
Eigen::Vector3d target_position_incameraframe(0, 0, 0);
Eigen::Vector3d tag_euler_incameraframe(0, 0, 0);
Eigen::Vector3d target_euler_incameraframe(0, 0, 0);

std_msgs::Float64 target_x_incameraframe_msg;
std_msgs::Float64 target_y_incameraframe_msg;
std_msgs::Float64 target_z_incameraframe_msg;
std_msgs::Float64 target_roll_incameraframe_msg;
std_msgs::Float64 target_pitch_incameraframe_msg;
std_msgs::Float64 target_yaw_incameraframe_msg;

std::ofstream fout;

void apriltags36h11Callback(const apriltags::AprilTagDetections::ConstPtr& apriltag_pos_msg)
{
    if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
    {
        found_36h11 = false;
        return;
    }
    else
    {
	for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++it)
	{
        //if ((*it).id != 6) found_36h11 = false;
	    if((*it).id == 6)
	    {
		found_36h11 = true;
		//tag_incameraframe是apriltags/detection的类型指针，该类型包括header,id,corners2d,tag_size,pose
		auto tag_incameraframe = it;
		geometry_msgs::Pose tag_pose_incameraframe = tag_incameraframe->pose;
		//tag_incameraframe->pose，类型为geometry_msgs/Pose，包括位置和四元数姿态，将其转换为标签相对于相机的4×4转换矩阵
		Eigen::Vector3d tag_position_incameraframe = Eigen::Vector3d(tag_pose_incameraframe.position.x, tag_pose_incameraframe.position.y, tag_pose_incameraframe.position.z);
		Eigen::Quaternion<double> tag_quaternion_incameraframe = Eigen::Quaternion<double>(tag_pose_incameraframe.orientation.w, tag_pose_incameraframe.orientation.x, tag_pose_incameraframe.orientation.y, tag_pose_incameraframe.orientation.z);
		std::cout << "tag_orientation: " << tag_pose_incameraframe.orientation.w << " " << tag_pose_incameraframe.orientation.x << " " << tag_pose_incameraframe.orientation.y << " " << tag_pose_incameraframe.orientation.z << '\n';
		tag_to_camera_transformation.block(0,0,3,3) = tag_quaternion_incameraframe.toRotationMatrix();
		tag_to_camera_transformation.block(0,3,3,1) = tag_position_incameraframe;
		//最终逼近点相对于相机坐标系的位置
		Eigen::Vector4d position = tag_to_camera_transformation * Eigen::Vector4d(0,0,1,1);
		target_position_incameraframe = position.head(3);
		std::cout << "target_x_incameraframe = " << target_position_incameraframe(0) << "m" << std::endl;
		std::cout << "target_y_incameraframe = " << target_position_incameraframe(1) << "m" << std::endl;
		std::cout << "target_z_incameraframe = " << target_position_incameraframe(2) << "m" << std::endl;
		fout << "target_x_incameraframe = " << target_position_incameraframe(0) << "m" << std::endl;
		fout << "target_y_incameraframe = " << target_position_incameraframe(1) << "m" << std::endl;
		fout << "target_z_incameraframe = " << target_position_incameraframe(2) << "m" << std::endl;
		//标签坐标系相对于相机坐标系的欧拉角
		tag_euler_incameraframe = tag_quaternion_incameraframe.toRotationMatrix().eulerAngles(2, 1, 0);
		//std::cout << "tag_euler_incameraframe around x axis = " << tag_euler_incameraframe[2] / M_PI * 180 << std::endl;
		//std::cout << "tag_euler_incameraframe around y axis = " << tag_euler_incameraframe[1] / M_PI * 180 << std::endl;
		//std::cout << "tag_euler_incameraframe around z axis = " << tag_euler_incameraframe[0] / M_PI * 180 << std::endl;
		//最终逼近点坐标系相对于目前相机坐标系的欧拉角
		//标签坐标系围绕其X轴转180度即为最终逼近点坐标系的姿态
		Eigen::AngleAxisd tag_to_target_angleaxis = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
		Eigen::Matrix3d camera_to_target_rotation = tag_quaternion_incameraframe.toRotationMatrix() * tag_to_target_angleaxis.matrix();
		target_euler_incameraframe = camera_to_target_rotation.eulerAngles(2, 1, 0); 
		//std::cout << "camera_to_tag_rotation =\n" << tag_quaternion_incameraframe.toRotationMatrix() << std::endl;
		//std::cout << "camera_to_target_rotation =\n" << camera_to_target_rotation << std::endl;
		std::cout << "target_euler_incameraframe around x axis = " << target_euler_incameraframe[2] / M_PI * 180 << "degree" << std::endl;
		std::cout << "target_euler_incameraframe around y axis = " << target_euler_incameraframe[1] / M_PI * 180 << "degree" << std::endl;
		std::cout << "target_euler_incameraframe around z axis = " << target_euler_incameraframe[0] / M_PI * 180 << "degree" << std::endl;
		fout << "target_euler_incameraframe around x axis = " << target_euler_incameraframe[2] / M_PI * 180 << "degree" << std::endl;
		fout << "target_euler_incameraframe around y axis = " << target_euler_incameraframe[1] / M_PI * 180 << "degree" << std::endl;
		fout << "target_euler_incameraframe around z axis = " << target_euler_incameraframe[0] / M_PI * 180 << "degree" << std::endl;
        
        Eigen::Quaterniond q = Eigen::Quaterniond( camera_to_target_rotation );
        static double x_angle_old1 = 0, x_angle_old2 = 0;
        static double y_angle_old1 = 0, y_angle_old2 = 0;
        static double z_angle_old1 = 0, z_angle_old2 = 0;
        double zero_const = 0.001;
        if (!(((fabs(q.w()-q.y())<zero_const) && (fabs(q.x()+q.z())<zero_const))||((fabs(q.x()-q.z())<zero_const) && (fabs(q.w()+q.y())<zero_const))))        
        {
            double x_angle_1 = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y())); //弧度
            double y_angle_1 = atan2(-(-2 * q.w() * q.y() + 2 * q.x() * q.z()), (2 * (q.w() * q.x() + q.y() * q.z())) / sin(x_angle_1));
            double z_angle_1 = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

            double x_angle_2 = atan2(-2 * (q.w() * q.x() + q.y() * q.z()), -(1 - 2 * (q.x() * q.x() + q.y() * q.y()))); //弧度
            double y_angle_2 = atan2(-(-2 * q.w() * q.y() + 2 * q.x() * q.z()), (2 * (q.w() * q.x() + q.y() * q.z())) / sin(x_angle_2));
            double z_angle_2 = atan2(-2 * (q.w() * q.z() + q.x() * q.y()), -(1 - 2 * (q.y() * q.y() + q.z() * q.z())));

            double delta1 = fabs(x_angle_1) + fabs(y_angle_1) + fabs(z_angle_1);
            double delta2 = fabs(x_angle_2) + fabs(y_angle_2) + fabs(z_angle_2);

            std::cout << "x_angle_1 = " << x_angle_1 / M_PI * 180 << "degree" << std::endl;
            std::cout << "y_angle_1 = " << y_angle_1 / M_PI * 180 << "degree" << std::endl;
            std::cout << "z_angle_1 = " << z_angle_1 / M_PI * 180 << "degree" << std::endl;

            std::cout << "x_angle_2 = " << x_angle_2 / M_PI * 180 << "degree" << std::endl;
            std::cout << "y_angle_2 = " << y_angle_2 / M_PI * 180 << "degree" << std::endl;
            std::cout << "z_angle_2 = " << z_angle_2 / M_PI * 180 << "degree" << std::endl;

            if (delta1 <= delta2)
            {
                target_euler_incameraframe[2] = x_angle_1;
                target_euler_incameraframe[1] = y_angle_1;
                target_euler_incameraframe[0] = z_angle_1;
                std::cout << "delta1 <= delta2" << std::endl;
                fout << "delta1 <= delta2" << std::endl;
            }
            else
            {
                target_euler_incameraframe[2] = x_angle_2;
                target_euler_incameraframe[1] = y_angle_2;
                target_euler_incameraframe[0] = z_angle_2;
                std::cout << "delta1 > delta2" << std::endl;
                fout << "delta1 > delta2" << std::endl;
            }
        }
        else
        {
            if ((fabs(q.w()-q.y())<zero_const) && (fabs(q.x()+q.z())<zero_const))
            {
                target_euler_incameraframe[1] = 0.5 * M_PI;
                double x_angle_min_z_angle = 2 * atan2(q.x(), q.w());
                target_euler_incameraframe[2] = x_angle_old1;
                target_euler_incameraframe[0] = x_angle_old1 - x_angle_min_z_angle;
                std::cout << "y_angle = 0.5 * M_PI" << std::endl;
                fout << "y_angle = 0.5 * M_PI" << std::endl;
            }
            else
            {
                target_euler_incameraframe[1] = -0.5 * M_PI;
                double x_angle_add_z_angle = 2 * atan2(q.x(), q.w());
                target_euler_incameraframe[2] = x_angle_old1;
                target_euler_incameraframe[0] = -x_angle_old1 + x_angle_add_z_angle;
                std::cout << "y_angle = -0.5 * M_PI" << std::endl;
                fout << "y_angle = -0.5 * M_PI" << std::endl;
            }
        } 
        std::cout << "modified x_angle " << target_euler_incameraframe[2] / M_PI * 180 << "degree" << std::endl;
		std::cout << "modified y_angle " << target_euler_incameraframe[1] / M_PI * 180 << "degree" << std::endl;
		std::cout << "modified z_angle " << target_euler_incameraframe[0] / M_PI * 180 << "degree" << std::endl;

        fout << "modified x_angle " << target_euler_incameraframe[2] / M_PI * 180 << "degree" << std::endl;
		fout << "modified y_angle " << target_euler_incameraframe[1] / M_PI * 180 << "degree" << std::endl;
		fout << "modified z_angle " << target_euler_incameraframe[0] / M_PI * 180 << "degree" << std::endl;

        

        target_euler_incameraframe[2] = (target_euler_incameraframe[2] + x_angle_old1 + x_angle_old2) / 3.0;
        target_euler_incameraframe[1] = (target_euler_incameraframe[1] + y_angle_old1 + y_angle_old2) / 3.0;
        target_euler_incameraframe[0] = (target_euler_incameraframe[0] + z_angle_old1 + z_angle_old2) / 3.0;
        std::cout << "average value of modified x_angle " << target_euler_incameraframe[2] / M_PI * 180 << "degree" << std::endl;
		std::cout << "average value of modified y_angle " << target_euler_incameraframe[1] / M_PI * 180 << "degree" << std::endl;
		std::cout << "average value of modified z_angle " << target_euler_incameraframe[0] / M_PI * 180 << "degree" << std::endl;

        fout << "average value of modified x_angle " << target_euler_incameraframe[2] / M_PI * 180 << "degree" << std::endl;
		fout << "average value of modified y_angle " << target_euler_incameraframe[1] / M_PI * 180 << "degree" << std::endl;
		fout << "average value of modified z_angle " << target_euler_incameraframe[0] / M_PI * 180 << "degree" << std::endl;


        x_angle_old2 = x_angle_old1;
        y_angle_old2 = y_angle_old1;
        z_angle_old2 = z_angle_old1;
        x_angle_old1 = target_euler_incameraframe[2];
        y_angle_old1 = target_euler_incameraframe[1];
        z_angle_old1 = target_euler_incameraframe[0];


        /*if (fabs(target_euler_incameraframe[0] / M_PI * 180)>169 && fabs(target_euler_incameraframe[1] / M_PI * 180)>169)
        {
            target_euler_incameraframe[2] = roll;
            std::cout << "roll modified" << std::endl;
            fout << "roll modified: " << roll / M_PI * 180 << "degree" << std::endl; 
        }*/

		if (fabs(target_euler_incameraframe[2]) + fabs(target_euler_incameraframe[1]) + fabs(target_euler_incameraframe[0]) >=9)
		{
		    ROS_INFO("出现了欧拉角为180度的情况,将[180 180 180]更改为[0 0 0]");
		    target_euler_incameraframe[2] = 0;
		    target_euler_incameraframe[1] = 0;
		    target_euler_incameraframe[0] = 0;
		}
	    }
	}
        
    }
}

void print_parameters()
{
    ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "m100_approaching_setpoint");
    ros::NodeHandle nh;
    ros::NodeHandle node_priv("~");
    fout.open("/home/mumu1408/m100_ws/src/m100_approaching/log.txt", std::ios::app);
    double secs = ros::Time::now().toSec();
    fout << "time: " << secs << std::endl;

    node_priv.param<std::string>("tag_36h11_detection_topic", tag_36h11_detection_topic, "/apriltags/detections");
    print_parameters();

    apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 5, apriltags36h11Callback);

    setpoint_target_x_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_x_incameraframe", 10);
    setpoint_target_y_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_y_incameraframe", 10);
    setpoint_target_z_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_z_incameraframe", 10);
    setpoint_target_roll_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_roll_incameraframe", 10);
    setpoint_target_pitch_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_pitch_incameraframe", 10);
    setpoint_target_yaw_incameraframe_pub = nh.advertise<std_msgs::Float64>("m100_approaching/setpoint_target_yaw_incameraframe", 10);

    tag_to_camera_transformation << 0, 0, 0, 0,
                                    0, 0, 0, 0,
                                    0, 0, 0, 0,
                                    0, 0, 0, 1;

    //ros::Rate loop_rate(200);
    ros::Rate loop_rate(100);

    ros::spinOnce();

    while (ros::ok())
    {
        ros::spinOnce(); 
        if(found_36h11)
        {
            target_x_incameraframe_msg.data = target_position_incameraframe(0);
            target_y_incameraframe_msg.data = target_position_incameraframe(1);
            target_z_incameraframe_msg.data = target_position_incameraframe(2);
            target_roll_incameraframe_msg.data = target_euler_incameraframe(2) / M_PI * 180;
            target_pitch_incameraframe_msg.data = target_euler_incameraframe(1) / M_PI * 180;
            target_yaw_incameraframe_msg.data = target_euler_incameraframe(0) / M_PI * 180;
        }
        else
        {
            target_x_incameraframe_msg.data = 0;
            target_y_incameraframe_msg.data = 0;
            target_z_incameraframe_msg.data = 0;
            //target_roll_incameraframe_msg.data = target_euler_incameraframe(2);
            //target_pitch_incameraframe_msg.data = target_euler_incameraframe(1);
            //target_yaw_incameraframe_msg.data = target_euler_incameraframe(0);
	        target_roll_incameraframe_msg.data = 0;
            target_pitch_incameraframe_msg.data = 0;
            target_yaw_incameraframe_msg.data = 0;
        }
        setpoint_target_x_incameraframe_pub.publish(target_x_incameraframe_msg);
        setpoint_target_y_incameraframe_pub.publish(target_y_incameraframe_msg);
        setpoint_target_z_incameraframe_pub.publish(target_z_incameraframe_msg);
        setpoint_target_roll_incameraframe_pub.publish(target_roll_incameraframe_msg);
        setpoint_target_pitch_incameraframe_pub.publish(target_pitch_incameraframe_msg);
        setpoint_target_yaw_incameraframe_pub.publish(target_yaw_incameraframe_msg);

        //std::cout << "pid target published." << std::endl;

        loop_rate.sleep();
    }
    fout.close();
}
