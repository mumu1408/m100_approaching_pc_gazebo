#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>

#include <tf/LinearMath/Quaternion.h>

geometry_msgs::Pose tag_pose;
geometry_msgs::Twist tag_twist;

std::string tag_name = "Tag6";
std::string tag_reference_frame = "world";

ros::Subscriber tag_twist_subscriber;

ros::ServiceClient tag_state_client;

gazebo_msgs::ModelState tag_model_state;
gazebo_msgs::SetModelState set_tag_state;

tf::Quaternion tag_q;
tf::Quaternion tag_next_q;

void tagTwistCallback(const geometry_msgs::Twist& twist)
{
    tag_q.setRPY(twist.angular.x, twist.angular.y, twist.angular.z);
    tag_next_q *= tag_q;

    tag_pose.orientation.w = tag_next_q.w();
    tag_pose.orientation.x = tag_next_q.x();
    tag_pose.orientation.y = tag_next_q.y();
    tag_pose.orientation.z = tag_next_q.z();

    tag_pose.position.x += twist.linear.x;
    tag_pose.position.y += twist.linear.y;
    tag_pose.position.z += twist.linear.z;

    tag_twist.linear.x = 0;
    tag_twist.linear.y = 0;
    tag_twist.linear.z = 0;
    tag_twist.angular.x = 0;
    tag_twist.angular.y = 0;
    tag_twist.angular.z = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_tag_move");

    ros::NodeHandle n;

    tag_twist_subscriber = n.subscribe("/cmd_vel", 1000, tagTwistCallback);
    tag_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);

    tag_pose.position.x = 2;
    tag_pose.position.y = 2;
    tag_pose.position.z = 1;

    tag_next_q = tf::Quaternion(0, 0, 0, 1);

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        ros::spinOnce();

        if(tag_state_client)
        {
            tag_model_state.model_name = tag_name;
            tag_model_state.reference_frame = tag_reference_frame;
            tag_model_state.pose = tag_pose;
            tag_model_state.twist = tag_twist;
            set_tag_state.request.model_state = tag_model_state;
            tag_state_client.call(set_tag_state);
            //ROS_INFO("updated");
        }
        else
        {
            ROS_INFO("update tag state failed.");
        }
        loop_rate.sleep();
    }
    return 0;
}

