#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "kinova_msgs/KinovaPose.h"
#include "kinova_msgs/PoseVelocityWithFingers.h"
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_driver/kinova_comm.h"
#include "eye_control/guiMsg.h"
#include "eye_control/changeFingerState.h"

// Forward declaring functions
void updateArmPos(const kinova_msgs::KinovaPose& msg);
void guiMsgToKinova(const eye_control::guiMsg& msg);
bool changeFingerState(eye_control::changeFingerState::Request &req,
                       eye_control::changeFingerState::Response &res);

// The following are needed later for communicating directly with the kinova
// api
boost::recursive_mutex api_mutex;
bool is_first_init = true;
std::string kinova_robotType = "j2n6s300";


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "gui_to_kinova");
    ros::NodeHandle nh;

    ros::Subscriber subGui = nh.subscribe("eye_control/in", 100, &guiMsgToKinova);
    ros::ServiceServer closeFingersService = nh.advertiseService("eye_control/change_finger_state", &changeFingerState);
    // The following should be used to get the current position of the arm
    // Turned off since unused and caused problem without proper callback
    // function implemented
    // ros::Subscriber subArm = nh.subscribe("j2n6s300_driver/out/tool_pose", 100, &updateArmPos);

    ros::spin();
}

void updateArmPos(const kinova_msgs::KinovaPose& msg)
{
    // This should be the callback function to the subArm subscriber and would
    // update the saved arm position
}

bool sendPose(const kinova_msgs::ArmPoseGoal& pose){
    // This function is used for cartesian position control of the arm
    // whatever pose is input is the amount the will move
    // should be used for implementing the setpoint control, by comparing the 
    // current position to the desired position and then sending the difference
    actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ac("/j2n6s300_driver/pose_action/tool_pose", true);
    ac.waitForServer();
    ROS_INFO_STREAM("Connected to server");
    ac.sendGoal(pose);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout)
    {
        ROS_INFO_STREAM("Action did not finish before the time out.");
        ac.cancelGoal();
        return false;
    }
    else
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO_STREAM("Action finished: " << state.toString().c_str());
    }
    return true;
}

void guiMsgToKinova(const eye_control::guiMsg& msg)
{
    kinova_msgs::PoseVelocityWithFingers msgToPub {};
    msgToPub.twist_linear_x = msg.twist_linear_x;
    msgToPub.twist_linear_y = msg.twist_linear_y;
    msgToPub.twist_linear_z = msg.twist_linear_z;
    msgToPub.twist_angular_x = msg.twist_angular_x;
    msgToPub.twist_angular_y = msg.twist_angular_y;
    msgToPub.twist_angular_z = msg.twist_angular_z;
    
    // The following is not ideal, it should be possible to have the callback
    // function pass the NodeHandle using boost::bind, but I could not get it
    // to work, static means that the NodeHandle is only initialized once
    static ros::NodeHandle nh;
    static ros::Publisher pub = nh.advertise<kinova_msgs::PoseVelocityWithFingers>("j2n6s300_driver/in/cartesian_velocity_with_fingers", 1000);
    pub.publish(msgToPub);
}

bool changeFingerState(eye_control::changeFingerState::Request &req,
                       eye_control::changeFingerState::Response &res){
    ROS_INFO_STREAM("changing finger state");
    bool fingerState = req.state;
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac("/j2n6s300_driver/fingers_action/finger_positions", true);
    ac.waitForServer();
    ROS_INFO_STREAM("Connected to server");

    kinova_msgs::SetFingersPositionGoal goal;
    // 6800 should (untested) be the open position
    int fingerClosed { 6800 };
    goal.fingers.finger1 = fingerState ? 0 : fingerClosed;
    goal.fingers.finger2 = fingerState ? 0 : fingerClosed;
    goal.fingers.finger3 = fingerState ? 0 : fingerClosed;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout)
    {
        ROS_INFO_STREAM("Action did not finish before the time out.");
        ac.cancelGoal();
        return false;
    }
    else
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO_STREAM("Action finished: " << state.toString().c_str());
    }


    return true;
}

