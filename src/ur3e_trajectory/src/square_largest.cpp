#include "../include/circle.hpp"
#include "math.h"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    // Write your code for following the circle trajectory here.
    // Create instance of joint target plan
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;

    joint_targets["shoulder_pan_joint"] = 90*M_PI/180;
    joint_targets["shoulder_lift_joint"] = -45*M_PI/180;
    joint_targets["elbow_joint"] = 70*M_PI/180;
    joint_targets["wrist_1_joint"] = -115*M_PI/180;
    joint_targets["wrist_2_joint"] = -90*M_PI/180;
    joint_targets["wrist_3_joint"] = 0*M_PI/180;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    std::string reference_frame = arm_move_group.getPlanningFrame(); 

    // Largest Possible Square
    // Initialize

    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose0 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose0 = start_pose0;
    ROS_INFO("start coord, x: %f", start_pose0.position.x);
    ROS_INFO("start coord, y: %f", start_pose0.position.y);
    ROS_INFO("start coord, z: %f", start_pose0.position.z);

    end_pose0.position.x = .45;
    end_pose0.position.y = 0;
    end_pose0.position.z = .77 +.128;
    end_pose0.orientation.x = 1;
    end_pose0.orientation.y = 0.0;
    end_pose0.orientation.z = 0.0;
    end_pose0.orientation.w = 0.0;

    ROS_INFO("first coord, x: %f", end_pose0.position.x);
    ROS_INFO("first coord, y: %f", end_pose0.position.y);
    ROS_INFO("first coord, z: %f", end_pose0.position.z);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints0;
    waypoints0.push_back(end_pose0);

    moveit_msgs::RobotTrajectory trajectory0;
    trajectory0 = ArmController::planCartesianPath(start_pose0, waypoints0, reference_frame, arm_move_group);
    
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory0);
    n.setParam("/record_pose", false);

    // corner 2
    geometry_msgs::Pose start_pose1 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose1 = start_pose1;
    // ROS_INFO("start coord, x: %f", start_pose1.position.x);
    // ROS_INFO("start coord, y: %f", start_pose1.position.y);
    // ROS_INFO("start coord, z: %f", start_pose1.position.z);

    end_pose1.position.x = 0;
    end_pose1.position.y = .45;
    end_pose1.position.z = .77 +.128;
    end_pose1.orientation.x = 1;
    end_pose1.orientation.y = 0.0;
    end_pose1.orientation.z = 0.0;
    end_pose1.orientation.w = 0.0;

    ROS_INFO("sec coord, x: %f", end_pose1.position.x);
    ROS_INFO("sec coord, y: %f", end_pose1.position.y);
    ROS_INFO("sec coord, z: %f", end_pose1.position.z);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints1;
    waypoints1.push_back(end_pose1);

    moveit_msgs::RobotTrajectory trajectory1;
    trajectory1 = ArmController::planCartesianPath(start_pose1, waypoints1, reference_frame, arm_move_group);
    
    ///////
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory1);
    n.setParam("/record_pose", false);


    // Corner 3
    geometry_msgs::Pose start_pose2 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose2 = start_pose2;
    // ROS_INFO("start coord, x: %f", start_pose2.position.x);
    // ROS_INFO("start coord, y: %f", start_pose2.position.y);
    // ROS_INFO("start coord, z: %f", start_pose2.position.z);

    end_pose2.position.x = -.45;
    end_pose2.position.y = .45;
    end_pose2.position.z = .77 +.128;
    end_pose2.orientation.x = 1;
    end_pose2.orientation.y = 0.0;
    end_pose2.orientation.z = 0.0;
    end_pose2.orientation.w = 0.0;

    ROS_INFO("third coord, x: %f", end_pose2.position.x);
    ROS_INFO("third coord, y: %f", end_pose2.position.y);
    ROS_INFO("third coord, z: %f", end_pose2.position.z);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(end_pose2);

    moveit_msgs::RobotTrajectory trajectory2;
    trajectory2 = ArmController::planCartesianPath(start_pose2, waypoints2, reference_frame, arm_move_group);
    
    ///////
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory2);
    n.setParam("/record_pose", false);

    // corner 4
    geometry_msgs::Pose start_pose3 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose3 = start_pose3;
    // ROS_INFO("start coord, x: %f", start_pose3.position.x);
    // ROS_INFO("start coord, y: %f", start_pose3.position.y);
    // ROS_INFO("start coord, z: %f", start_pose3.position.z);

    end_pose3.position.x = 0;
    end_pose3.position.y = .1;
    end_pose3.position.z = .77 +.128;
    end_pose3.orientation.x = 1;
    end_pose3.orientation.y = 0.0;
    end_pose3.orientation.z = 0.0;
    end_pose3.orientation.w = 0.0;

    ROS_INFO("fourth coord, x: %f", end_pose3.position.x);
    ROS_INFO("fourth coord, y: %f", end_pose3.position.y);
    ROS_INFO("fourth coord, z: %f", end_pose3.position.z);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints3;
    waypoints3.push_back(end_pose3);

    moveit_msgs::RobotTrajectory trajectory3;
    trajectory3 = ArmController::planCartesianPath(start_pose3, waypoints3, reference_frame, arm_move_group);
    
    ///////
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory3);
    n.setParam("/record_pose", false);

    // back to first corner
    geometry_msgs::Pose start_pose4 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose4 = start_pose4;
    // ROS_INFO("start coord, x: %f", start_pose4.position.x);
    // ROS_INFO("start coord, y: %f", start_pose4.position.y);
    // ROS_INFO("start coord, z: %f", start_pose4.position.z);

    end_pose4.position.x = .45;
    end_pose4.position.y = 0;
    end_pose4.position.z = .77 +.128;
    end_pose4.orientation.x = 1;
    end_pose4.orientation.y = 0.0;
    end_pose4.orientation.z = 0.0;
    end_pose4.orientation.w = 0.0;

    ROS_INFO("first coord, x: %f", end_pose4.position.x);
    ROS_INFO("first coord, y: %f", end_pose4.position.y);
    ROS_INFO("first coord, z: %f", end_pose4.position.z);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints4;
    waypoints4.push_back(end_pose4);

    moveit_msgs::RobotTrajectory trajectory4;
    trajectory4 = ArmController::planCartesianPath(start_pose4, waypoints4, reference_frame, arm_move_group);
    
    ///////
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory4);
    n.setParam("/record_pose", false);
}