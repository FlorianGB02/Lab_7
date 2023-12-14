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
    
    // Horizontal Square
    // Initialize
    int p = 4;                      // number of nodes
    float r = .1;                   // 'radius' of square
    int i;                          // used in for loop
    float theta = 2*M_PI/p;         // angle
    float l = 2*r*sin(theta/2);     // length of sides

    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose0 = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose0 = start_pose0;
    ROS_INFO("start coord, x: %f", start_pose0.position.x);
    ROS_INFO("start coord, y: %f", start_pose0.position.y);
    ROS_INFO("start coord, z: %f", start_pose0.position.z);

    end_pose0.position.x -= l/2;
    end_pose0.position.y += l/2;
    end_pose0.orientation.x = 1;
    end_pose0.orientation.y = 0.0;
    end_pose0.orientation.z = 0.0;
    end_pose0.orientation.w = 0.0;
    // end_pose0.orientation = reference_orientation;

    ROS_INFO("first coord, x: %f", end_pose0.position.x);
    ROS_INFO("first coord, y: %f", end_pose0.position.y);
    ROS_INFO("first coord, z: %f", end_pose0.position.z);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints0;
    waypoints0.push_back(end_pose0);

    moveit_msgs::RobotTrajectory trajectory0;
    trajectory0 = ArmController::planCartesianPath(start_pose0, waypoints0, reference_frame, arm_move_group);
    
    ///////
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory0);
    n.setParam("/record_pose", false);

    // for loop over the 20 nodes
    for(i = 1; i <= p; ++i)
    {
        
    // Get the start Pose
    geometry_msgs::Pose start_pose1 = arm_move_group.getCurrentPose().pose;
    geometry_msgs::Pose end_pose1 = start_pose1;
    
    end_pose1.position.x += r*sin(i*theta);
    end_pose1.position.y += r*cos(i*theta);
    // end_pose1.orientation = reference_orientation;
    end_pose1.orientation.x = 1;
    end_pose1.orientation.y = 0.0;
    end_pose1.orientation.z = 0.0;
    end_pose1.orientation.w = 0.0;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints1;
    waypoints1.push_back(end_pose1);

    moveit_msgs::RobotTrajectory trajectory1;
    trajectory1 = ArmController::planCartesianPath(start_pose1, waypoints1, reference_frame, arm_move_group);
    
    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory1);
    n.setParam("/record_pose", false);
    ROS_INFO("next coord, x: %f", end_pose1.position.x);
    ROS_INFO("next coord, y: %f", end_pose1.position.y);
    ROS_INFO("next coord, z: %f", end_pose1.position.z);
        
    }

}