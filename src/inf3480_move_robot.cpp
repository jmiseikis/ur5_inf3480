#include <iostream>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
//#include <moveit/core/transforms/transforms.h>	// Eigen transforms

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <leap_motion/leapros.h>

#include "std_msgs/String.h"

// Include Leap Motion
#include <leap_motion/leapros.h>

class MoveRobot
{

	ros::NodeHandle nh_;
	// Leap Motion data subscriber
	ros::Subscriber leap_target_pose_sub_;

public:

	// Global planning scene interface
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
	
	// Collision object
	moveit_msgs::CollisionObject collision_object;
	std::vector<moveit_msgs::CollisionObject> collision_objects_list;

	MoveRobot()
	{
		/* This sleep is ONLY to allow Rviz to come up */
		sleep(10.0);

		moveit::planning_interface::MoveGroup group("manipulator");

		// Pre-programmed robot move
		moveRobotToHome();
		moveRobotToHomeWithFloor();
		moveRobotCartesianPath();

		// ############ LEAP MOTION ############
		// collision_object.header.frame_id = "base_link";//group.getPlanningFrame();

		// // The id of the object is used to identify it
		// collision_object.id = "box1";

		// // Define a box to add to the world
		// shape_msgs::SolidPrimitive primitive;
		// primitive.type = primitive.BOX;
		// primitive.dimensions.resize(3);
		// primitive.dimensions[0] = 0.3;
		// primitive.dimensions[1] = 0.3;
		// primitive.dimensions[2] = 0.3;

		// collision_object.primitives.push_back(primitive);

		// // A pose for the box (specified relative to frame_id) 
		// geometry_msgs::Pose box_pose;
		// box_pose.orientation.w = 1.0;
		// box_pose.position.x = 0;
		// box_pose.position.y = 0;
		// box_pose.position.z =  -0.15;

		// collision_object.primitive_poses.push_back(box_pose);
		// collision_object.operation = collision_object.ADD;

		// collision_objects_list.push_back(collision_object);  

		// leap_target_pose_sub_ = nh_.subscribe("/leapmotion/data", 1, &MoveRobot::AddLeapMotionObstacle, this);

		// ############ END ############
	}

	~MoveRobot()
	{
	}

	void moveRobotToHome()
	{
		moveit::planning_interface::MoveGroup group("manipulator");

		// We will use the :planning_scene_interface:`PlanningSceneInterface`
  		// class to deal directly with the world.
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

		// (Optional) Create a publisher for visualizing plans in Rviz.
		ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		moveit_msgs::DisplayTrajectory display_trajectory;

		// Getting Basic Information
		// We can print the name of the reference frame for this robot.
		ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

		// We can also print the name of the end-effector link for this group.
		ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

		// Planning to a Pose goal
		// ^^^^^^^^^^^^^^^^^^^^^^^
		// We can plan a motion for this group to a desired pose for the 
		// end-effector.
		geometry_msgs::Pose target_pose1;
		target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
		target_pose1.position.x = 0.10;
		target_pose1.position.y = 0.25;
		target_pose1.position.z = 0.50;
		group.setPoseTarget(target_pose1);

		// Now, we call the planner to compute the plan
		// and visualize it.
		// Note that we are just planning, not asking move_group 
		// to actually move the robot.
		moveit::planning_interface::MoveItErrorCode success = group.asyncMove();
		

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
		// Sleep to give Rviz time to visualize the plan 
		sleep(5.0);

		//Execute the movement on real robot
		//group.move();
	}

	void moveRobotToHomeWithFloor()
	{

		moveit::planning_interface::MoveGroup group("manipulator");

		// We will use the :planning_scene_interface:`PlanningSceneInterface`
  		// class to deal directly with the world.
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

		// (Optional) Create a publisher for visualizing plans in Rviz.
		ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		moveit_msgs::DisplayTrajectory display_trajectory;

		// Get starting robot state
		robot_state::RobotState start_state = *group.getCurrentState();

		// Set replanning time
		group.setPlanningTime(10.0);

		// Planning to a Pose goal
		// ^^^^^^^^^^^^^^^^^^^^^^^
		// We can plan a motion for this group to a desired pose for the 
		// end-effector.
		geometry_msgs::Pose target_pose1;
		target_pose1.orientation.w = 1;// = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
		target_pose1.position.x = 0.10;
		target_pose1.position.y = 0.25;
		target_pose1.position.z = 0.50;
		group.setPoseTarget(target_pose1);

		// Now, we call the planner to compute the plan
		// and visualize it.
		// Note that we are just planning, not asking move_group 
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;
		//bool success = group.plan(my_plan);
		moveit::planning_interface::MoveItErrorCode success = group.asyncMove();

		//bool success = group.move();

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
		// Sleep to give Rviz time to visualize the plan
		sleep(5.0);

		// Execute the movement on the real robot
		//group.move();

		// Getting Basic Information
		// We can print the name of the reference frame for this robot.
		ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

		// We can also print the name of the end-effector link for this group.
		ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

		// Adding/Removing Objects and Attaching/Detaching Objects
		// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		// First, we will define the collision object message.
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = "base_link";//group.getPlanningFrame();

		// The id of the object is used to identify it
		collision_object.id = "box1";

		// Define a box to add to the world
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[0] = 2;
		primitive.dimensions[1] = 2;
		primitive.dimensions[2] = 0.1;

		// A pose for the box (specified relative to frame_id)
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0;
		box_pose.position.y = 0;
		box_pose.position.z =  -0.15;

		// Push back size and pose
		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		// Add to the list of collision objects
		std::vector<moveit_msgs::CollisionObject> collision_objects;  
		collision_objects.push_back(collision_object);  

		// Now, let's add the collision object into the world
		ROS_INFO("Add an object into the world");  
		planning_scene_interface.addCollisionObjects(collision_objects);

		// Sleep so we have time to see the object in RViz
		sleep(2.0);
		
		// Replan
		group.setStartState(start_state);
		group.setPoseTarget(target_pose1);
		// Move with obstacle avoidance
		success = group.plan(my_plan);

		//bool success = group.move();

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 
	}

	void moveRobotCartesianPath()
	{
		moveit::planning_interface::MoveGroup group("manipulator");

		// We will use the :planning_scene_interface:`PlanningSceneInterface`
  		// class to deal directly with the world.
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

		// (Optional) Create a publisher for visualizing plans in Rviz.
		ros::Publisher display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
		moveit_msgs::DisplayTrajectory display_trajectory;

		// Allow replanning
		group.allowReplanning(true);

		// Set the start state to current state
		group.setStartState(*group.getCurrentState());

		// Re-define starting position
		std::vector<geometry_msgs::Pose> waypoints;
		geometry_msgs::Pose target_pose1;
		target_pose1 = group.getCurrentPose().pose;
		waypoints.push_back(target_pose1);

		// Position 1
		target_pose1.position.x -= 0.2;
		waypoints.push_back(target_pose1);

		// Position 2
		target_pose1.position.z += 0.2;
		waypoints.push_back(target_pose1);

		// Position 3
		target_pose1.position.x -= 0.2;
		waypoints.push_back(target_pose1);

		// We want the cartesian path to be interpolated at a resolution of 1 cm
		// which is why we will specify 0.01 as the max step in cartesian
		// translation.  We will specify the jump threshold as 0.0, effectively
		// disabling it.
		moveit_msgs::RobotTrajectory trajectory;
		double fraction = group.computeCartesianPath(waypoints,
		                                           0.01,  // eef_step
		                                           0.0,   // jump_threshold
		                                           trajectory);

		// Get robot trajectory to which joint speeds will be added
		robot_trajectory::RobotTrajectory robot_move_trajectory(group.getCurrentState()->getRobotModel(), "manipulator");
		// Second get a RobotTrajectory from trajectory
		robot_move_trajectory.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);

		ROS_INFO("Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);    
		// Sleep to give Rviz time to visualize the plan
		sleep(5.0);

		moveit::planning_interface::MoveItErrorCode success = group.asyncMove();
	}

	void AddLeapMotionObstacle(const leap_motion::leapros& leap_msg)
	{
		geometry_msgs::Point target_point;

		target_point = leap_msg.palmpos;

		//ROS_INFO("Leap motion position %f %f %f", target_point.x, target_point.y, target_point.z);

		// Remap the leap motion data
		//collision_objects[0].primitive_poses[0].position.x = (target_point.x) / 500;
		//collision_objects[0].primitive_poses[0].position.y = (target_point.z * (-1)) / 500;
		//collision_objects[0].primitive_poses[0].position.z = (target_point.y) / 500;
		//collision_objects.push_back(collision_object);  

		// Now, let's add the collision object into the world
		//ROS_INFO("Add an object into the world");  
		//planning_scene_interface.addCollisionObjects(collision_objects);
		//sleep(0.1);
	}

}; // End of class

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_robot", ros::init_options::AnonymousName);
	MoveRobot mv;
	ros::spin();

	ros::waitForShutdown();
	return 0;
}
