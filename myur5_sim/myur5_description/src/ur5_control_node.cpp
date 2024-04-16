#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include <urdf/model.h>
#include <iostream>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "control_msgs/JointTrajectoryControllerState.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "control_msgs/JointControllerState.h"
#include "sensor_msgs/JointState.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

const int Joints = 6;
KDL::JntArray jnt_pos_start(Joints);

// void get_arm_joint_position(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
void get_arm_joint_position(const sensor_msgs::JointState::ConstPtr& msg) {
	jnt_pos_start(0) = msg->position[0];
	jnt_pos_start(1) = msg->position[1];
	jnt_pos_start(2) = msg->position[2];
	jnt_pos_start(3) = msg->position[3];
	jnt_pos_start(4) = msg->position[4];
	jnt_pos_start(5) = msg->position[5];
}

float compute_linear(double q_start, double q_goal, float t, float t_max) {
	return((q_goal - q_start) * (t/t_max) + q_start);
}

void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {

	std::cout << "Please define the offset you want to move for each axis and the time in which the motion should be completed:\n";
		//Get user input
		float x,y,z;
		std::cout << "x:";
		std::cin >> x;
		std::cout << "y:";
		std::cin >> y;
		std::cout << "z:";
		std::cin >> z;
		std::cout << "Time:";
		std::cin >> (*t_max);

		(*vec_tcp_pos_goal)(0) = (x);
		(*vec_tcp_pos_goal)(1) = (y);
		(*vec_tcp_pos_goal)(2) = (z);
}

double distanciaEuclidiana(double x1, double y1, double z1, double x2, double y2, double z2) {
  
   return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));

}


const int loop_rate_val = 100;

int main(int argc, char **argv)
{
		double errorMinimo = 0.05;
		double errorActual;
		double punto1_x, punto1_y, punto1_z, punto2_x, punto2_y, punto2_z;


		std::string urdf_path = ros::package::getPath("myur5_description");
		
		if(urdf_path.empty()) {
			ROS_ERROR("ur_description package path was not found");
		}
		urdf_path += "/urdf/myur5.urdf";

		ros::init(argc, argv, "tcp_control");
		ros::NodeHandle n;
		ros::Rate loop_rate(loop_rate_val);

		//Create subscribers for all joint states
		ros::Subscriber joint_sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1000, get_arm_joint_position);

		//Create publishers to send position commands to all joints
		ros::Publisher joint_com_pub; 
		joint_com_pub = n.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 1000);

				
			//Parse urdf model and generate KDL tree
		KDL::Tree ur5_tree;
		if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
			ROS_ERROR("Failed to construct kdl tree");
			return false;
		}

		//Generate a kinematic chain from the robot base to its tcp
		KDL::Chain ur5_chain;
		ur5_tree.getChain("pedestal", "tool0", ur5_chain);

		std::cout << "Número de articulaciones en la cadena: " <<ur5_chain.getNrOfJoints() << std::endl;

		//Create solvers
		KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
		KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001,10);
		KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver,5);

		//Make sure we have received proper joint angles already
		for(int i=0; i< 2; i++) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		const float t_step = 1/((float)loop_rate_val);
		int count = 0;
	
		while (ros::ok()) {

				
				//Compute current tcp position
				KDL::Frame tcp_pos_start;
				fk_solver.JntToCart(jnt_pos_start, tcp_pos_start); //resuelve cinematica directa entregando tcp y orientacion
			
				// tcp_pos_start.p(2) = tcp_pos_start.p(2) + 0.4945;
				ROS_INFO("Current tcp Position/Twist KDL:");		
				ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));		
				ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

				//get user input
				float t_max;
				KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0); //crea un vector para posicion x,y,z 
				get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);
			do
			{	
				errorActual = 0; 

				std::cout << "vec_tcp_pos_goal(0): " << vec_tcp_pos_goal(0) << " , vec_tcp_pos_goal(1): " << vec_tcp_pos_goal(1) << " , vec_tcp_pos_goal(2): " << vec_tcp_pos_goal(2) << std::endl;
				

				// double x_orientation = 0.0048;
				// double y_orientation = 0.8262;
				// double z_orientation = 0.00263;
				// double w_orientation = 0.5632;
				// KDL::Rotation rotation_goal = KDL::Rotation::Quaternion(x_orientation, y_orientation, z_orientation, w_orientation);
		
				KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal); //crea kdl::frame de tcp final pero manteniendo la misma orientacion
				// KDL::Frame tcp_pos_goal(rotation_goal, vec_tcp_pos_goal); 

				std::cout << "tcp_pos_goal(0): " << tcp_pos_goal.p(0) << " , tcp_pos_goal(1): " << tcp_pos_goal.p(1) << " , tcp_pos_goal(2): " << tcp_pos_goal.p(2) << std::endl;
				// std::cout << "tcp_pos_goal.ORIENTATION(0): " << tcp_pos_goal.M().data[0] << " , tcp_pos_goal(1): " << tcp_pos_goal.M().data[1] << " , tcp_pos_goal(2): " << tcp_pos_goal.M().data[2] << std::endl;

				//Compute inverse kinematics
				KDL::JntArray jnt_pos_goal(Joints);
				ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);
					
				float t = 0.0;
				while(t<t_max) {
					std_msgs::Float64 position_;
					std_msgs::Float64 zero_;
					// float position_;
					trajectory_msgs::JointTrajectory mover_arm_;
					trajectory_msgs::JointTrajectoryPoint points_;
					// points_.positions[Joints];
					points_.positions.clear();
					mover_arm_.points.clear();
					points_.positions[Joints];

					mover_arm_.joint_names.push_back("shoulder_pan_joint");
					mover_arm_.joint_names.push_back("shoulder_lift_joint");
					mover_arm_.joint_names.push_back("elbow_joint");
					mover_arm_.joint_names.push_back("wrist_1_joint");
					mover_arm_.joint_names.push_back("wrist_2_joint"); 
					mover_arm_.joint_names.push_back("wrist_3_joint");

					//Compute next position step for all joints
					for(int i=0; i<Joints; i++) {
						position_.data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
						zero_.data = 0.0;
						points_.positions.push_back(position_.data);
						points_.velocities.push_back(zero_.data);
						points_.accelerations.push_back(zero_.data);
						points_.effort.push_back(zero_.data);
					}
					
					points_.time_from_start = ros::Duration(1.0);
					mover_arm_.points.push_back(points_);				
					joint_com_pub.publish(mover_arm_);
					
								
					ros::spinOnce();
					loop_rate.sleep();
					++count;
					t += t_step;	
				}		
					
				fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);
						
				double punto1_x = tcp_pos_start.p(1);
				double punto1_y = tcp_pos_start.p(1);
				double punto1_z = tcp_pos_start.p(2);
				double punto2_x = vec_tcp_pos_goal(0);
				double punto2_y = vec_tcp_pos_goal(1);
				double punto2_z = vec_tcp_pos_goal(2);

				errorActual = distanciaEuclidiana(punto1_x, punto1_y, punto1_z, punto2_x, punto2_y, punto2_z);

				// std::cout << "La distancia euclidiana entre los puntos es: " << errorActual << std::endl;
				std::cout << "errorActual1: " << errorActual << std::endl;
				std::cout << "errorMinimo2: " << errorMinimo << std::endl;

				} while (errorActual > errorMinimo);
		}	
		
	
	return 0;
	
}



