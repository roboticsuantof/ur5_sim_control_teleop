#include <ros/ros.h>
#include <ros/package.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_position_control");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();


    ros::Publisher joint_com_pub; 
	joint_com_pub = n.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 1000);

    
    // Inicializa el grupo de movimiento del brazo UR5
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
   
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");
    // //puntero que lee el estado actual de los joints
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // Print the current joint values
    for (size_t i = 0; i < joint_group_positions.size(); ++i) {
    ROS_INFO("Joint %zu: %f", i, joint_group_positions[i]);
     }




    // Configura la velocidad y la aceleración máximas
    move_group.setMaxVelocityScalingFactor(0.05);  // Ajusta la velocidad máxima
    move_group.setMaxAccelerationScalingFactor(0.05);  // Ajusta la aceleración máxima

    moveit::core::RobotState start_state(*move_group.getCurrentState());

   

std::cout << "1"<< std::endl;

   

     // Define la posición deseada
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.28; // Cambia las coordenadas según tu posición deseada
    target_pose.position.y = 0.26;
    target_pose.position.z = 0.9;
    // target_pose.position.x = 0.5; // Cambia las coordenadas según tu posición deseada
    // target_pose.position.y = 0.6;
    // target_pose.position.z = 0.9;
    target_pose.orientation.x = 0; 
    target_pose.orientation.y = 0; 
    target_pose.orientation.z = 0; 
    target_pose.orientation.w = 1; // Orientación deseada (orientación neutra)
 
    start_state.setFromIK(joint_model_group, target_pose);
    move_group.setStartState(start_state);
    
    // Establece la posición deseada como objetivo
    move_group.setPoseTarget(target_pose);

    // Planifica la trayectoria hacia el objetivo
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

std::cout << "2"<< std::endl; 
   
    // move_group.plan(my_plan);
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

 std::cout << "3"<< std::endl;   
    trajectory_msgs::JointTrajectory mover_arm_trajectory =  my_plan.trajectory_.joint_trajectory ; //define mensaje para publicar
    
    // mover_arm_.joint_trajectory.push_back(mover_arm_trajectory);		
    
    // Imprime la información de la JointTrajectory en la consola
    ROS_INFO("Trajectory Information:");
    ROS_INFO("Header Stamp: %f", mover_arm_trajectory.header.stamp.toSec());
    ROS_INFO("Joint Names: %s", mover_arm_trajectory.joint_names[0].c_str());
    ROS_INFO("Joint Positions: %f", mover_arm_trajectory.points[0].positions[0]);
    ROS_INFO("Joint Velocities: %f", mover_arm_trajectory.points[0].velocities[0]);
std::cout << "34"<< std::endl;
	joint_com_pub.publish(mover_arm_trajectory);
    
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    move_group.clearPathConstraints();
    // if (success)
    // {
    //     // Ejecuta la trayectoria planificada
    //     move_group.move();
    //     ROS_INFO("Movimiento completado.");
    // }
    // else
    // {
    //     ROS_ERROR("No se pudo planificar el movimiento.");
    // }

    ros::shutdown();
    return 0;
}
