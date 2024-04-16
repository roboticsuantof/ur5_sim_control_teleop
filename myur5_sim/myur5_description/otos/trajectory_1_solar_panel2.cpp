#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <fstream>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <yaml-cpp/yaml.h>
#include <std_srvs/Trigger.h>

class MissionInterface
{
    enum taskType {unknown = 0, take_off, land, wp, wpd, set_home, wait, wait_trigger};
    
    struct missionTask
    {
        taskType type_;
        double x_, y_, z_;
        double qx_, qy_, qz_, qw_;
    };

public:
    

    MissionInterface(std::string node_name)
    {
        // Local node handler
        // nh_.reset(new ros::NodeHandle("~"));  
        ros::NodeHandle nh_("~");

        // Read node parameters
        nh_.param("mission_file_path", mission_file_, (std::string) "~/");
        nh_.param("start_immediately", startImmediately_, true);
        
        // Load yaml file with the mission tasks
        loadMission(mission_file_);
        printf("Mission:\n");
        for (size_t i =0; i < mission_.size(); i++ ){
            missionTask &t = mission_[i];
            switch (t.type_)
            {
                case wp:
                printf("[%lu] wp: [%lf %lf %lf][%lf %lf %lf %lf]\n", i, t.x_, t.y_, t.z_, t.qx_, t.qy_, t.qz_, t.qw_);           
                break;
                case wpd:
                printf("[%lu] wp: [%lf %lf %lf]\n", i, t.x_, t.y_, t.z_);               
                break;
          
            }
        }
        
    }


    void executeMission(void)
    {
        int taskIndex = 0;
        ros::NodeHandle nh_;

        ros::Publisher joint_com_pub; 
        joint_com_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 1000,true);
        std::vector<geometry_msgs::Pose> waypoints;
        
        while(ros::ok() && taskIndex < mission_.size()){
                      
                              
            // Define la posición deseada
            missionTask t = mission_[taskIndex];
            switch (t.type_) {
            case taskType::wp: {

                //posiciones y orientacion segun archivo de configuracion
                
                geometry_msgs::Pose target_pose;;
                target_pose.position.x = t.x_;
                target_pose.position.y = t.y_;
                target_pose.position.z = t.z_;
                target_pose.orientation.x = t.qx_;
                target_pose.orientation.y = t.qy_;
                target_pose.orientation.z = t.qz_;
                target_pose.orientation.w = t.qw_;
                printf("[%d] wp [%lf, %lf, %lf] ...\n", taskIndex, t.x_, t.y_, t.z_); 
                
                waypoints.push_back(target_pose);
                taskIndex++;
                
                                
            }
            // case taskType::wpd: {

            //     const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("manipulator");
                
            //     moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

            //     std::vector<double> joint_group_positions;
            //     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            //     // Print the current joint values
            //     for (size_t i = 0; i < joint_group_positions.size(); ++i) {
            //     ROS_INFO("Joint %zu: %f", i, joint_group_positions[i]);
            //     }        

            //     // Configura la velocidad y la aceleración máximas
            //     move_group.setMaxVelocityScalingFactor(0.1);  // Ajusta la velocidad máxima
            //     move_group.setMaxAccelerationScalingFactor(0.1);  // Ajusta la aceleración máxima

            //     moveit::core::RobotState start_state(*move_group.getCurrentState());
            //     // establecer link como end effector
            //     std::string end_effector_link = "wrist_3_link";
            //     move_group.setEndEffectorLink(end_effector_link);

            //     //posiciones y orientacion segun archivo de configuracion
            //     geometry_msgs::Pose target_pose;;
            //     target_pose.position.x = t.x_;
            //     target_pose.position.y = t.y_;
            //     target_pose.position.z = t.z_;
            //     printf("[%d] wp [%lf, %lf, %lf] ...\n", taskIndex, t.x_, t.y_, t.z_); 
                
            //     start_state.setFromIK(joint_model_group, target_pose);
            //     move_group.setStartState(start_state);

            //     // Establece la posición deseada como objetivo
            //     move_group.setPoseTarget(target_pose);

            //     // Planifica la trayectoria hacia el objetivo
            //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            //     bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            //     ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            //     trajectory_msgs::JointTrajectory mover_arm_trajectory =  my_plan.trajectory_.joint_trajectory ; //define mensaje para publicar

            //     // Imprime la información de la JointTrajectory en la consola
            //     ROS_INFO("Trajectory Information:");
            //     ROS_INFO("Header Stamp: %f", mover_arm_trajectory.header.stamp.toSec());
            //     ROS_INFO("Joint Names: %s", mover_arm_trajectory.joint_names[0].c_str());
            //     ROS_INFO("Joint Positions: %f", mover_arm_trajectory.points[0].positions[0]);
            //     ROS_INFO("Joint Velocities: %f", mover_arm_trajectory.points[0].velocities[0]);
            //     // ROS_INFO("time_from_estart: %f", mover_arm_trajectory.points.back().time_from_start);
            //     ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str()); 
            //     joint_com_pub.publish(mover_arm_trajectory);
                
            //     // move_group.move();

                
            //     // ros::Duration espera(10.0);

            //     // // Dormimos durante 10 segundos
            //     // espera.sleep();
            //     mover_arm_trajectory.points.back().time_from_start.sleep();
                
            //     move_group.clearPathConstraints();

            //     taskIndex++;
                
                                
            // }
       
            // Inicializa el grupo de movimiento del brazo UR5
            moveit::planning_interface::MoveGroupInterface move_group("manipulator");
                    

                // Configura la velocidad y la aceleración máximas
                move_group.setMaxVelocityScalingFactor(0.1);  // Ajusta la velocidad máxima
                move_group.setMaxAccelerationScalingFactor(0.1);  // Ajusta la aceleración máxima
                
                // establecer link como end effector
                std::string end_effector_link = "wrist_3_link";
                move_group.setEndEffectorLink(end_effector_link);

                moveit_msgs::RobotTrajectory trajectory1;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;
                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory1);
               
                trajectory_msgs::JointTrajectory mover_arm_trajectory =  trajectory1.joint_trajectory;
                joint_com_pub.publish(mover_arm_trajectory);
                
                // move_group.execute(trajectory);

                ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
                // Imprime la información de la JointTrajectory en la consola
                // ROS_INFO("Trajectory Information:");
                // ROS_INFO("Header Stamp: %f", mover_arm_trajectory.header.stamp.toSec());
                // ROS_INFO("Joint Names: %s", mover_arm_trajectory.joint_names[0].c_str());
                // ROS_INFO("Joint Positions: %f", mover_arm_trajectory.points[0].positions[0]);
                // ROS_INFO("Joint Velocities: %f", mover_arm_trajectory.points[0].velocities[0]);
                // ROS_INFO("time_from_estart: %f", mover_arm_trajectory.points.back().time_from_start);
                // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str()); 
     
                
                move_group.clearPathConstraints();      
                taskIndex++;
        }
        printf("Mission finished!\n");
    }


    }

    void loadMission(const std::string &path_file)
    {
        YAML::Node file = YAML::LoadFile(path_file);

        // read number of taks  
        int size = (file["length"].as<int>());

        // read tasks one by one 
        mission_.clear();
        for (int i = 0; i < size; i++) {
            missionTask t;

            // Create the task text to search
            std::string task = "task" + std::to_string(i);

            // Read task type and parse it
            std::string typeString = file[task]["type"].as<std::string>();
            if(typeString.compare("take-off") == 0)
                t.type_ = taskType::take_off;
            else if(typeString.compare("land") == 0)
                t.type_ = taskType::land;
            else if(typeString.compare("wp") == 0)
                t.type_ = taskType::wp;
            else if(typeString.compare("wpd") == 0)
                t.type_ = taskType::wpd;
            else if(typeString.compare("set-home") == 0)
                t.type_ = taskType::set_home;
            else if(typeString.compare("wait") == 0)
                t.type_ = taskType::wait;
            else if(typeString.compare("wait-trigger") == 0)
                t.type_ = taskType::wait_trigger;
            else 
            {
                ROS_WARN("%s: Unknown type %s. skipping.", typeString.c_str(), task.c_str()); 
                continue;
            }
            
            // Read task parameters depending on the task type
            switch(t.type_)
            {
                
                case taskType::wp:
                    t.x_ = file[task]["position"]["x"].as<double>();
                    t.y_ = file[task]["position"]["y"].as<double>();
                    t.z_ = file[task]["position"]["z"].as<double>();
                    t.qx_ = file[task]["orientation"]["x"].as<double>();
                    t.qy_ = file[task]["orientation"]["y"].as<double>();
                    t.qz_ = file[task]["orientation"]["z"].as<double>();
                    t.qw_ = file[task]["orientation"]["w"].as<double>();
                    break;
                case taskType::wpd:
                    t.x_ = file[task]["position"]["x"].as<double>();
                    t.y_ = file[task]["position"]["y"].as<double>();
                    t.z_ = file[task]["position"]["z"].as<double>();
                    break;
            }

            // Store the task into the mission
            mission_.push_back(t);
        }
        std::cout << "YAML FILE readed. YAML FILE NAME: " << path_file << std::endl;
        std::cout << "Number of tasks: " << mission_.size() << std::endl;
    }

    // Node params
    bool startImmediately_;
    std::string base_frame_, world_frame_, mission_file_; 

    // Node variables
    std::vector<missionTask> mission_;
    bool startMission_;
    // ros::NodeHandlePtr nh_;
 
    bool inputTriggers_[10];
};

int main(int argc, char** argv)
{
    std::string node_name = "trajectory_1_solar_panel";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ros::NodeHandle n;
    MissionInterface mission(node_name);
    mission.executeMission();

    return 0;
   

}
