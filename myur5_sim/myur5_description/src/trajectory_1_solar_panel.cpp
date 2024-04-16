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
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

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
                printf("[%lu] wpd: [%lf %lf %lf][%lf %lf %lf %lf]\n", i, t.x_, t.y_, t.z_, t.qx_, t.qy_, t.qz_, t.qw_);           
                break;
                      
            }
        }
        
    }


    void executeMission(void)
    {
        int taskIndex = 0;
        std::vector<geometry_msgs::Pose> waypoints;
        
        ros::NodeHandle nh_;
       
        // Inicializa el grupo de movimiento del brazo UR5
        static const std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
            
        // const std::string planner_plugin_name = "RRTstar::CommandPlanner";
        // move_group.setPlannerId(planner_plugin_name);

        //tiempo mximo de planificacion
        move_group.setPlanningTime(12.0);
        std::string end_effector_link = "wrist_3_link";
        // double orientation_tolerance = 0.5;
        // double position_tolerance = 1E-6;
        
        // // establecer valores de referencia
        move_group.setEndEffectorLink(end_effector_link);
        // move_group.setGoalOrientationTolerance(orientation_tolerance);
        // move_group.setGoalPositionTolerance(position_tolerance);  

        //mover a HOME POSE
        move_group.setNamedTarget("HomePose2");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool success1 = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Moving to 'HomePose2' %s", success1 ? "SUCCEEDED" : "FAILED");
        std::cout << my_plan.trajectory_ << std::endl;
        // Si la planificación fue exitosa, ejecuta el movimiento.
        if (success1) {
            move_group.move();
        }



        while(ros::ok() && taskIndex < mission_.size()){
                
            // Define la posición deseada
            missionTask t = mission_[taskIndex];
            switch (t.type_) {
                                              
                case taskType::wpd: {

                    //posiciones y orientacion segun archivo de configuracion
                    geometry_msgs::Pose target_pose1;;
                    target_pose1.position.x = t.x_;
                    target_pose1.position.y = t.y_;
                    target_pose1.position.z = t.z_;
                    target_pose1.orientation.x = t.qx_;
                    target_pose1.orientation.y = t.qy_;
                    target_pose1.orientation.z = t.qz_;
                    target_pose1.orientation.w = t.qw_;
                    printf("[%d] wpd [%lf, %lf, %lf][%lf %lf %lf %lf]...\n", taskIndex, t.x_, t.y_, t.z_, t.qx_, t.qy_, t.qz_, t.qw_); 
                    // waypoints.push_back(target_pose);           
                    move_group.setPoseTarget(target_pose1);
                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                    
                    move_group.move();
                    break; 
                }

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
                    printf("[%d] wp [%lf, %lf, %lf][%lf %lf %lf %lf]\n", taskIndex, t.x_, t.y_, t.z_, t.qx_, t.qy_, t.qz_, t.qw_); 
                    
                    std::cout << "66"<< std::endl; 
                   
                    waypoints.push_back(target_pose);           
               }
                       
            }
            taskIndex++;
        }    
       
        
        // const moveit::core::JointModelGroup* joint_model_group1 = move_group.getCurrentState()->getJointModelGroup("manipulator");
        // std::vector<double> joint_group_positions;
        // current_state->copyJointGroupPositions(joint_model_group1, joint_group_positions);

        // // Print the current joint values
        // for (size_t i = 0; i < joint_group_positions.size(); ++i) {
        // ROS_INFO("Joint %zu: %f", i, joint_group_positions[i]);
        // }        
        // // Configura la velocidad y la aceleración máximas
        // move_group.setMaxVelocityScalingFactor(0.2);  // Ajusta la velocidad máxima
        // move_group.setMaxAccelerationScalingFactor(0.2);  // Ajusta la aceleración máxima
        // moveit::core::RobotState start_state(*move_group.getCurrentState());
        // std::cout << "2"<< std::endl; 
        
        moveit_msgs::RobotTrajectory trajectory;
        move_group.setPlanningTime(10.0);

        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
    // The trajectory needs to be modified so it will include velocities as well.
        // First to create a RobotTrajectory object
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "manipulator");

        // Second get a RobotTrajectory from trajectory
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
        
        // Thrid create a IterativeParabolicTimeParameterization object
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        // Fourth compute computeTimeStamps
        bool success = iptp.computeTimeStamps(rt);
        ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
        // Get RobotTrajectory_msg from RobotTrajectory
        rt.getRobotTrajectoryMsg(trajectory);
        // Check trajectory_msg for velocities not empty
        std::cout << trajectory << std::endl;

        plan.trajectory_ = trajectory;
        ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
        // sleep(5.0);                   
        
    // view trajectory in RVIZ.
        // namespace rvt = rviz_visual_tools;
        // moveit_visual_tools::MoveItVisualTools visual_tools("pedestal");
            // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        // text_pose.translation().z() = 1.0;
        // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
        // visual_tools.deleteAllMarkers();
        // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
        // visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
        // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        // for (std::size_t i = 0; i < waypoints.size(); ++i)
        // visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        // visual_tools.trigger();
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        move_group.execute(plan);
        printf("Mission finished!\n");
        ros::spinOnce();           
        
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
                    t.qx_ = file[task]["orientation"]["x"].as<double>();
                    t.qy_ = file[task]["orientation"]["y"].as<double>();
                    t.qz_ = file[task]["orientation"]["z"].as<double>();
                    t.qw_ = file[task]["orientation"]["w"].as<double>();
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
