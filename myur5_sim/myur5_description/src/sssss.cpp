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


#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <yaml-cpp/yaml.h>
#include <std_srvs/Trigger.h>

#include <industrial_trajectory_filters/filter_base.h>
#include <industrial_trajectory_filters/uniform_sample_filter.h>

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
                      
            }
        }
        
    }


    void executeMission(void)
    {
        int taskIndex = 0;
        ros::NodeHandle nh_;

        // Inicializa el grupo de movimiento del brazo UR5
        moveit::planning_interface::MoveGroupInterface move_group("manipulator");
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
        moveit::planning_interface::MoveGroupInterface::Plan plan;
               
        // tiempo mximo de planificacion
        move_group.setPlanningTime(10.0);
         
        std::vector<geometry_msgs::Pose> waypoints;
        
        // Configura la velocidad y la aceleración máximas
        move_group.setMaxVelocityScalingFactor(0.5);  // Ajusta la velocidad máxima
        move_group.setMaxAccelerationScalingFactor(0.5);  // Ajusta la aceleración máxima
       
        std::string end_effector_link = "wrist_3_link";
        double orientation_tolerance = 0.5;
        double position_tolerance = 1E-3;
        double joint_tolerance= 0.1;
        
        // establecer valores de referencia
        move_group.setEndEffectorLink(end_effector_link);
        move_group.setGoalOrientationTolerance(orientation_tolerance);
        move_group.setGoalPositionTolerance(position_tolerance);
        move_group.setGoalJointTolerance(joint_tolerance);
        
        

        while(ros::ok() && taskIndex < mission_.size()){
             
            // Define la posición deseada
            missionTask t = mission_[taskIndex];
            switch (t.type_) {
                case taskType::wp: {

                    //posiciones y orientacion segun archivo de configuracion
                    geometry_msgs::Pose target_pose;
                    target_pose.position.x = t.x_;
                    target_pose.position.y = t.y_;
                    target_pose.position.z = t.z_;
                    target_pose.orientation.x = t.qx_;
                    target_pose.orientation.y = t.qy_;
                    target_pose.orientation.z = t.qz_;
                    target_pose.orientation.w = t.qw_;
                    printf("[%d] wp [%lf, %lf, %lf] ...\n", taskIndex, t.x_, t.y_, t.z_); 
                    waypoints.push_back(target_pose);           
                                    
                }
                taskIndex++;
            }
                        
        }

        for (const auto& waypoints : waypoints) {
        std::cout << "Position - x: " << waypoints.position.x
              << ", y: " << waypoints.position.y
              << ", z: " << waypoints.position.z
              << "; Orientation - x: " << waypoints.orientation.x
              << ", y: " << waypoints.orientation.y
              << ", z: " << waypoints.orientation.z
              << ", w: " << waypoints.orientation.w << std::endl;
        }

        moveit_msgs::RobotTrajectory trajectory;
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

        

        // Filtering the "trajectory" with Uniform Sampler
        industrial_trajectory_filters::MessageAdapter t_in;
        t_in.request.trajectory = trajectory.joint_trajectory;
        industrial_trajectory_filters::MessageAdapter t_out;
        industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
        adapter.update(t_in, t_out);

          // Adding the "trajectory" to the plan.
            trajectory.joint_trajectory = t_out.request.trajectory;
        plan.trajectory_ = trajectory;
            if (fraction == 1.0) {
                move_group.execute(plan);
            } else
                ROS_WARN("Could not compute the cartesian path :( ");
        // move_group.execute(plan);
        // ros::spin();
               
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
