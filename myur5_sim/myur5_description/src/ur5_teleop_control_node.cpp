#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h> //controladores ur5
// #include <sensor_msgs/JointState.h> //estado de articulaciones
#include <sensor_msgs/Joy.h> //Joystick
#include <sensor_msgs/JointState.h> 
// #include <geometry_msgs/TwistStamped.h> // para conversion de msgs



class TeleopUr5
{
public:
  TeleopUr5();
  bool button_= false;
  int mode_= 0;

   ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_, move_1_, move_2_, move_3_, move_4_, move_5_, move_6_;
  ros::Publisher joints_pub_;
  ros::Subscriber joy_sub_, joint_states_sub_;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
};


TeleopUr5::TeleopUr5(): linear_(1), angular_(2)
{
  printf("Initializing Class TeleopUr5\n");
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
 
  joints_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopUr5::joyCallback, this);
  joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &TeleopUr5::jointstatesCallback,this);

  printf("Class TeleopUr5 created successfully\n");
}

void TeleopUr5::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  trajectory_msgs::JointTrajectory mover_arm_;
  trajectory_msgs::JointTrajectoryPoint points_;
  points_.positions.clear();
  mover_arm_.points.clear();

  if (!button_ && joy->buttons[3] == 1)
  {
     mode_ = mode_ + 1; //mode++ 
     printf("%d\n",mode_);  
     sleep(1);
  }

  
  if (mode_ > 5)
    mode_= 0;

  mover_arm_.joint_names.push_back("shoulder_pan_joint");
  mover_arm_.joint_names.push_back("shoulder_lift_joint");
  mover_arm_.joint_names.push_back("elbow_joint");
  mover_arm_.joint_names.push_back("wrist_1_joint");
  mover_arm_.joint_names.push_back("wrist_2_joint"); 
  mover_arm_.joint_names.push_back("wrist_3_joint");

 
  if (mode_ == 0)
      move_1_ = move_1_ + 0.1 * joy->axes[0];              
  if (mode_ == 1)
      move_2_ = move_2_ + 0.1 * joy->axes[0];
  if  (mode_ == 2)
      move_3_ = move_3_ + 0.1 * joy->axes[0];
  if (mode_ == 3)
      move_4_ = move_4_ + 0.1 * joy->axes[0];
  if (mode_ == 4)
      move_5_ = move_5_ + 0.1 * joy->axes[0];
  if  (mode_ == 5)
      move_6_ = move_6_ + 0.1 * joy->axes[0];
  points_.positions.push_back(move_1_);
  points_.positions.push_back(move_2_);
  points_.positions.push_back(move_3_);
  points_.positions.push_back(move_4_);
  points_.positions.push_back(move_5_);
  points_.positions.push_back(move_6_);

  points_.velocities.push_back(0.0);
  points_.velocities.push_back(0.0);
  points_.velocities.push_back(0.0);
  points_.velocities.push_back(0.0);
  points_.velocities.push_back(0.0);
  points_.velocities.push_back(0.0);

  points_.accelerations.push_back(0.0);
  points_.accelerations.push_back(0.0);
  points_.accelerations.push_back(0.0);
  points_.accelerations.push_back(0.0);
  points_.accelerations.push_back(0.0);
  points_.accelerations.push_back(0.0);

  points_.effort.push_back(0);
 
  points_.time_from_start = ros::Duration(1.0);
  // points_.time_from_start.push_back(n2);

  mover_arm_.points.push_back(points_);

  joints_pub_.publish(mover_arm_);
}

void TeleopUr5::jointstatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  // Almacena las posiciones actuales en las variables miembro
  if (joint_states->position.size() >= 6)
  {
    move_1_ = joint_states->position[0];
    move_2_ = joint_states->position[1];
    move_3_ = joint_states->position[2];
    move_4_ = joint_states->position[3];
    move_5_ = joint_states->position[4];
    move_6_ = joint_states->position[5];
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_ur5_node");
  ros::NodeHandlePtr nh;
  ros::NodeHandle pnh("~");

  TeleopUr5 teleop_ur5;

  // ros::Rate r(ros::Duration(0.05));
	while (ros::ok()) {
    ros::spinOnce();
    // r.sleep();
  }	

	return 0;
}