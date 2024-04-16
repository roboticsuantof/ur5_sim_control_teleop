#include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h> //controladores ur5
// #include <sensor_msgs/JointState.h> //estado de articulaciones
#include <sensor_msgs/Joy.h> //Joystick
// #include <geometry_msgs/TwistStamped.h> // para conversion de msgs


// ros::Publisher jointp_pub; //definicion de pub y sub
// ros::Subscriber joints_sub; joy_sub

// //definir tipo de mensajes

// trajectory_msgs::JointTrajectory jointp_msg;
// sensor_msgs::JointState joints_msg;


// // extraer datos de joystick
// void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
// {
//     // Extract joystick axes values
//     double x = joy->axes[0];
//     double y = joy->axes[1];
//     double z = joy->axes[2];



//     }


// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "simple movimiento joystick");
//     ros::NodeHandle nh:
//     ros::Rate loop_rate(20); //frecuancia de nodo

//     jointp_pub=nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",10)
//     joints_sub= nh.subscribe("/joints_state", 10, jointsstateCallback);
    
// }



class TeleopUr5
{
public:
  TeleopUr5();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_, mode_=0;
  double l_scale_, a_scale_, move_1_, move_2_, move_3_, move_4_, move_5_, move_6_;
  ros::Publisher joints_pub_;
  ros::Subscriber joy_sub_;

};


TeleopUr5::TeleopUr5():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  mode_ = 0;
  move_1_ = 0;
  move_2_ = 0;
  move_3_ = 0;
  move_4_ = 0;
  move_5_ = 0;
  move_6_ = 0;

  joints_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("arm_controllers/command", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopUr5::joyCallback, this);

}

void TeleopUr5::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)

{
//   geometry_msgs::Twist twist;
  trajectory_msgs::JointTrajectory mover_arm_;


  if (mode_ > 5)
    mode_= 0;

  if (joy->buttons[3] == 1)
    mode_ = mode_ + 1; //mode++   


  

  mover_arm_.joint_names [0] = "elbow_joint";
  mover_arm_.joint_names [1] = "shoulder_pan_joint";
  mover_arm_.joint_names [2] = "shoulder_lift_joint";
  mover_arm_.joint_names [3] = "wrist_1_joint";
  mover_arm_.joint_names [4] =  "wrist_2_joint"; 
  mover_arm_.joint_names [5] = "wrist_3_joint";


  if (mode_ == 0){                         
      move_1_ = joy->axes[0];
      mover_arm_.points.positions[] = [move_1_, 0, 0, 0, 0, 0];
  }
 else if (mode_ == 1){
    mover_arm_.points.positions = [0, move_2_, 0, 0, 0, 0];
    move_2_ = joy->axes[0];}
 else if (mode_ == 2){
    mover_arm_.points.positions = [0, 0, move_3_, 0, 0, 0];
    move_3_ = joy->axes[0];
  }
 else if (mode_ == 3){
    mover_arm_.points.positions = [0, 0, 0, move_4_, 0, 0];
    move_4_ = joy->axes[0];
  }
 else if (mode_ == 4){
    move_5_ = joy->axes[0];
    mover_arm_.points.positions = [0, 0, 0, 0, move_5_, 0];
  }
 else if (mode_ == 5){
    move_6_ = joy->axes[0];
    mover_arm_.points.positions = [0, 0, 0, 0, 0, move6_];
  }

  // twist.angular.z = a_scale_*joy->axes[angular_];
  // twist.linear.x = l_scale_*joy->axes[linear_];
  joints_pub_.publish(mover_arm_);//Hay crear uno nuevo

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node_of_PIER");
  TeleopUr5 teleop_ur5;

  ros::spin();
}
