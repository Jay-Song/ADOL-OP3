#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>

#include <string>

#define TIME_STEP_MS (8)

std::string op3_joint_names[20] = {
    "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
    "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
    "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
    "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
    "head_pan", "head_tilt"};

std::string webots_joint_names[20] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, 
    "PelvYR" /*ID7 */, "PelvYL" /*ID8 */, "PelvR" /*ID9 */, "PelvL" /*ID10*/,
    "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, 
    "AnkleR" /*ID15*/, "AnkleL" /*ID16*/, "FootR" /*ID17*/, "FootL" /*ID18*/, 
    "Neck" /*ID19*/, "Head" /*ID20*/
};

double goal_joint_angles_rad[20];
double present_joint_angles_rad[20];

bool goal_joint_angle_rcv_flag[20];

webots_ros::set_int time_step_srv;

ros::Publisher present_joint_state_publisher;

ros::Subscriber goal_pos_subs[20];
ros::Subscriber present_pos_subs[20];

ros::ServiceClient webots_time_step;

ros::ServiceClient set_pos_clients[20];
ros::ServiceClient pos_sensor_enable_clients[20];

void posCommandCallback(const std_msgs::Float64::ConstPtr &msg, const int &joint_idx);
void presentJointAnglesCallback(const webots_ros::Float64Stamped::ConstPtr &msg, const int &joint_idx);

void initializePositionSensors();

void sendPresentPosition();
void setGoalPosition();

void process();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_wevots_node");
  ros::NodeHandle nh;
  
  /* Publishers, Subsribers, and Service Clients */
  for(int i = 0; i < 20; i++)
  {
    // make subscribers for the joint position topic from robotis framework
    std::string goal_pos_topic_name = "/robotis_op3/" + op3_joint_names[i] + "_position/command";
    goal_pos_subs[i] = nh.subscribe<std_msgs::Float64>(goal_pos_topic_name, 1, boost::bind(posCommandCallback, _1, i));
    
    // make service clients for setting goal position in webots
    std::string set_pos_srv_name = "/" + webots_joint_names[i] + "/set_position";  
    set_pos_clients[i] = nh.serviceClient<webots_ros::set_float>(set_pos_srv_name);
    
    // make service clients for enabling joint position sensors in webots
    std::string pos_sensor_enable_srv_name = "/" + webots_joint_names[i] + "S/enable";  
    pos_sensor_enable_clients[i] = nh.serviceClient<webots_ros::set_int>(pos_sensor_enable_srv_name);

    // make subscribers for getting present pos angle rad from webots
    std::string pos_sensor_topic_name = "/" + webots_joint_names[i] + "S/value";  
    present_pos_subs[i] = nh.subscribe<webots_ros::Float64Stamped>(pos_sensor_topic_name, 1, boost::bind(presentJointAnglesCallback, _1, i));
  }
  
  //make present joint state publisher
  present_joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/robotis_op3/joint_states", 1);
  webots_time_step = nh.serviceClient<webots_ros::set_int>("/robot/time_step");

  time_step_srv.request.value = TIME_STEP_MS;

  usleep(1000*1000);
  // Initialize Webots
  initializePositionSensors();
  webots_time_step.call(time_step_srv);
  usleep(8*1000);
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  sendPresentPosition();
  ros::spinOnce();
  
  ros::Rate rate(125);
  while(ros::ok())
  {
    ros::spinOnce();
    webots_time_step.call(time_step_srv);
    process();
    rate.sleep();
  }
  return 0;
}

void process()
{
  bool goal_pos_flag = true;

  // while (ros::ok())
  // {
  //   for (int i = 0; i < 20; i++)
  //   {
  //     goal_pos_flag = goal_pos_flag & goal_joint_angle_rcv_flag[i];
  //   }

  //   if (goal_pos_flag)
  //     break;
    
  //   goal_pos_flag = true;
  //   ros::spinOnce();
  // }

  setGoalPosition();
  sendPresentPosition();
}

void initializePositionSensors()
{
  webots_ros::set_int srv;
  srv.request.value = TIME_STEP_MS;
  for(int i = 0; i < 20; i++)
  {
    pos_sensor_enable_clients[i].call(srv);
    if (srv.response.success == 0)
      ROS_ERROR_STREAM("Failed to enable the position sensor of " << webots_joint_names[i]);
  }
}

void posCommandCallback(const std_msgs::Float64::ConstPtr &msg, const int &joint_idx)
{
  goal_joint_angles_rad[joint_idx] = msg->data;
  goal_joint_angle_rcv_flag[joint_idx] = true;
}

void presentJointAnglesCallback(const webots_ros::Float64Stamped::ConstPtr &msg, const int &joint_idx)
{
  present_joint_angles_rad[joint_idx] = msg->data;
}

void sendPresentPosition()
{
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  for(int i = 0; i < 20; i++)
  {
    msg.name.push_back(op3_joint_names[i]);
    msg.position.push_back(present_joint_angles_rad[i]);
    msg.velocity.push_back(0);
    msg.effort.push_back(0);
  }

  present_joint_state_publisher.publish(msg);
}

void setGoalPosition()
{
  webots_ros::set_float srv;
  for(int i = 0; i < 20; i++)
  {
    goal_joint_angle_rcv_flag[i] = false;
    srv.request.value = goal_joint_angles_rad[i];
    set_pos_clients[i].call(srv);

    if (srv.response.success == 0)
      ROS_ERROR_STREAM("Failed to set the goal position of " << webots_joint_names[i]);
  }
}