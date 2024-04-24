#include "op3_webots_ros/op3_extern_ros_controller.h"

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>
#include <webots/Camera.hpp>

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

using namespace std;
using namespace adol;

OP3ExternROSController::OP3ExternROSController()
{
  time_step_ms_  = 8;
  time_step_sec_ = 0.008;

  current_time_sec_ = 0; // control time

  // for motor angle
  for (int i = 0; i < N_MOTORS; i++)
  {
    desired_joint_angle_rad_[i] = 0;
    current_joint_angle_rad_[i] = 0;
    current_joint_torque_Nm_[i] = 0;
  }  

  // center of mass
  for (int i = 0; i < 3; i++)
  {
    current_com_m_[i]       = 0;
    previous_com_m_[i]      = 0;
    current_com_vel_mps_[i] = 0;
  }  
}

OP3ExternROSController::~OP3ExternROSController()
{
  queue_thread_.join();
}


//   void run();

void OP3ExternROSController::initialize()
{
  cout << "--- Demo of ROBOTIS OP3 ---" << endl;
  cout << "This is op3 remote control demo" << endl;

  time_step_ms_ = getBasicTimeStep();
  time_step_sec_ = time_step_ms_ * 0.001;

  head_led_ = getLED("HeadLed");
  body_led_ = getLED("BodyLed");
  camera_ = getCamera("Camera");
  speaker_ = getSpeaker("Speaker");
  key_board_ = getKeyboard();

  camera_->enable(time_step_ms_);
  key_board_->enable(time_step_ms_);

  // initialize motors
  for (int i = 0; i < N_MOTORS; i++) {
    // get motors
    motors_[i] = getMotor(webots_joint_names[i]);

    // enable torque feedback
    motors_[i]->enableTorqueFeedback(time_step_ms_);

    // initialize encoders
    std::string sensorName = webots_joint_names[i];
    sensorName.push_back('S');
    encoders_[i] = getPositionSensor(sensorName);
    encoders_[i]->enable(time_step_ms_);
  }

  // making subscribers and ros spin
  queue_thread_ = boost::thread(boost::bind(&OP3ExternROSController::queueThread, this));
}

void OP3ExternROSController::process()
{
  getPresentJointAngles();
  getPresentJointTorques();
  getCurrentRobotCOM();

  publishPresentJointStates();

  setDesiredJointAngles();
  
  myStep();
}



void OP3ExternROSController::setDesiredJointAngles()
{
  for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
  {
    motors_[joint_idx]->setPosition(desired_joint_angle_rad_[joint_idx]);
  }
}

void OP3ExternROSController::getPresentJointAngles()
{
  for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
  {
    current_joint_angle_rad_[joint_idx] = encoders_[joint_idx]->getValue();
  }
}

void OP3ExternROSController::getPresentJointTorques()
{
  for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
  {
    current_joint_torque_Nm_[joint_idx] = motors_[joint_idx]->getTorqueFeedback();
  }
}

void OP3ExternROSController::getCurrentRobotCOM()
{
  const double* com = this->getSelf()->getCenterOfMass();

  previous_com_m_[0] = current_com_m_[0];
  previous_com_m_[1] = current_com_m_[1];
  previous_com_m_[2] = current_com_m_[2];
  
  current_com_m_[0] = com[0];
  current_com_m_[1] = com[1];
  current_com_m_[2] = com[2];

  current_com_vel_mps_[0] = (current_com_m_[0] - previous_com_m_[0]) / time_step_sec_;
  current_com_vel_mps_[1] = (current_com_m_[1] - previous_com_m_[1]) / time_step_sec_;
  current_com_vel_mps_[2] = (current_com_m_[2] - previous_com_m_[2]) / time_step_sec_;
}

void OP3ExternROSController::publishPresentJointStates()
{
  joint_state_msg_.name.clear();
  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();

  joint_state_msg_.header.stamp = ros::Time::now();
    
  for(int i = 0; i < 20; i++)
  {
    joint_state_msg_.name.push_back(op3_joint_names[i]);
    joint_state_msg_.position.push_back(current_joint_angle_rad_[i]);
    joint_state_msg_.velocity.push_back(0);
    joint_state_msg_.effort.push_back(current_joint_torque_Nm_[i]);
  }

  present_joint_state_publisher_.publish(joint_state_msg_);
}


void OP3ExternROSController::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* Publishers, Subsribers, and Service Clients */
  // make present joint state publisher
  present_joint_state_publisher_ = ros_node.advertise<sensor_msgs::JointState>("/robotis_op3/joint_states", 1);

  ros::Subscriber goal_pos_subs[20];

  for (int i = 0; i < 20; i++)
  {
    // make subscribers for the joint position topic from robotis framework
    std::string goal_pos_topic_name = "/robotis_op3/" + op3_joint_names[i] + "_position/command";
    goal_pos_subs[i] = ros_node.subscribe<std_msgs::Float64>(goal_pos_topic_name, 1, boost::bind(&OP3ExternROSController::posCommandCallback, this, boost::placeholders::_1, i));
  }

  ros::WallDuration duration(time_step_sec_);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);

}

void OP3ExternROSController::posCommandCallback(const std_msgs::Float64::ConstPtr &msg, const int &joint_idx)
{
  desired_joint_angle_rad_[joint_idx] = msg->data;
  desired_joint_angle_rcv_flag_[joint_idx] = true;
}

void OP3ExternROSController::myStep() {
  if (step(time_step_ms_) == -1)
    exit(EXIT_SUCCESS);
}