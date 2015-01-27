#include "arm5e_arm_services/arm5e_service_server.h"

#include <sensor_msgs/JointState.h>

arm5e_arm_services::ArmServer::ArmServer(
  ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh), nh_private_(nh_private)
{

  fk_service_server_ = nh_private_.advertiseService("forward_kinematics", &ArmServer::fkServiceCallback, this);
  ROS_INFO("Service \"forwards_kinematics\" advertised.");
  ik_service_server_ = nh_private_.advertiseService("inverse_kinematics", &ArmServer::ikServiceCallback, this);
  ROS_INFO("Service \"inverse_kinematics\" advertised.");
  move_service_server_ = nh_private_.advertiseService("move_arm", &ArmServer::moveServiceCallback, this);
  ROS_INFO("Service \"move_arm\" advertised.");
  joints_service_server_ = nh_private_.advertiseService("get_joints", &ArmServer::jointsServiceCallback, this);
  ROS_INFO("Service \"get_joints\" advertised.");
  robot_ = new ARM5Arm(nh_private, "/uwsim/joint_state", "/uwsim/joint_state_command");
}

bool arm5e_arm_services::ArmServer::fkServiceCallback(
    ForwardKinematicsRequest& req,
    ForwardKinematicsResponse& res)
{
  vpColVector joints(5);
  joints[0] = req.in_joints[0];
  joints[1] = req.in_joints[1];
  joints[2] = req.in_joints[2];
  joints[3] = req.in_joints[3];
  joints[4] = req.in_joints[4];/// @TODO from gripper aperture parameter
  vpHomogeneousMatrix bMe = robot_->directKinematics(joints);
  geometry_msgs::Pose p;
  p.position.x=bMe[0][3];
  p.position.y=bMe[1][3];
  p.position.z=bMe[2][3];
  vpQuaternionVector q; bMe.extract(q);
  p.orientation.x=q.x();
  p.orientation.y=q.y();
  p.orientation.z=q.z();
  p.orientation.w=q.w();
  res.bMe=p;
  return true;
}
bool arm5e_arm_services::ArmServer::ikServiceCallback(
    InverseKinematicsRequest& req,
    InverseKinematicsResponse& res)
{
  vpTranslationVector t(req.desired_bMe.position.x,req.desired_bMe.position.y,req.desired_bMe.position.z);
  vpQuaternionVector q(req.desired_bMe.orientation.x, req.desired_bMe.orientation.y,
                        req.desired_bMe.orientation.z, req.desired_bMe.orientation.w);
  vpHomogeneousMatrix bMe(t,q);
  vpColVector joints(5);
  joints = robot_->armIK(bMe);
  res.out_joints.push_back(joints[0]);
  res.out_joints.push_back(joints[1]);
  res.out_joints.push_back(joints[2]);
  res.out_joints.push_back(joints[3]);
  res.out_joints.push_back(joints[4]);
  geometry_msgs::Pose p;
  p.position.x=bMe[0][3];
  p.position.y=bMe[1][3];
  p.position.z=bMe[2][3];
  bMe.extract(q);
  p.orientation.x=q.x();
  p.orientation.y=q.y();
  p.orientation.z=q.z();
  p.orientation.w=q.w();
  res.bMe=p;
  res.found=true;//FIX-ME
  return true;
}
bool arm5e_arm_services::ArmServer::moveServiceCallback(
    MoveArmRequest& req,
    MoveArmResponse& res)
{
  //get req
  //CARTESIAN
  //vpColVector xdot(6);
  //robot_->setCartesianVelocity(xdot);
  //JOINT SPACE
  vpColVector joints(5);
  joints[0] = req.in_vel[0];
  joints[1] = req.in_vel[1];
  joints[2] = req.in_vel[2];
  joints[3] = req.in_vel[3];
  joints[4] = req.in_vel[4];
  robot_->setJointVelocity(joints);
  return true;
}
bool arm5e_arm_services::ArmServer::jointsServiceCallback(
    GetJointsRequest& req,
    GetJointsResponse& res)
{
  vpColVector jstate(5);
  robot_->getJointValues(jstate);//getPosition para no tener que hacer FK(current)
  sensor_msgs::JointState js;
  js.name.push_back(std::string("Slew"));
  js.position.push_back(jstate[0]);
  js.name.push_back(std::string("Shoulder"));
  js.position.push_back(jstate[1]);
  js.name.push_back(std::string("Elbow"));
  js.position.push_back(jstate[2]);
  js.name.push_back(std::string("JawRotate"));
  js.position.push_back(jstate[3]);
  js.name.push_back(std::string("JawOpening"));
  js.position.push_back(jstate[4]);
  res.joints=js;
  return true;
}



int main(int argc, char **argv){
  ros::init(argc, argv, "arm5_grasp_exec");
  ros::NodeHandle nh;
  arm5e_arm_services::ArmServer a(nh,nh);
  ros::spin();
  return 0;
}
