#ifndef ARM5_SERVICES
#define ARM5_SERVICES


#include "arm5e_arm_services/ForwardKinematics.h"
#include "arm5e_arm_services/InverseKinematics.h"
#include "arm5e_arm_services/MoveArm.h"
#include "arm5e_arm_services/GetJoints.h"

#include <mar_robot_arm5e/ARM5Arm.h>

#include "ros/ros.h"

namespace arm5e_arm_services
{

class ArmServer
{
public:

  ArmServer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:

  bool fkServiceCallback(ForwardKinematicsRequest& req, ForwardKinematicsResponse& res);
  bool ikServiceCallback(InverseKinematicsRequest& req, InverseKinematicsResponse& res);
  bool moveServiceCallback(MoveArmRequest& req, MoveArmResponse& res);
  bool jointsServiceCallback(GetJointsRequest& req, GetJointsResponse& res);

  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer fk_service_server_, ik_service_server_, move_service_server_, joints_service_server_;
  ARM5Arm * robot_;
  // planner pointer * MarkerDetector detector_;
};

} // end of namespace
#endif
