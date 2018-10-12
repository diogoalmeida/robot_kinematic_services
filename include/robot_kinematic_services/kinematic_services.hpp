#ifndef __KINEMATIC_SERVICES__
#define __KINEMATIC_SERVICES__

#include <ros/ros.h>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <robot_kinematic_services/InverseKinematics.h>
#include <robot_kinematic_services/ForwardKinematics.h>

namespace robot_kinematic_services
{
  /**
    Implements a ROS services which computes the forward and inverse kinematics of a robot described in a URDF file
    by using iterative methods.
  **/
  class KinematicServices
  {
  public:
    KinematicServices();
    ~KinematicServices();

    bool ikCallback(InverseKinematics::Request& req, InverseKinematics::Response& res);
    bool fkCallback(ForwardKinematics::Request& req, ForwardKinematics::Response& res);
    void stateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  private:
    ros::NodeHandle nh_;
    sensor_msgs::JointState state_;
    ros::Subscriber state_sub_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;

    /**
      Loads the necessary parameters to initialize the class.

      @returns False in case of error, true otherwise.
    **/
    bool loadParams();

    /**
      Processes the common elements of a kinematics request: Initializes the eef in the kdl_manager_ and
      setups a grip_point.

      @param eef The kinematic chain end-effector.
      @param grip_point An optional gripping point, assumed to be rigidly attached to the eef. The kinematic services will assume a chain ending at this point.
      @returns false in case of error, true otherwise.
    **/
    bool processRequestCommon(const std::string &eef, const std::string &grip_point);
  };
}
#endif
