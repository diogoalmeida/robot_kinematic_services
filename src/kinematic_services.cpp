#include <robot_kinematic_services/kinematic_services.hpp>

namespace robot_kinematic_services
{
  KinematicServices::KinematicServices() : nh_("~")
  {
    if (!loadParams())
    {
      throw std::runtime_error("ERROR getting Kinematic services parameters");
    }

    state_sub_ = nh_.subscribe("/joint_states", 1000, &KinematicServices::stateCallback, this);
  }

  KinematicServices::~KinematicServices() {}

  bool KinematicServices::loadParams()
  {
    std::string base_link;
    if (!nh_.getParam("robot_chain_base_link", base_link))
    {
      ROS_ERROR("Missing robot_chain_base_link parameter");
      return false;
    }

    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_link, nh_);
    return true;
  }

  bool KinematicServices::ikCallback(InverseKinematics::Request& req, InverseKinematics::Response& res)
  {
    if (!processRequestCommon(req.chain_end_effector_name, req.tooltip_name))
    {
      res.status.code = res.status.INPUT_FAILURE;
      return true;
    }

    KDL::Frame pose;
    KDL::JntArray ik_solution;

    sensor_msgs::JointState state = state_;
    while(!kdl_manager_->checkStateMessage(req.chain_end_effector_name, state))
    {
      if (!ros::ok())
      {
        return false;
      }

      ROS_INFO_THROTTLE(1, "Waiting for a valid joint state message...");
      state = state_;
      ros::Duration(0.01).sleep();
    }

    ROS_INFO_STREAM("Got a valid joint state for requested end-effector chain: " << req.chain_end_effector_name << ". Processing...");

    // if pose request frame != chain base link, convert pose to chain base link

    tf::poseMsgToKDL(req.desired_pose.pose, pose);
    if (!kdl_manager_->getGrippingPoseIK(req.chain_end_effector_name, state, pose, ik_solution))
    {
      res.status.code = res.status.IK_FAILURE;
      return true;
    }

    for (int i = 0; i < ik_solution.rows(); i++)
    {
      res.ik_solution.push_back(ik_solution(i));
    }

    res.status.code = res.status.SUCCESS;
    return true;
  }

  bool KinematicServices::fkCallback(ForwardKinematics::Request& req, ForwardKinematics::Response& res)
  {
    if (!processRequestCommon(req.chain_end_effector_name, req.tooltip_name))
    {
      res.status.code = res.status.INPUT_FAILURE;
      return true;
    }


    KDL::Frame pose;
    if (!kdl_manager_->getGrippingPoint(req.chain_end_effector_name, req.state, pose))
    {
      res.status.code = res.status.IK_FAILURE;
      return true;
    }

    tf::poseKDLToMsg(pose, res.fk_solution.pose);
    res.fk_solution.header.frame_id = req.tooltip_name;
    res.fk_solution.header.stamp = ros::Time::now();
    res.status.code = res.status.SUCCESS;
    return true;
  }

  bool KinematicServices::processRequestCommon(const std::string &eef, const std::string &grip_point)
  {
    if (!kdl_manager_->isInitialized(eef))
    {
      if (!kdl_manager_->initializeArm(eef))
      {
        return false;
      }
    }

    if (eef != grip_point && !grip_point.empty())
    {
      if(!kdl_manager_->setGrippingPoint(eef, grip_point))
      {
        return false;
      }
    }
    else
    {
      kdl_manager_->setGrippingPoint(eef, eef); // we know the eef is valid at this point.
    }

    return true;
  }

  void KinematicServices::stateCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    state_ = *msg;
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "kinematic_services");
  ros::NodeHandle nh;
  robot_kinematic_services::KinematicServices service_handler;

  ros::ServiceServer ik_server = nh.advertiseService("/compute_ik", &robot_kinematic_services::KinematicServices::ikCallback, &service_handler);
  ros::ServiceServer fk_server = nh.advertiseService("/compute_fk", &robot_kinematic_services::KinematicServices::fkCallback, &service_handler);

  ROS_INFO("Kinematic services node is ready");
  ros::MultiThreadedSpinner spinner(3); // so that we can process state message callbacks while waiting in a service callback
  spinner.spin();
}
