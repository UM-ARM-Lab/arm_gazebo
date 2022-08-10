#include <arm_gazebo_msgs/GetJointStates.h>
#include <arm_gazebo_msgs/GetWorldInitialSDF.h>
#include <arm_gazebo_msgs/SetJointStates.h>
#include <arm_gazebo_msgs/SetLinkStates.h>
#include <arm_gazebo_msgs/SetModelStates.h>
#include <arm_gazebo_msgs/WorldControl.h>
#include <geometry_msgs/Vector3.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

constexpr auto const LOGGER = "arm_gazebo_ros_plugin";

// this is a free function because
// (1) to remove a clang-tidy "complexity" warning
// (2) because the string is long and doesn't look good when 3 levels on indent int
void print_ros_init_error() {
  ROS_FATAL(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
}

namespace gazebo {

class ArmGazeboRosPlugin : public WorldPlugin {
 public:
  ~ArmGazeboRosPlugin() override {
    // Don't attempt to unload this plugin if it was never loaded in the Load() function
    if (!plugin_loaded_) {
      ROS_DEBUG_STREAM_NAMED(LOGGER, "Deconstructor skipped because never loaded");
      return;
    }

    pub_joint_states_event_.reset();

    async_spinner_->stop();
  }

  void Load(physics::WorldPtr parent, sdf::ElementPtr /*_sdf*/) override {
    if (!ros::isInitialized()) {
      print_ros_init_error();
      return;
    }
    ROS_DEBUG_NAMED(LOGGER, "ROS is Initialized Successfully");

    world_ = parent;

    world_initial_sdf_string_ = world_->SDF()->ToString("");

    nh_ = std::make_unique<ros::NodeHandle>("arm_gazebo");

    async_spinner_ = std::make_unique<ros::AsyncSpinner>(0);  // will use a thread for each CPU core
    async_spinner_->start();
    ROS_DEBUG_NAMED(LOGGER, "Async Spinner Running");

    seconds_per_step_ = parent->Physics()->GetMaxStepSize();

    transport::NodePtr node(new transport::Node());
    node->Init(parent->Name());
    world_control_pub_ = node->Advertise<msgs::WorldControl>("~/world_control");
    ROS_DEBUG_NAMED(LOGGER, "Created gz transport publisher");

    auto update = [&]() {
      PublishJointStates();

      // for stepping
      if (step_count_ > 0) {
        --step_count_;
      }
    };
    pub_joint_states_event_ = event::Events::ConnectWorldUpdateEnd(update);
    ROS_DEBUG_NAMED(LOGGER, "Connected to WorldUpdateEnd");

    SetupRosConnections();

    plugin_loaded_ = true;
    ROS_INFO_NAMED(LOGGER, "Finished loading ARM Gazebo ROS Plugin.");
  }

  void SetupRosConnections() {
    pub_joint_states_ = nh_->advertise<arm_gazebo_msgs::JointStates>("joint_states", 10);

    set_model_states_sv_ = nh_->advertiseService("set_model_states", &ArmGazeboRosPlugin::SetModelStates, this);
    get_joint_states_sv_ = nh_->advertiseService("get_joint_state", &ArmGazeboRosPlugin::GetJointStates, this);
    set_joint_states_sv_ = nh_->advertiseService("set_joint_states", &ArmGazeboRosPlugin::SetJointStates, this);
    set_link_states_sv_ = nh_->advertiseService("set_link_states", &ArmGazeboRosPlugin::SetLinkStates, this);
    step_service_ = nh_->advertiseService("world_control", &ArmGazeboRosPlugin::OnWorldControl, this);
    world_initial_sdf_service_ =
        nh_->advertiseService("world_initial_sdf", &ArmGazeboRosPlugin::OnWorldInitialSDF, this);
  }

  bool SetLinkStates(arm_gazebo_msgs::SetLinkStates::Request &req,  // NOLINT(readability-make-member-function-const)
                     arm_gazebo_msgs::SetLinkStates::Response &res) {
    for (auto const &req_link_state : req.link_states) {
      auto const link = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(req_link_state.link_name));
      auto const frame =
          boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(req_link_state.reference_frame));
      if (!link) {
        ROS_ERROR_NAMED(LOGGER, "Updating LinkState: link [%s] does not exist", req_link_state.link_name.c_str());
        res.success = false;
        res.status_message = "SetLinkState: link does not exist";
        return true;
      }

      // get reference frame (link/model(link)) pose and
      // transform target pose to absolute world frame
      ignition::math::Vector3d target_pos(req_link_state.pose.position.x, req_link_state.pose.position.y,
                                          req_link_state.pose.position.z);
      ignition::math::Quaterniond target_rot(req_link_state.pose.orientation.w, req_link_state.pose.orientation.x,
                                             req_link_state.pose.orientation.y, req_link_state.pose.orientation.z);
      ignition::math::Pose3d target_pose(target_pos, target_rot);
      ignition::math::Vector3d target_linear_vel(req_link_state.twist.linear.x, req_link_state.twist.linear.y,
                                                 req_link_state.twist.linear.z);
      ignition::math::Vector3d target_angular_vel(req_link_state.twist.angular.x, req_link_state.twist.angular.y,
                                                  req_link_state.twist.angular.z);

      if (frame) {
        auto const frame_pose = frame->WorldPose();
        auto const frame_linear_vel = frame->WorldLinearVel();
        auto const frame_angular_vel = frame->WorldAngularVel();
        auto const &frame_pos = frame_pose.Pos();
        auto const &frame_rot = frame_pose.Rot();

        target_pose = target_pose + frame_pose;

        target_linear_vel -= frame_linear_vel;
        target_angular_vel -= frame_angular_vel;
      } else if (req_link_state.reference_frame.empty() || req_link_state.reference_frame == "world") {
        ROS_INFO_NAMED(LOGGER, "Updating LinkState: reference_frame is empty/world, using inertial frame");
      } else {
        ROS_ERROR_NAMED(LOGGER, "Updating LinkState: reference_frame is not a valid entity name");
        res.success = false;
        res.status_message = "failed";
        return true;
      }

      bool is_paused = world_->IsPaused();
      world_->SetPaused(true);

      link->SetWorldPose(target_pose);
      link->SetLinearVel(target_linear_vel);
      link->SetAngularVel(target_angular_vel);

      world_->SetPaused(is_paused);
    }

    res.success = true;
    res.status_message = "success";
    return true;
  }

  bool SetModelStates(arm_gazebo_msgs::SetModelStates::Request &req,  // NOLINT(readability-make-member-function-const)
                      arm_gazebo_msgs::SetModelStates::Response &res) {
    for (auto const &req_model_state : req.model_states) {
      ignition::math::Vector3d target_pos(req_model_state.pose.position.x, req_model_state.pose.position.y,
                                          req_model_state.pose.position.z);
      ignition::math::Quaterniond target_rot(req_model_state.pose.orientation.w, req_model_state.pose.orientation.x,
                                             req_model_state.pose.orientation.y, req_model_state.pose.orientation.z);
      target_rot.Normalize();  // eliminates invalid rotation (0, 0, 0, 0)
      ignition::math::Pose3d target_pose(target_pos, target_rot);
      ignition::math::Vector3d target_pos_dot(req_model_state.twist.linear.x, req_model_state.twist.linear.y,
                                              req_model_state.twist.linear.z);
      ignition::math::Vector3d target_rot_dot(req_model_state.twist.angular.x, req_model_state.twist.angular.y,
                                              req_model_state.twist.angular.z);

      auto model = world_->ModelByName(req_model_state.model_name);
      if (!model) {
        ROS_WARN_NAMED(LOGGER, "Updating ModelState: model [%s] does not exist", req_model_state.model_name.c_str());
        res.success = false;
        res.status_message = "SetModelState: model does not exist";
        continue;
      } else {
        auto const relative_entity = world_->EntityByName(req_model_state.reference_frame);
        if (relative_entity) {
          auto const frame_pose = relative_entity->WorldPose();

          target_pose = target_pose + frame_pose;

          // Velocities should be commanded in the requested reference
          // frame, so we need to translate them to the world frame
          target_pos_dot = frame_pose.Rot().RotateVector(target_pos_dot);
          target_rot_dot = frame_pose.Rot().RotateVector(target_rot_dot);
        } else if (req_model_state.reference_frame.empty() || req_model_state.reference_frame == "world") {
          ROS_DEBUG_NAMED(LOGGER, "Updating ModelState: reference frame is empty/world, using inertial frame");
        } else {
          ROS_ERROR_NAMED(LOGGER,
                          "Updating ModelState: for model[%s], specified reference frame entity [%s] does not exist",
                          req_model_state.model_name.c_str(), req_model_state.reference_frame.c_str());
          res.success = false;
          res.status_message = "SetModelState: specified reference frame entity does not exist";
          return true;
        }

        // save current pause state so we can restore it
        auto const is_paused = world_->IsPaused();
        world_->SetPaused(true);

        ROS_DEBUG_STREAM_NAMED(LOGGER, "Setting State for model " << model->GetName());
        model->SetWorldPose(target_pose);
        model->SetLinearVel(target_pos_dot);
        model->SetAngularVel(target_rot_dot);

        world_->SetPaused(is_paused);
      }
    }

    res.success = true;
    res.status_message = "SetModelState: set model state done";

    return true;
  }

  bool GetJointStates(arm_gazebo_msgs::GetJointStates::Request &req,  // NOLINT(readability-make-member-function-const)
                      arm_gazebo_msgs::GetJointStates::Response &res) {
    arm_gazebo_msgs::JointStates joint_states_msg = getJointStatesMsg();
    res.joint_states = joint_states_msg;
    return true;
  }

  bool SetJointStates(arm_gazebo_msgs::SetJointStates::Request &req,  // NOLINT(readability-make-member-function-const)
                      arm_gazebo_msgs::SetJointStates::Response &res) {
    // NOT IMPLEMENTED;
    return false;
  }

  void PublishJointStates() const {
    arm_gazebo_msgs::JointStates joint_states_msg = getJointStatesMsg();
    pub_joint_states_.publish(joint_states_msg);
  }

  [[nodiscard]] arm_gazebo_msgs::JointStates getJointStatesMsg() const {
    arm_gazebo_msgs::JointStates joint_states_msg;
    for (auto const &model : world_->Models()) {
      for (auto const &joint : model->GetJoints()) {
        if (joint->DOF() == 0) {
          continue;
        }
        arm_gazebo_msgs::JointState joint_state_msg;
        joint_state_msg.joint_name = joint->GetName();
        joint_state_msg.model_name = model->GetName();
        for (auto axis = 0u; axis <= joint->DOF(); ++axis) {
          arm_gazebo_msgs::JointAxisState joint_axis_state;
          joint_axis_state.axis = axis;
          joint_axis_state.position = joint->Position(axis);
          joint_state_msg.states.push_back(joint_axis_state);
        }
        joint_states_msg.joint_states.push_back(joint_state_msg);
      }
    }
    return joint_states_msg;
  }

  bool OnWorldControl(arm_gazebo_msgs::WorldControlRequest &req, arm_gazebo_msgs::WorldControlResponse &res) {
    (void)res;
    auto const steps = [&]() {
      if (req.seconds > 0) {
        return static_cast<unsigned int>(req.seconds / seconds_per_step_);
      } else {
        return req.steps;
      }
    }();
    // TODO: why not use world_->Step() ?!?!?!
    step_count_ = steps;
    msgs::WorldControl gz_msg;
    gz_msg.set_multi_step(steps);
    world_control_pub_->Publish(gz_msg);
    while (step_count_ != 0)
      ;
    return true;
  }

  bool OnWorldInitialSDF(arm_gazebo_msgs::GetWorldInitialSDFRequest &req,
                         arm_gazebo_msgs::GetWorldInitialSDFResponse &res) {
    res.world_initial_sdf = world_initial_sdf_string_;
    return true;
  }

  bool plugin_loaded_ = false;

  std::unique_ptr<ros::NodeHandle> nh_;

  physics::WorldPtr world_;
  event::ConnectionPtr pub_joint_states_event_;

  [[maybe_unused]] ros::ServiceServer set_model_states_sv_;
  [[maybe_unused]] ros::ServiceServer get_joint_states_sv_;
  [[maybe_unused]] ros::ServiceServer set_joint_states_sv_;
  [[maybe_unused]] ros::ServiceServer set_link_states_sv_;
  [[maybe_unused]] ros::ServiceServer step_service_;
  [[maybe_unused]] ros::ServiceServer world_initial_sdf_service_;
  ros::Publisher pub_joint_states_;

  std::unique_ptr<ros::AsyncSpinner> async_spinner_;

  transport::PublisherPtr world_control_pub_;

  std::string world_initial_sdf_string_;
  std::atomic<int> step_count_{0};
  double seconds_per_step_{0.0};
};

GZ_REGISTER_WORLD_PLUGIN(ArmGazeboRosPlugin)
}  // namespace gazebo