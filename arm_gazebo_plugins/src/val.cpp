#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

constexpr auto const LOGGER = "val";

const double KP = 50;

void print_ros_init_error() {
  ROS_FATAL(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
}

namespace gazebo {

physics::JointPtr GetJoint(char const *plugin_name, physics::ModelPtr model, std::string const joint_name) {
  auto const scoped_model_name = model->GetScopedName();
  auto const full_joint_name = scoped_model_name + "::" + joint_name;
  physics::JointPtr joint = model->GetJoint(full_joint_name);
  if (!joint) {
    ROS_ERROR_STREAM_NAMED(plugin_name, "No joint " << full_joint_name << " found");
    ROS_WARN_STREAM_NAMED(plugin_name, "Available joints are:");
    for (const auto &j : model->GetJoints()) {
      ROS_WARN_STREAM_NAMED(plugin_name, j->GetScopedName());
    }
  }
  return joint;
}

class ValPlugin : public ModelPlugin {
 public:
  ~ValPlugin() override {
    if (!plugin_loaded_) {
      ROS_DEBUG_STREAM_NAMED(LOGGER, "Deconstructor skipped because never loaded");
      return;
    }
    update_event_.reset();
    async_spinner_->stop();
  }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override {
    parent_ = parent;
    if (!ros::isInitialized()) {
      print_ros_init_error();
      return;
    }
    ROS_DEBUG_NAMED(LOGGER, "ROS is Initialized Successfully");

    if (sdf->HasElement("robotNamespace")) {
      robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    nh_ = std::make_unique<ros::NodeHandle>("val_gazebo");

    async_spinner_ = std::make_unique<ros::AsyncSpinner>(0);  // will use a thread for each CPU core
    async_spinner_->start();
    ROS_DEBUG_NAMED(LOGGER, "Async Spinner Running");

    auto update = [&]() {
      PublishJointStates();
      Control();
    };

    update_event_ = event::Events::ConnectWorldUpdateEnd(update);
    ROS_DEBUG_NAMED(LOGGER, "Connected to WorldUpdateEnd");

    SetupRosConnections();

    plugin_loaded_ = true;
    ROS_INFO_NAMED(LOGGER, "Finished loading ARM Gazebo ROS Plugin.");
  }

  void SetupRosConnections() {
    auto const joint_states_topic = ros::names::append(robot_namespace_, "joint_states");
    pub_joint_states_ = nh_->advertise<sensor_msgs::JointState>(joint_states_topic, 10);
    sub_arms_joint_cmd_ = nh_->subscribe<sensor_msgs::JointState>("/hdt_adroit_coms/joint_cmd", 10,
                                                                  &ValPlugin::ArmsJointCmdCallback, this);
    sub_base_cmd_vel_ = nh_->subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &ValPlugin::BaseCmdVelCallback, this);
  }

  void PublishJointStates() {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    // TODO: the joints should come from the URDF transmission elements
    std::vector<std::string> joint_names{
        "front_left_wheel", "front_right_wheel",
        "rear_left_wheel",  "rear_right_wheel",
        "joint56",          "joint57",
        "joint41",          "joint42",
        "joint43",          "joint44",
        "joint45",          "joint46",
        "joint47",          "leftgripper",
        "leftgripper2",     "joint1",
        "joint2",           "joint3",
        "joint4",           "joint5",
        "joint6",           "joint7",
        "rightgripper",     "rightgripper2",
    };
    for (auto const &joint_name : joint_names) {
      auto const joint = GetJoint(LOGGER, parent_, joint_name);
      if (!joint) {
        continue;
      }
      auto const position = joint->Position();
      auto const velocity = joint->GetVelocity(0);
      joint_state.name.push_back(joint_name);
      joint_state.position.push_back(position);
      joint_state.velocity.push_back(velocity);
      joint_state.effort.push_back(0);
    }
    pub_joint_states_.publish(joint_state);
  }

  [[nodiscard]] auto GetLatestCmd() const {
    // return copies
    std::lock_guard guard(cmd_mutex);
    return std::tuple{latest_arms_cmd_, latest_base_cmd_};
  }

  void Control() {
    auto const world = parent_->GetWorld();
    if (!world) {
      ROS_WARN_NAMED(LOGGER, "parent world ptr is null");
      return;
    }
    auto const dt_s = world->Physics()->GetMaxStepSize();

    auto [latest_arms_cmd, latest_base_cmd] = GetLatestCmd();
    // if latest command is out of date, use a zero velocity command to mimic real world behavior
    auto const now = ros::Time::now();
    ros::Duration stale_duration(0.1);
    if (latest_arms_cmd.header.stamp + stale_duration < now) {
      if (not printed_arms_stale_debug_msg) {
        ROS_DEBUG_STREAM_NAMED(LOGGER + ".stale", "arms command is stale");
        printed_arms_stale_debug_msg = true;
      }
      latest_arms_cmd.velocity = std::vector<double>(latest_arms_cmd_.velocity.size(), 0);
    } else {
      printed_arms_stale_debug_msg = false;
      ControlArms(latest_arms_cmd, dt_s);
    }

    if (latest_base_cmd.header.stamp + stale_duration < now) {
      if (not printed_base_stale_debug_msg) {
        ROS_DEBUG_STREAM_NAMED(LOGGER + ".stale", "base command is stale");
        printed_base_stale_debug_msg = true;
      }
      latest_base_cmd.twist.linear.x = 0;
      latest_base_cmd.twist.angular.z = 0;
    } else {
      printed_base_stale_debug_msg = false;
      ControlBase(latest_base_cmd, dt_s);
    }
  }

  void ControlArms(sensor_msgs::JointState const &current_arms_cmd, double const dt_s) {
    for (auto i{0}; i < current_arms_cmd.name.size(); ++i) {
      auto const name = current_arms_cmd.name[i];
      auto const joint = GetJoint(LOGGER, parent_, name);
      if (!joint) {
        continue;
      }
      auto const current_position = joint->Position(0);
      auto const position_error = current_arms_cmd.position[i] - current_position;
      // TODO implement realistic acceleration
      auto const velocity = std::min(current_arms_cmd.velocity[i], KP * position_error);
      ROS_DEBUG_STREAM_NAMED(LOGGER + ".arms_cmd",
                             "joint " << name << " vel " << velocity << " pos err " << position_error);
      joint->SetPosition(0, current_position + velocity * dt_s);
    }
  }

  void ControlBase(geometry_msgs::TwistStamped const &current_base_cmd, double const dt_s) {
    // TODO implement realistic acceleration
    auto current_pose = parent_->WorldPose();
    auto new_pose = current_pose;
    auto const dlinear = current_base_cmd.twist.linear.x * dt_s;
    auto const dyaw = current_base_cmd.twist.angular.z * dt_s;
    auto const current_yaw = current_pose.Rot().Yaw();
    auto const dx_world = cos(current_yaw) * dlinear;
    auto const dy_world = sin(current_yaw) * dlinear;
    new_pose.Pos() += ignition::math::Vector3d(dx_world, dy_world, 0);
    new_pose.Rot().Euler(0, 0, new_pose.Rot().Yaw() + dyaw);
    parent_->SetWorldPose(new_pose);
  }

  void ArmsJointCmdCallback(const sensor_msgs::JointStateConstPtr &msg) {
    std::lock_guard guard(cmd_mutex);
    latest_arms_cmd_ = *msg;
  }

  void BaseCmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
    std::lock_guard guard(cmd_mutex);
    latest_base_cmd_.twist = *msg;
    latest_base_cmd_.header.stamp = ros::Time::now();
  }

  bool plugin_loaded_ = false;
  std::unique_ptr<ros::NodeHandle> nh_;
  event::ConnectionPtr update_event_;
  physics::ModelPtr parent_;
  ros::Publisher pub_joint_states_;
  [[maybe_unused]] ros::Subscriber sub_arms_joint_cmd_;
  [[maybe_unused]] ros::Subscriber sub_base_cmd_vel_;
  std::string robot_namespace_;
  std::unique_ptr<ros::AsyncSpinner> async_spinner_;
  sensor_msgs::JointState latest_arms_cmd_;
  geometry_msgs::TwistStamped latest_base_cmd_;
  bool printed_arms_stale_debug_msg = false;
  bool printed_base_stale_debug_msg = false;

  // https://stackoverflow.com/questions/4127333/should-mutexes-be-mutable
  mutable std::mutex cmd_mutex;
};

GZ_REGISTER_MODEL_PLUGIN(ValPlugin)
}  // namespace gazebo