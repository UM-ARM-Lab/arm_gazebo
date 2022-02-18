#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

constexpr auto const LOGGER = "val_arms";

const double KP = 50;

void print_ros_init_error() {
  ROS_FATAL(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
}

namespace gazebo {

auto min_vel(auto const &v1, auto const &v2) { return std::abs(v1) < std::abs(v2) ? v1 : v2; }

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

class ValArmsPlugin : public ModelPlugin {
 public:
  ~ValArmsPlugin() override {
    if (!plugin_loaded_) {
      ROS_DEBUG_STREAM_NAMED(LOGGER, "Destructor skipped because never loaded");
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

    nh_ = std::make_unique<ros::NodeHandle>("val_arms");

    async_spinner_ = std::make_unique<ros::AsyncSpinner>(0);  // will use a thread for each CPU core
    async_spinner_->start();
    ROS_DEBUG_NAMED(LOGGER, "Async Spinner Running");

    auto update = [&]() { Control(); };

    update_event_ = event::Events::ConnectWorldUpdateEnd(update);
    ROS_DEBUG_NAMED(LOGGER, "Connected to WorldUpdateEnd");

    SetupRosConnections();

    plugin_loaded_ = true;
    ROS_INFO_NAMED(LOGGER, "Finished loading ARM Gazebo ROS Plugin.");
  }

  void SetupRosConnections() {
    sub_arms_joint_cmd_ = nh_->subscribe<sensor_msgs::JointState>("/hdt_adroit_coms/joint_cmd", 10,
                                                                  &ValArmsPlugin::ArmsJointCmdCallback, this);
  }

  [[nodiscard]] auto GetLatestCmd() const {
    // return copies
    std::lock_guard guard(cmd_mutex);
    return latest_arms_cmd_;
  }

  void Control() {
    auto const world = parent_->GetWorld();
    if (!world) {
      ROS_WARN_NAMED(LOGGER, "parent world ptr is null");
      return;
    }
    auto const dt_s = world->Physics()->GetMaxStepSize();

    auto latest_arms_cmd = GetLatestCmd();
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
  }

  void ControlArms(sensor_msgs::JointState const &current_arms_cmd, double const dt_s) {
    for (auto i{0}; i < current_arms_cmd.name.size(); ++i) {
      auto const name = current_arms_cmd.name[i];
      auto const joint = GetJoint(LOGGER, parent_, name);
      if (!joint) {
        continue;
      }
      auto const current_position = joint->Position(0);
      // NOTE: is subtracting these angles correct? or do we need to wrap around?
      auto const position_error = current_arms_cmd.position[i] - current_position;
      // TODO: implement realistic acceleration
      auto const abs_vel = current_arms_cmd.velocity[i];
      auto const signed_vel = position_error > 0 ? abs_vel : -abs_vel;
      auto const velocity = min_vel(signed_vel, KP * position_error);
      auto const d_pos = velocity * dt_s;
      ROS_DEBUG_STREAM_NAMED(LOGGER + ".arms_cmd", "joint " << name << " vel " << velocity << " pos err "
                                                            << position_error << " dt_s " << dt_s << " dpos " << d_pos);
      joint->SetPosition(0, current_position + d_pos);
    }
  }

  void ArmsJointCmdCallback(const sensor_msgs::JointStateConstPtr &msg) {
    std::lock_guard guard(cmd_mutex);
    latest_arms_cmd_ = *msg;
  }

  bool plugin_loaded_ = false;
  std::unique_ptr<ros::NodeHandle> nh_;
  event::ConnectionPtr update_event_;
  physics::ModelPtr parent_;
  [[maybe_unused]] ros::Subscriber sub_arms_joint_cmd_;
  std::unique_ptr<ros::AsyncSpinner> async_spinner_;
  sensor_msgs::JointState latest_arms_cmd_;
  bool printed_arms_stale_debug_msg = false;

  // https://stackoverflow.com/questions/4127333/should-mutexes-be-mutable
  mutable std::mutex cmd_mutex;
};

GZ_REGISTER_MODEL_PLUGIN(ValArmsPlugin)
}  // namespace gazebo