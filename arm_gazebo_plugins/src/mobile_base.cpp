#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

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

class MobileBasePlugin : public ModelPlugin {
 public:
  ~MobileBasePlugin() override {
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

    nh_ = std::make_unique<ros::NodeHandle>("mobile_base");

    async_spinner_ = std::make_unique<ros::AsyncSpinner>(0);  // will use a thread for each CPU core
    async_spinner_->start();
    ROS_DEBUG_NAMED(LOGGER, "Async Spinner Running");

    auto update = [&]() {
      Control();
    };

    update_event_ = event::Events::ConnectWorldUpdateEnd(update);
    ROS_DEBUG_NAMED(LOGGER, "Connected to WorldUpdateEnd");

    SetupRosConnections();

    plugin_loaded_ = true;
    ROS_INFO_NAMED(LOGGER, "Finished loading ARM Gazebo ROS Plugin.");
  }

  void SetupRosConnections() {
    sub_base_cmd_vel_ = nh_->subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &MobileBasePlugin::BaseCmdVelCallback, this);
  }

  [[nodiscard]] auto GetLatestCmd() const {
    // return copies
    std::lock_guard guard(cmd_mutex);
    return latest_base_cmd_;
  }

  void Control() {
    auto const world = parent_->GetWorld();
    if (!world) {
      ROS_WARN_NAMED(LOGGER, "parent world ptr is null");
      return;
    }
    auto const dt_s = world->Physics()->GetMaxStepSize();

    auto latest_base_cmd = GetLatestCmd();
    // if latest command is out of date, use a zero velocity command to mimic real world behavior
    auto const now = ros::Time::now();
    ros::Duration stale_duration(0.1);

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

  void BaseCmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
    std::lock_guard guard(cmd_mutex);
    latest_base_cmd_.twist = *msg;
    latest_base_cmd_.header.stamp = ros::Time::now();
  }

  bool plugin_loaded_ = false;
  std::unique_ptr<ros::NodeHandle> nh_;
  event::ConnectionPtr update_event_;
  physics::ModelPtr parent_;
  [[maybe_unused]] ros::Subscriber sub_base_cmd_vel_;
  std::unique_ptr<ros::AsyncSpinner> async_spinner_;
  geometry_msgs::TwistStamped latest_base_cmd_;
  bool printed_base_stale_debug_msg = false;

  // https://stackoverflow.com/questions/4127333/should-mutexes-be-mutable
  mutable std::mutex cmd_mutex;
};

GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}  // namespace gazebo