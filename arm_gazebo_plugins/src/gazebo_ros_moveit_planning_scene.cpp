/* * Publishes the gazebo world state as a MoveIt! planning scene */

#include <arm_gazebo_msgs/ExcludeModels.h>
#include <arm_gazebo_plugins/gazebo_ros_moveit_planning_scene.h>

#include <algorithm>
#include <arc_utilities/ostream_operators.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <mutex>

namespace gazebo {
static std::string get_id(const physics::LinkPtr &link) { return link->GetModel()->GetName() + "." + link->GetName(); }

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMoveItPlanningScene);

constexpr auto LOGNAME = "GazeboRosMoveItPlanningScene";

GazeboRosMoveItPlanningScene::~GazeboRosMoveItPlanningScene() {
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

// Load the controller
void GazeboRosMoveItPlanningScene::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->model_ = _model;
  this->world_ = _model->GetWorld();
  this->model_name_ = _model->GetName();

  {
    // load parameters
    if (_sdf->HasElement("robotNamespace")) {
      this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    } else {
      this->robot_namespace_ = "";
    }

    if (!_sdf->HasElement("robotName")) {
      this->robot_name_ = _model->GetName();
    } else {
      this->robot_name_ = _sdf->GetElement("robotName")->Get<std::string>();
    }

    if (!_sdf->HasElement("sceneName")) {
      this->scene_name_ = "";
    } else {
      this->scene_name_ = _sdf->GetElement("sceneName")->Get<std::string>();
    }

    if (!_sdf->HasElement("frameId")) {
      this->frame_id_ = "world";
    } else {
      this->frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();
    }

    if (_sdf->HasElement("scalePrimitivesFactor")) {
      this->scale_primitives_factor_ = _sdf->GetElement("scalePrimitivesFactor")->Get<double>();
    }

    excluded_model_names_.emplace_back(model_name_);
    if (_sdf->HasElement("excludedModels")) {
      auto const excluded_models_str = _sdf->GetElement("excludedModels")->Get<std::string>();
      boost::split(excluded_model_names_, excluded_models_str, boost::is_any_of(" ,\n"));
    }
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "Excluded models " << excluded_model_names_);
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_.reset(new ros::NodeHandle(this->robot_namespace_));

  // Publish the planning scene
  planning_scene_pub_ = rosnode_->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  auto exclude_bind = [this](arm_gazebo_msgs::ExcludeModelsRequest &req, arm_gazebo_msgs::ExcludeModelsResponse &res) {
    for (auto const &name : req.model_names) {
      if (not name.empty()) {
        excluded_model_names_.push_back(name);
      }
    }
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Updated excluded models " << excluded_model_names_);

    // copies into the response
    res.all_model_names = excluded_model_names_;
    return true;
  };
  auto exclude_so = ros::AdvertiseServiceOptions::create<arm_gazebo_msgs::ExcludeModels>(
      "exclude_models_from_planning_scene", exclude_bind, ros::VoidPtr(), &queue_);
  excluded_models_srv_ = rosnode_->advertiseService(exclude_so);

  // Custom Callback Queue for services
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosMoveItPlanningScene::QueueThread, this));

  auto periodic_update = [this]() {
    while (ros::ok()) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(0.01s);
      PeriodicUpdate();
    }
  };
  periodic_event_thread_ = std::thread(periodic_update);
}

void GazeboRosMoveItPlanningScene::PeriodicUpdate() {
  std::lock_guard const lock(ros_mutex_);
  auto const scene = BuildMessage();
  planning_scene_pub_.publish(scene);
}

moveit_msgs::PlanningScene GazeboRosMoveItPlanningScene::BuildMessage() {
  using namespace gazebo::common;
  using namespace gazebo::physics;

  moveit_msgs::PlanningScene planning_scene_msg;
  planning_scene_msg.name = scene_name_;
  planning_scene_msg.robot_model_name = robot_name_;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.collision_objects.clear();
  std::vector<ModelPtr> models = this->world_->Models();

  // Iterate through the tracked models and clear their dynamic information
  // This also sets objects to be removed if they currently aren't in the scene
  for (auto &[name, object] : collision_object_map_) {
    // Mark for removal unless it's found in the gazebo world
    object.operation = moveit_msgs::CollisionObject::REMOVE;

    // Clear shape information
    object.primitives.clear();
    object.primitive_poses.clear();

    object.meshes.clear();
    object.mesh_poses.clear();

    object.planes.clear();
    object.plane_poses.clear();
  }

  for (auto model_it = models.cbegin(); model_it != models.end(); ++model_it) {
    const ModelPtr &model = *model_it;
    const std::string model_name = model->GetName();

    // Don't declare collision objects for the robot links
    if (model_name == model_name_) {
      continue;
    }

    // Skip some models
    if (std::find(excluded_model_names_.cbegin(), excluded_model_names_.cend(), model_name) !=
        excluded_model_names_.cend()) {
      ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "Skipping model " << model_name);
      continue;
    }

    // Iterate over all links in the model, and add collision objects from each one
    // This adds meshes and primitives to:
    //  object.meshes,
    //  object.primitives, and
    //  object.planes
    std::vector<LinkPtr> links = model->GetLinks();
    for (auto link_it = links.cbegin(); link_it != links.end(); ++link_it) {
      const LinkPtr &link = *link_it;
      const std::string id = get_id(link);

      // Get all the collision objects for this link
      const auto &collisions = link->GetCollisions();
      if (collisions.empty()) {
        // we get an error if we try to publish this object, so skip it
        continue;
      }

      // Check if the gazebo model is already in the collision object map
      auto found_collision_object = collision_object_map_.find(id);

      // Create a new collision object representing this link if it's not in the map
      if (found_collision_object == collision_object_map_.end()) {
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Creating collision object for model "
                                            << model_name << " model_name " << model_name_
                                            << " n objects = " << collision_object_map_.size());

        moveit_msgs::CollisionObject new_object;
        new_object.id = id;
        new_object.header.frame_id = frame_id_;
        new_object.operation = moveit_msgs::CollisionObject::ADD;

        collision_object_map_[id] = new_object;
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "Adding object: " << id);
      } else {
        collision_object_map_[id].operation = moveit_msgs::CollisionObject::MOVE;
        ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "Moving object: " << id);
      }

      // Get a reference to the object from the map
      moveit_msgs::CollisionObject &object = collision_object_map_[id];
      object.id = id;
      object.header.frame_id = this->frame_id_;
      object.header.stamp = ros::Time::now();
      object.pose.orientation.w = 1.0;
      object.operation = moveit_msgs::CollisionObject::ADD;

      for (const auto &collision : collisions) {
        const ShapePtr shape = collision->GetShape();

        auto const world2collision = collision->WorldPose();
        auto const world2robot = model_->WorldPose();
        auto const robot2world = world2robot.Inverse();
        auto const robot2collision = world2collision + robot2world;
        geometry_msgs::Pose collision_pose_msg;
        {
          collision_pose_msg.position.x = robot2collision.Pos().X();
          collision_pose_msg.position.y = robot2collision.Pos().Y();
          collision_pose_msg.position.z = robot2collision.Pos().Z();
          collision_pose_msg.orientation.x = robot2collision.Rot().X();
          collision_pose_msg.orientation.y = robot2collision.Rot().Y();
          collision_pose_msg.orientation.z = robot2collision.Rot().Z();
          collision_pose_msg.orientation.w = robot2collision.Rot().W();
        }

        // Always add pose information
        if (shape->HasType(Base::MESH_SHAPE)) {
          boost::shared_ptr<MeshShape> mesh_shape = boost::dynamic_pointer_cast<MeshShape>(shape);
          std::string uri = mesh_shape->GetMeshURI();
          const Mesh *mesh = MeshManager::Instance()->GetMesh(uri);
          if (!mesh) {
            ROS_WARN_STREAM("Shape has mesh type but mesh could not be retried from the MeshManager. Loading ad-hoc!");
            ROS_WARN_STREAM(" mesh uri: " << uri);

            // Load the mesh ad-hoc if the manager doesn't have it
            // this happens with model:// uris
            mesh = MeshManager::Instance()->Load(uri);

            if (!mesh) {
              ROS_WARN_STREAM("Mesh could not be loaded: " << uri);
              continue;
            }
          }

          // Iterate over submeshes
          unsigned n_submeshes = mesh->GetSubMeshCount();

          for (unsigned m = 0; m < n_submeshes; m++) {
            const SubMesh *submesh = mesh->GetSubMesh(m);

            switch (submesh->GetPrimitiveType()) {
              case SubMesh::POINTS:
              case SubMesh::LINES:
              case SubMesh::LINESTRIPS:
                // These aren't supported
                ROS_ERROR_STREAM("Unsupported primitive type " << submesh->GetPrimitiveType());
                break;
              case SubMesh::TRIANGLES:
              case SubMesh::TRISTRIPS:
                object.mesh_poses.push_back(collision_pose_msg);
                break;
              case SubMesh::TRIFANS:
                // Unsupported
                ROS_ERROR_STREAM("TRIFANS not supported");
                break;
            }
          }
        } else {
          object.primitive_poses.push_back(collision_pose_msg);
        }

        if (shape->HasType(Base::MESH_SHAPE)) {
          // Get the mesh structure from the mesh shape
          boost::shared_ptr<MeshShape> mesh_shape = boost::dynamic_pointer_cast<MeshShape>(shape);
          std::string name = mesh_shape->GetName();
          std::string uri = mesh_shape->GetMeshURI();
          ignition::math::Vector3d scale = mesh_shape->Scale();
          const Mesh *mesh = MeshManager::Instance()->GetMesh(uri);

          if (!mesh) {
            ROS_WARN_STREAM(
                "Shape has mesh type but mesh could not be retried from the MeshManager. Loading "
                "ad-hoc!");
            ROS_WARN_STREAM(" mesh uri: " << uri);

            // Load the mesh ad-hoc if the manager doesn't have it
            // this happens with model:// uris
            mesh = MeshManager::Instance()->Load(uri);

            if (!mesh) {
              ROS_WARN_STREAM("Mesh could not be loded: " << uri);
              continue;
            }
          }

          // Iterate over submeshes
          unsigned n_submeshes = mesh->GetSubMeshCount();

          for (unsigned m = 0; m < n_submeshes; m++) {
            const SubMesh *submesh = mesh->GetSubMesh(m);
            unsigned n_vertices = submesh->GetVertexCount();

            switch (submesh->GetPrimitiveType()) {
              case SubMesh::POINTS:
              case SubMesh::LINES:
              case SubMesh::LINESTRIPS:
                // These aren't supported
                break;
              case SubMesh::TRIANGLES: {
                shape_msgs::Mesh mesh_msg;
                mesh_msg.vertices.resize(n_vertices);
                mesh_msg.triangles.resize(n_vertices / 3);

                for (size_t v = 0; v < n_vertices; v++) {
                  const int index = submesh->GetIndex(v);
                  const ignition::math::Vector3d vertex = submesh->Vertex(v);

                  mesh_msg.vertices[index].x = vertex.X() * scale.X();
                  mesh_msg.vertices[index].y = vertex.Y() * scale.Y();
                  mesh_msg.vertices[index].z = vertex.Z() * scale.Z();

                  mesh_msg.triangles[v / 3].vertex_indices[v % 3] = index;
                }

                object.meshes.push_back(mesh_msg);
                break;
              }
              case SubMesh::TRISTRIPS: {
                shape_msgs::Mesh mesh_msg;
                mesh_msg.vertices.resize(n_vertices);
                mesh_msg.triangles.resize(n_vertices - 2);

                for (size_t v = 0; v < n_vertices; v++) {
                  const int index = submesh->GetIndex(v);
                  const ignition::math::Vector3d vertex = submesh->Vertex(v);

                  mesh_msg.vertices[index].x = vertex.X() * scale.X();
                  mesh_msg.vertices[index].y = vertex.Y() * scale.Y();
                  mesh_msg.vertices[index].z = vertex.Z() * scale.Z();

                  if (v < n_vertices - 2) mesh_msg.triangles[v].vertex_indices[0] = index;
                  if (v > 0 && v < n_vertices - 1) mesh_msg.triangles[v - 1].vertex_indices[1] = index;
                  if (v > 1) mesh_msg.triangles[v - 2].vertex_indices[2] = index;
                }

                object.meshes.push_back(mesh_msg);
                break;
              }
              case SubMesh::TRIFANS:
                // Unsupported
                ROS_ERROR_STREAM("TRIFANS not supported");
                break;
            }
          }
        } else {
          // Solid primitive
          shape_msgs::SolidPrimitive primitive_msg;

          if (shape->HasType(Base::BOX_SHAPE)) {
            boost::shared_ptr<BoxShape> box_shape = boost::dynamic_pointer_cast<BoxShape>(shape);

            primitive_msg.type = primitive_msg.BOX;
            primitive_msg.dimensions.resize(3);
            primitive_msg.dimensions[0] = box_shape->Size().X() * scale_primitives_factor_;
            primitive_msg.dimensions[1] = box_shape->Size().Y() * scale_primitives_factor_;
            primitive_msg.dimensions[2] = box_shape->Size().Z() * scale_primitives_factor_;
          } else if (shape->HasType(Base::CYLINDER_SHAPE)) {
            boost::shared_ptr<CylinderShape> cylinder_shape = boost::dynamic_pointer_cast<CylinderShape>(shape);

            primitive_msg.type = primitive_msg.CYLINDER;
            primitive_msg.dimensions.resize(2);
            primitive_msg.dimensions[0] = cylinder_shape->GetLength() * scale_primitives_factor_;
            primitive_msg.dimensions[1] = cylinder_shape->GetRadius() * scale_primitives_factor_;
          } else if (shape->HasType(Base::SPHERE_SHAPE)) {
            boost::shared_ptr<SphereShape> sphere_shape = boost::dynamic_pointer_cast<SphereShape>(shape);

            primitive_msg.type = primitive_msg.SPHERE;
            primitive_msg.dimensions.resize(1);
            primitive_msg.dimensions[0] = sphere_shape->GetRadius() * scale_primitives_factor_;
          } else {
            // HEIGHTMAP_SHAPE, MAP_SHAPE, MULTIRAY_SHAPE, RAY_SHAPE
            // Unsupported
            continue;
          }

          object.primitives.push_back(primitive_msg);
        }
        ROS_DEBUG_THROTTLE_NAMED(1, LOGNAME, "model %s has %zu links", model_name.c_str(), links.size());
        ROS_DEBUG_THROTTLE_NAMED(1, LOGNAME, "model %s has %zu meshes, %zu mesh poses", model_name.c_str(),
                                 object.meshes.size(), object.mesh_poses.size());
      }
    }
  }

  // Iterate through the objects we're already tracking
  std::vector<std::string> to_remove;
  for (auto &[name, object] : collision_object_map_) {
    // Update stamp
    object.header.stamp = ros::Time::now();
    // Add the object to the planning scene message
    planning_scene_msg.world.collision_objects.push_back(object);

    moveit_msgs::ObjectColor object_color;
    object_color.id = object.id;
    std_msgs::ColorRGBA color;
    color.r = 0.f;
    color.g = 0.f;
    color.b = 1.f;
    color.a = 1.f;
    object_color.color = color;

    // planning_scene_msg.object_colors.push_back(object_color);

    // Actually remove objects from being tracked
    if (object.operation == moveit_msgs::CollisionObject::REMOVE) {
      // Mark for removal from the map
      to_remove.push_back(name);
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Removing object: " << object.id);
    }
  }

  // Actually remove the collision object from the map

  for (auto const &name : to_remove) {
    collision_object_map_.erase(name);
  }

  return planning_scene_msg;
}

void GazeboRosMoveItPlanningScene::QueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode_->ok()) {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace gazebo
