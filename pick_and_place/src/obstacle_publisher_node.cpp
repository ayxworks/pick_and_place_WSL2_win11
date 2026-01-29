#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <yaml-cpp/yaml.h>

class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher()
  : Node("obstacle_publisher")
  {
    this->declare_parameter<std::string>("yaml_file", "");
    auto yaml_file = this->get_parameter("yaml_file").as_string();

    if (yaml_file.empty()) {
      RCLCPP_ERROR(get_logger(), "No YAML file provided");
      return;
    }

    loadFromYaml(yaml_file);
    planning_scene_interface_.applyCollisionObjects(collision_objects_);

    RCLCPP_INFO(get_logger(), "Published %ld obstacles",
                collision_objects_.size());
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;

  void loadFromYaml(const std::string &path)
  {
    YAML::Node root = YAML::LoadFile(path);

    for (const auto &node : root["obstacles"]) {
      std::string id = node["id"].as<std::string>();
      std::string type = node["type"].as<std::string>();

      moveit_msgs::msg::CollisionObject co;
      co.id = id;
      co.header.frame_id = node["frame"].as<std::string>();
      co.operation = co.ADD;

      if (type == "box")
        makeBox(node, co);
      else if (type == "sphere")
        makeSphere(node, co);
      else if (type == "cylinder")
        makeCylinder(node, co);
      else if (type == "mesh")
        makeMesh(node, co);
      else {
        RCLCPP_WARN(get_logger(), "Unknown obstacle type: %s", type.c_str());
        continue;
      }

      collision_objects_.push_back(co);
    }
  }

  geometry_msgs::msg::Pose makePose(const YAML::Node &node)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = node["position"][0].as<double>();
    p.position.y = node["position"][1].as<double>();
    p.position.z = node["position"][2].as<double>();

    p.orientation.x = node["orientation"][0].as<double>();
    p.orientation.y = node["orientation"][1].as<double>();
    p.orientation.z = node["orientation"][2].as<double>();
    p.orientation.w = node["orientation"][3].as<double>();

    return p;
  }

  void makeBox(const YAML::Node &node,
               moveit_msgs::msg::CollisionObject &co)
  {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {
      node["size"][0].as<double>(),
      node["size"][1].as<double>(),
      node["size"][2].as<double>()
    };

    co.primitives.push_back(prim);
    co.primitive_poses.push_back(makePose(node));
  }

  void makeSphere(const YAML::Node &node,
                  moveit_msgs::msg::CollisionObject &co)
  {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.SPHERE;
    prim.dimensions = { node["radius"].as<double>() };

    co.primitives.push_back(prim);
    co.primitive_poses.push_back(makePose(node));
  }

  void makeCylinder(const YAML::Node &node,
                    moveit_msgs::msg::CollisionObject &co)
  {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.CYLINDER;
    prim.dimensions = {
      node["height"].as<double>(),
      node["radius"].as<double>()
    };

    co.primitives.push_back(prim);
    co.primitive_poses.push_back(makePose(node));
  }

  void makeMesh(const YAML::Node &node,
                moveit_msgs::msg::CollisionObject &co)
  {
    std::string resource = node["resource"].as<std::string>();
    double scale = node["scale"] ? node["scale"].as<double>() : 1.0;

    shapes::Mesh *mesh = shapes::createMeshFromResource(
      resource,
      Eigen::Vector3d(scale, scale, scale)
    );

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);

    shape_msgs::msg::Mesh mesh_msg =
      boost::get<shape_msgs::msg::Mesh>(shape_msg);

    co.meshes.push_back(mesh_msg);
    co.mesh_poses.push_back(makePose(node));

    delete mesh;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}
