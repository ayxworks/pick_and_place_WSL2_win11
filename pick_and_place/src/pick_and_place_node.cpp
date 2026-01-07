#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode() : Node("pick_and_place_node"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(tf_buffer_)
    {
        // Inicializar MoveIt C++ interfaces
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");

        RCLCPP_INFO(this->get_logger(), "Nodo Pick and Place inicializado");
    }

    geometry_msgs::msg::PoseStamped get_frame_pose(const std::string &frame_name, const std::string &reference_frame = "world")
    {
        geometry_msgs::msg::PoseStamped pose;
        try
        {
            geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(reference_frame, frame_name, tf2::TimePointZero);
            pose.header.frame_id = reference_frame;
            pose.pose.position.x = t.transform.translation.x;
            pose.pose.position.y = t.transform.translation.y;
            pose.pose.position.z = t.transform.translation.z;
            pose.pose.orientation = t.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "No se pudo obtener el frame %s: %s", frame_name.c_str(), ex.what());
        }
        return pose;
    }

    bool move_to_pose(const geometry_msgs::msg::PoseStamped &pose)
    {
        arm_->setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            arm_->execute(plan);
            arm_->clearPoseTargets();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planificación fallida");
        }
        return success;
    }

    bool move_gripper(const std::string &position_name)
    {
        gripper_->setNamedTarget(position_name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            gripper_->execute(plan);
            gripper_->clearPoseTargets();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planificación gripper fallida para %s", position_name.c_str());
        }
        return success;
    }

    void pick(const std::string &target_frame)
    {
        geometry_msgs::msg::PoseStamped pose = get_frame_pose(target_frame);
        RCLCPP_INFO(this->get_logger(), "Moviendo al frame %s", target_frame.c_str());
        move_to_pose(pose);

        // Mover 10 cm en Z del gripper
        pose.pose.position.z += 0.10;
        RCLCPP_INFO(this->get_logger(), "Moviendo 10cm en Z del gripper");
        move_to_pose(pose);

        // Cerrar gripper
        RCLCPP_INFO(this->get_logger(), "Cerrando gripper");
        move_gripper("closed");

        // Volver a la pose pick
        pose.pose.position.z -= 0.10;
        RCLCPP_INFO(this->get_logger(), "Volviendo a la pose pick");
        move_to_pose(pose);

        // Subir 10 cm en Z del world
        pose.pose.position.z += 0.10;
        RCLCPP_INFO(this->get_logger(), "Subiendo 10cm en Z world");
        move_to_pose(pose);
    }

    void place(const std::string &target_frame)
    {
        geometry_msgs::msg::PoseStamped pose = get_frame_pose(target_frame);
        RCLCPP_INFO(this->get_logger(), "Moviendo al frame %s", target_frame.c_str());
        move_to_pose(pose);

        // Bajar 10 cm en Z de world
        pose.pose.position.z -= 0.10;
        RCLCPP_INFO(this->get_logger(), "Bajando 10cm en Z world");
        move_to_pose(pose);

        // Abrir gripper
        RCLCPP_INFO(this->get_logger(), "Abriendo gripper");
        move_gripper("open");
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceNode>();

    // Esperar un momento a que TF y MoveIt estén listos
    rclcpp::sleep_for(std::chrono::seconds(1));

    try
    {
        node->pick("frame_pick");    // Cambiar por el frame real
        node->place("frame_place");  // Cambiar por el frame real
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
