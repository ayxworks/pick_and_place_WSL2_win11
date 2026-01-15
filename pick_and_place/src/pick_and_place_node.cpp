#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// ROS 2 node implementing a basic Pick & Place pipeline using MoveIt and TF
class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode()
        : Node("pick_and_place_node",
               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Read MoveIt group names and TF frames from parameters
        arm_move_group_ = this->get_parameter("arm_move_group").as_string();
        gripper_move_group_ = this->get_parameter("gripper_move_group").as_string();
        pre_pick_frame_ = this->get_parameter("pre_pick_frame").as_string();
        pre_place_frame_ = this->get_parameter("pre_place_frame").as_string();

        // Read named joint targets
        home_position_ = this->get_parameter("home_position").as_string();
        observation_position_ = this->get_parameter("observation_position").as_string();
        gripper_open_position_ = this->get_parameter("gripper_open_position").as_string();
        gripper_closed_position_ = this->get_parameter("gripper_closed_position").as_string();
        pick_approach_distance_ = this->get_parameter("pick_approach_distance").as_double();
        place_leave_distance_ = this->get_parameter("place_leave_distance").as_double();
        lift_distance_ = this->get_parameter("lift_distance").as_double();
        drop_distance_ = this->get_parameter("drop_distance").as_double();

        // Read object detection service name
        detect_service_name_ = this->get_parameter("detect_service_name").as_string();

        // Create service client for object detection
        detect_client_ = this->create_client<std_srvs::srv::Trigger>(detect_service_name_);
    }

    // Initialize MoveIt MoveGroup interfaces
    void init_moveit()
    {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_move_group_);

        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), gripper_move_group_);

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized");
    }

    // Call object detection service (blocking)
    bool call_detect_service()
    {
        // Wait until the service becomes available
        RCLCPP_INFO(this->get_logger(), "Waiting for service %s...", detect_service_name_.c_str());
        if (!detect_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", detect_service_name_.c_str());
            return false;
        }

        // Create empty Trigger request
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        RCLCPP_INFO(this->get_logger(), "Calling detection service...");

        // Send request asynchronously
        auto future = detect_client_->async_send_request(request);

        // Block until response is received
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Detection successful: %s", response->message.c_str());
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Detection failed: %s", response->message.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call detection service");
            return false;
        }
    }

    // Move the arm to a named joint configuration in MoveIt
    bool move_to_named_target(const std::string &target_name)
    {
        set_ompl_planner();
        arm_->setNamedTarget(target_name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            arm_->execute(plan);
            arm_->clearPoseTargets();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed for target: %s", target_name.c_str());
        }
        return success;
    }

    // Main Pick & Place sequence
    void execute_pick_and_place()
    {
        // Give some time for TF and MoveIt to be ready
        rclcpp::sleep_for(std::chrono::seconds(1));

        try
        {
            RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence");

            // Move to home position
            RCLCPP_INFO(this->get_logger(), "Moving to home position");
            if (!move_to_named_target(home_position_))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to home position");
                return;
            }

            // Move to observation pose
            RCLCPP_INFO(this->get_logger(), "Moving to observation position");
            if (!move_to_named_target(observation_position_))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to observation position");
                return;
            }

            // Run object detection
            if (!call_detect_service())
            {
                RCLCPP_ERROR(this->get_logger(), "Object detection failed, aborting sequence");
                return;
            }

            // Execute pick
            if (!pick(pre_pick_frame_))
            {
                RCLCPP_ERROR(this->get_logger(), "Pick failed, aborting");
                return;
            }

            // Execute place
            if (!place(pre_place_frame_))
            {
                RCLCPP_ERROR(this->get_logger(), "Place failed, aborting");
                return;
            }

            // Return to home position
            RCLCPP_INFO(this->get_logger(), "Returning to home position");
            move_to_named_target(home_position_);

            RCLCPP_INFO(this->get_logger(), "Pick and place sequence completed successfully");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Pick and place sequence error: %s", e.what());
        }
    }

    // Get the pose of a TF frame expressed in a reference frame
    bool get_frame_pose(const std::string &frame_name,
                        geometry_msgs::msg::PoseStamped &pose,
                        const std::string &reference_frame = "world")
    {
        try
        {
            auto t = tf_buffer_.lookupTransform(reference_frame, frame_name, tf2::TimePointZero);
            pose.header.frame_id = reference_frame;
            pose.pose.position.x = t.transform.translation.x;
            pose.pose.position.y = t.transform.translation.y;
            pose.pose.position.z = t.transform.translation.z;
            pose.pose.orientation = t.transform.rotation;
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed for %s: %s",
                         frame_name.c_str(), ex.what());
            return false;
        }
    }

    // Plan and execute motion to an absolute pose (OMPL)
    bool move_to_pose(const geometry_msgs::msg::PoseStamped &pose)
    {
        set_ompl_planner();
        arm_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            arm_->execute(plan);
            arm_->clearPoseTargets();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
        }
        return success;
    }

    // Move the gripper to a named configuration
    bool move_gripper(const std::string &position_name)
    {
        gripper_->setNamedTarget(position_name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (gripper_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            gripper_->execute(plan);
            gripper_->clearPoseTargets();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper planning failed for %s", position_name.c_str());
        }
        return success;
    }

    // Move linearly to a pose using Pilz LIN planner
    bool move_lin_to_pose(const geometry_msgs::msg::PoseStamped &pose)
    {
        set_pilz_lin_planner();
        arm_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            arm_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "PILZ LIN planning failed");
        }

        arm_->clearPoseTargets();
        return success;
    }

    // Applies a relative displacement in the frame of the end-effector
    bool move_relative_to_end_effector(double x, double y, double z)
    {
        // Obtain the actual pose of the end-effector
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.frame_id = arm_->getPlanningFrame();
        current_pose.pose = arm_->getCurrentPose().pose;

        // Create the displacement in the frame of the end-effector
        geometry_msgs::msg::Vector3Stamped offset;
        offset.header.frame_id = arm_->getEndEffectorLink();
        offset.vector.x = x;
        offset.vector.y = y;
        offset.vector.z = z;

        // Transform the offset to the planning frame
        geometry_msgs::msg::Vector3Stamped offset_transformed;
        try
        {
            tf_buffer_.transform(offset, offset_transformed, arm_->getPlanningFrame());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
            return false;
        }

        // Apply the offset to the current pose
        geometry_msgs::msg::PoseStamped target_pose = current_pose;
        target_pose.pose.position.x += offset_transformed.vector.x;
        target_pose.pose.position.y += offset_transformed.vector.y;
        target_pose.pose.position.z += offset_transformed.vector.z;

        return move_lin_to_pose(target_pose);
    } 

    // Pick routine: approach, grasp, and lift
    bool pick(const std::string &target_frame)
    {
        geometry_msgs::msg::PoseStamped pre_pick_pose;
        if (!get_frame_pose(target_frame, pre_pick_pose))
            return false;

        // Go to approximation pose
        RCLCPP_INFO(this->get_logger(), "Moving to pre-pick pose");
        if (!move_to_pose(pre_pick_pose)) 
            return false;

        // Relative movement in Z direction of the gripper
        RCLCPP_INFO(this->get_logger(), "Approaching object (%.3f m in gripper Z)", 
                pick_approach_distance_);
        if (!move_relative_to_end_effector(0.0, 0.0, pick_approach_distance_))
            return false;

        // Close gripper
        RCLCPP_INFO(this->get_logger(), "Closing gripper");
        if (!move_gripper(gripper_closed_position_)) return false;

        // Lift object - movement in Z direction of WORLD frame
        RCLCPP_INFO(this->get_logger(), "Lifting object (%.3f m in world Z)", lift_distance_);

        // Get current end-effector pose in world frame
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_frame_pose(arm_->getEndEffectorLink(), current_pose, "world"))
            return false;

        // Create lift pose by adding to Z in world frame
        geometry_msgs::msg::PoseStamped lift_pose = current_pose;
        lift_pose.pose.position.z += lift_distance_;

        if (!move_lin_to_pose(lift_pose)) return false;
        
        return true;
    }

    // Place routine: approach, release, and retreat
    bool place(const std::string &target_frame)
    {
        geometry_msgs::msg::PoseStamped pre_place_pose;
        if (!get_frame_pose(target_frame, pre_place_pose))
            return false;

        // Go to pre-place pose
        RCLCPP_INFO(this->get_logger(), "Moving to pre-place pose");
        if (!move_to_pose(pre_place_pose)) return false;

        // Relative movement in Z direction of the WORLD to lower the object
        geometry_msgs::msg::PoseStamped place_pose;
        pre_place_pose.pose.position.z -= drop_distance_;
        RCLCPP_INFO(this->get_logger(), "Lowering object (%.3f m in world Z)", drop_distance_);
        if (!move_lin_to_pose(place_pose)) return false;

        // Open gripper
        if (!move_gripper(gripper_open_position_)) return false;

        // Retreat after placing
        RCLCPP_INFO(this->get_logger(), "Retreating from place pose (%.3f m in gripper Z)", 
                place_leave_distance_);
        if (!move_relative_to_end_effector(0.0, 0.0, -place_leave_distance_))
            return false;

        return true;
    }

    // Configure OMPL planner
    void set_ompl_planner()
    {
        arm_->setPlanningPipelineId("ompl");
        arm_->setPlannerId("RRTConnectkConfigDefault");
    }

    // Configure Pilz LIN planner
    void set_pilz_lin_planner()
    {
        arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_->setPlannerId("LIN");
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr detect_client_;

    std::string arm_move_group_;
    std::string gripper_move_group_;
    std::string pre_pick_frame_;
    std::string pre_place_frame_;
    std::string home_position_;
    std::string observation_position_;
    std::string gripper_open_position_;
    std::string gripper_closed_position_;
    std::string detect_service_name_;
    double pick_approach_distance_;
    double place_leave_distance_;
    double lift_distance_;
    double drop_distance_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PickAndPlaceNode>();
    node->init_moveit();

    // Execute pick and place sequence once
    node->execute_pick_and_place();

    rclcpp::shutdown();
    return 0;
}
