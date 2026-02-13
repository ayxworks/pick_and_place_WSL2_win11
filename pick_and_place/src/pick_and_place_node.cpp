#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// ROS 2 node implementing a basic Pick & Place pipeline using MoveIt and TF
class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode()
        : Node("pick_and_place_node",
               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
          node_(std::make_shared<rclcpp::Node>("pick_and_place_group_node")),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
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
        try {
            object_id_ = this->get_parameter("object_id").as_string();
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
            object_id_ = "picked_object";
            RCLCPP_INFO(this->get_logger(), "Using default object_id: %s", object_id_.c_str());
        }

        try {
            object_dimensions_.push_back(this->get_parameter("object_length").as_double());
            object_dimensions_.push_back(this->get_parameter("object_width").as_double());
            object_dimensions_.push_back(this->get_parameter("object_height").as_double());
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
            // Default dimensions: 5cm x 5cm x 10cm
            object_dimensions_ = {0.05, 0.05, 0.10};
            RCLCPP_INFO(this->get_logger(), 
                "Using default object dimensions: %.3f x %.3f x %.3f m",
                object_dimensions_[0], object_dimensions_[1], object_dimensions_[2]);
        }

        // Read object detection service name
        detect_service_name_ = this->get_parameter("detect_service_name").as_string();

        // Velocity and acceleration scaling factors
        velocity_scaling_ = this->get_parameter("velocity_scaling").as_double();
        acceleration_scaling_ = this->get_parameter("acceleration_scaling").as_double();

        // Planning retries
        planning_retries_ = this->get_parameter("planning_retries").as_int();


        // Create service client for object detection
        detect_client_ = this->create_client<std_srvs::srv::Trigger>(detect_service_name_);

        // Create start and stop services
        start_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "start_pick_place",
            std::bind(&PickAndPlaceNode::start_cb, this,
                    std::placeholders::_1, std::placeholders::_2));

        stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_pick_place",
            std::bind(&PickAndPlaceNode::stop_cb, this,
                    std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces initialized");

        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { this->executor_->spin(); });

        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, arm_move_group_);

        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, gripper_move_group_);

        arm_->setMaxVelocityScalingFactor(velocity_scaling_);
        arm_->setMaxAccelerationScalingFactor(acceleration_scaling_);

        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();


    }

    void start_cb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (running_) {
            res->success = false;
            res->message = "Pick & Place already running";
            return;
        }

        running_ = true;
        worker_thread_ = std::thread([this]() {
            execute_pick_and_place();
            running_ = false;
        });

        worker_thread_.detach();

        res->success = true;
        res->message = "Pick & Place started";
    }

    void stop_cb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (!running_) {
            res->success = false;
            res->message = "Pick & Place not running";
            return;
        }

        running_ = false;
        arm_->stop();
        gripper_->stop();

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        arm_->clearPoseTargets();
        gripper_->clearPoseTargets();

        res->success = true;
        res->message = "Pick & Place stopped";
    }

    // Attach collision object to the planning scene and to the end-effector
    bool attach_collision_object()
    {
        // Create collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = object_id_;
        collision_object.header.frame_id = arm_->getEndEffectorLink();

        // Define box primitive
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = object_dimensions_[0]; // length (x)
        primitive.dimensions[1] = object_dimensions_[1]; // width (y)
        primitive.dimensions[2] = object_dimensions_[2]; // height (z)

        // Pose of the object relative to end-effector
        geometry_msgs::msg::Pose object_pose;
        object_pose.orientation.w = 1.0;
        object_pose.position.x = 0.0;
        object_pose.position.y = 0.0;
        object_pose.position.z = object_dimensions_[2] / 2.0; // Centro del objeto

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(object_pose);
        collision_object.operation = collision_object.ADD;

        // Add to planning scene
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_->addCollisionObjects(collision_objects);

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // Attach to end-effector
        std::vector<std::string> touch_links;
        // Add relevant touch links
        touch_links.push_back(arm_->getEndEffectorLink());
        touch_links.push_back("left_inner_finger_pad");
        touch_links.push_back("right_inner_finger_pad");
        arm_->attachObject(object_id_, arm_->getEndEffectorLink(), touch_links);

        RCLCPP_INFO(this->get_logger(), "Object '%s' attached to end-effector", object_id_.c_str());
        return true;
    }

    // Detach and remove collision object from the planning scene
    bool detach_collision_object()
    {
        arm_->detachObject(object_id_);
        
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        
        // Remove from planning scene
        std::vector<std::string> object_ids;
        object_ids.push_back(object_id_);
        planning_scene_->removeCollisionObjects(object_ids);

        RCLCPP_INFO(this->get_logger(), "Object '%s' detached and removed", object_id_.c_str());
        return true;
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

        future.wait();

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

    // Move the arm to a named joint configuration in MoveIt
    bool move_to_named_target(const std::string &target_name)
    {
        return retry_planning([&]() {

            set_ompl_planner();
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(target_name);

            moveit::planning_interface::MoveGroupInterface::Plan plan;

            if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
                return false;

            auto exec_result = arm_->execute(plan);
            arm_->clearPoseTargets();

            return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);

        }, "Move to named target: " + target_name);
    }


    // Main Pick & Place sequence
    void execute_pick_and_place()
    {
        // Give some time for TF and MoveIt to be ready
        rclcpp::sleep_for(std::chrono::seconds(1));

        try
        {

            // Move to home position
            RCLCPP_INFO(this->get_logger(), "Moving to home position");
            if (!move_to_named_target(home_position_) || !running_)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to home position");
                return;
            }

            // Move to observation position
            RCLCPP_INFO(this->get_logger(), "Moving to observation position");
            if (!move_to_named_target(observation_position_) || !running_)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to observation position");
                return;
            }

            // Run object detection
            if (!call_detect_service() || !running_)
            {
                RCLCPP_ERROR(this->get_logger(), "Object detection failed, aborting sequence");
                return;
            }

            // Execute pick
            if (!pick(pre_pick_frame_) || !running_)
            {
                RCLCPP_ERROR(this->get_logger(), "Pick failed, aborting");
                return;
            }

            // Execute place
            if (!place(pre_place_frame_) || !running_)
            {
                RCLCPP_ERROR(this->get_logger(), "Place failed, aborting");
                return;
            }

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
        return retry_planning([&]() {

            set_ompl_planner();
            arm_->setStartStateToCurrentState();
            arm_->setPoseTarget(pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;

            if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
                return false;

            auto exec_result = arm_->execute(plan);
            arm_->clearPoseTargets();

            return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);

        }, "Move to pose");
    }


    // Move the gripper to a named configuration
    bool move_gripper(const std::string &position_name)
    {
        return retry_planning([&]() {

            gripper_->setNamedTarget(position_name);

            moveit::planning_interface::MoveGroupInterface::Plan plan;

            if (gripper_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
                return false;

            auto exec_result = gripper_->execute(plan);
            gripper_->clearPoseTargets();

            return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);

        }, "Move gripper: " + position_name);
    }


    bool move_relative_to_end_effector(double x, double y, double z)
    {
        // Get current end-effector pose in planning frame
        geometry_msgs::msg::PoseStamped current_pose;

        try {
            auto tf = tf_buffer_.lookupTransform(
                arm_->getPlanningFrame(),      // target frame
                arm_->getEndEffectorLink(),    // source frame
                tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            
            current_pose.header = tf.header;
            current_pose.pose.position.x = tf.transform.translation.x;
            current_pose.pose.position.y = tf.transform.translation.y;
            current_pose.pose.position.z = tf.transform.translation.z;
            current_pose.pose.orientation = tf.transform.rotation;

        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to get EE pose from TF: %s", ex.what());
            return false;
        }

        // Create offset vector in end-effector frame
        geometry_msgs::msg::Vector3Stamped offset_ee;
        offset_ee.header.frame_id = arm_->getEndEffectorLink();
        offset_ee.vector.x = x;
        offset_ee.vector.y = y;
        offset_ee.vector.z = z;

        // Transform offset to planning frame
        geometry_msgs::msg::Vector3Stamped offset_planning_frame;
        try {
            auto transform = tf_buffer_.lookupTransform(
                arm_->getPlanningFrame(),
                arm_->getEndEffectorLink(),
                tf2::TimePointZero);

            tf2::doTransform(offset_ee, offset_planning_frame, transform);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
            return false;
        }

        // Calculate target pose by adding offset in planning frame
        geometry_msgs::msg::PoseStamped target_pose = current_pose;
        target_pose.pose.position.x += offset_planning_frame.vector.x;
        target_pose.pose.position.y += offset_planning_frame.vector.y;
        target_pose.pose.position.z += offset_planning_frame.vector.z;
        

        bool success = move_cartesian_to_pose(target_pose);

        return success;
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

        // Attach collision object to the gripper
        if (!attach_collision_object()) return false;

        // Lift object - movement in Z direction of WORLD frame
        RCLCPP_INFO(this->get_logger(), "Lifting object (%.3f m in world Z)", lift_distance_);

        // Get current end-effector pose in world frame
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_frame_pose(arm_->getEndEffectorLink(), current_pose, "world"))
            return false;

        // Create lift pose by adding to Z in world frame
        geometry_msgs::msg::PoseStamped lift_pose = current_pose;
        lift_pose.pose.position.z += lift_distance_;

        if (!move_cartesian_to_pose(lift_pose)) return false;
        
        return true;
    }

    // Move linearly to an absolute pose using Cartesian path
    bool move_cartesian_to_pose(const geometry_msgs::msg::PoseStamped &pose)
    {
        return retry_planning([&]() {

            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.01;
            const double jump_threshold = 2.5

            double fraction = arm_->computeCartesianPath(
                waypoints,
                eef_step,
                jump_threshold,
                trajectory);

            if (fraction < 0.95)
                return false;

            // Manually time-parameterize the trajectory to apply velocity and acceleration scaling
            robot_trajectory::RobotTrajectory rt(
                arm_->getCurrentState()->getRobotModel(),
                arm_->getName());

            rt.setRobotTrajectoryMsg(*arm_->getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;

            if (!iptp.computeTimeStamps(rt, velocity_scaling_, acceleration_scaling_))
                return false;

            rt.getRobotTrajectoryMsg(trajectory);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            arm_->setStartStateToCurrentState();

            return (arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        }, "Cartesian move");
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
        geometry_msgs::msg::PoseStamped place_pose = pre_place_pose;
        place_pose.pose.position.z -= drop_distance_;
        RCLCPP_INFO(this->get_logger(), "Lowering object (%.3f m in world Z)", drop_distance_);
        if (!move_cartesian_to_pose(place_pose)) return false;

        // Detach collision object from the gripper
        if (!detach_collision_object()) return false;

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
        arm_->setPlannerId("RRTstarkConfigDefault");
    }

private:

    template <typename Func>
    bool retry_planning(Func &&func, const std::string &description)
    {
        for (int attempt = 1; attempt <= planning_retries_; ++attempt)
        {
            if (!running_)
                return false;

            RCLCPP_INFO(this->get_logger(),
                "[%s] Attempt %d/%d",
                description.c_str(), attempt, planning_retries_);

            if (func())
            {
                RCLCPP_INFO(this->get_logger(),
                    "[%s] Success on attempt %d",
                    description.c_str(), attempt);
                return true;
            }

            RCLCPP_WARN(this->get_logger(),
                "[%s] Failed attempt %d/%d",
                description.c_str(), attempt, planning_retries_);

            arm_->clearPoseTargets();
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_ERROR(this->get_logger(),
            "[%s] All %d attempts failed",
            description.c_str(), planning_retries_);

        return false;
    } 

    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Executor::SharedPtr executor_;
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr detect_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

    std::string arm_move_group_;
    std::string gripper_move_group_;
    std::string pre_pick_frame_;
    std::string pre_place_frame_;
    std::string home_position_;
    std::string observation_position_;
    std::string gripper_open_position_;
    std::string gripper_closed_position_;
    std::string detect_service_name_;
    std::string object_id_;
    
    double pick_approach_distance_;
    double place_leave_distance_;
    double lift_distance_;
    double drop_distance_;
    
    std::vector<double> object_dimensions_;
    std::atomic_bool running_{false};
    std::thread worker_thread_;
    std::thread executor_thread_;

    double velocity_scaling_;
    double acceleration_scaling_;

    int planning_retries_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PickAndPlaceNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}