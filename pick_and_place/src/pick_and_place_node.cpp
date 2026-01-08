#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode()
        : Node("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
          
    {
        // Leer parámetros de move groups y frames
        arm_move_group_ = this->get_parameter("arm_move_group").as_string();
        gripper_move_group_ = this->get_parameter("gripper_move_group").as_string();
        pick_frame_ = this->get_parameter("pick_frame").as_string();
        place_frame_ = this->get_parameter("place_frame").as_string();
        
        // Leer parámetros de posiciones nombradas
        home_position_ = this->get_parameter("home_position").as_string();
        observation_position_ = this->get_parameter("observation_position").as_string();
        gripper_open_position_ = this->get_parameter("gripper_open_position").as_string();
        gripper_closed_position_ = this->get_parameter("gripper_closed_position").as_string();
        
        // Leer nombre del servicio de detección
        detect_service_name_ = this->get_parameter("detect_service_name").as_string();
        
        // Crear cliente del servicio de detección
        detect_client_ = this->create_client<std_srvs::srv::Trigger>(detect_service_name_);
        
        RCLCPP_INFO(this->get_logger(), "Nodo Pick and Place inicializado");
        RCLCPP_INFO(this->get_logger(), "Pick frame: %s", pick_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Place frame: %s", place_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Home position: %s", home_position_.c_str());
        RCLCPP_INFO(this->get_logger(), "Observation position: %s", observation_position_.c_str());
        RCLCPP_INFO(this->get_logger(), "Gripper open: %s, closed: %s", gripper_open_position_.c_str(), gripper_closed_position_.c_str());
        RCLCPP_INFO(this->get_logger(), "Servicio de detección: %s", detect_service_name_.c_str());
    }

    void init_moveit()
    {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), arm_move_group_);

        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), gripper_move_group_);

        RCLCPP_INFO(this->get_logger(), "MoveIt interfaces inicializadas");
    }


    bool call_detect_service()
    {
        // Esperar a que el servicio esté disponible
        RCLCPP_INFO(this->get_logger(), "Esperando al servicio %s...", detect_service_name_.c_str());
        if (!detect_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Servicio %s no disponible", detect_service_name_.c_str());
            return false;
        }

        // Crear la petición
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
        RCLCPP_INFO(this->get_logger(), "Llamando al servicio de detección...");
        
        // Llamar al servicio de forma síncrona
        auto future = detect_client_->async_send_request(request);
        
        // Esperar la respuesta
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Detección exitosa: %s", response->message.c_str());
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Detección fallida: %s", response->message.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error al llamar al servicio de detección");
            return false;
        }
    }

    bool move_to_named_target(const std::string &target_name)
    {
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
            RCLCPP_ERROR(this->get_logger(), "Planificación fallida para target: %s", target_name.c_str());
        }
        return success;
    }

    void execute_pick_and_place()
    {
        // Esperar un momento a que TF y MoveIt estén listos
        rclcpp::sleep_for(std::chrono::seconds(1));

        try
        {
            RCLCPP_INFO(this->get_logger(), "Iniciando secuencia pick and place");
            
            // Ir a home antes de observar
            RCLCPP_INFO(this->get_logger(), "Moviendo a posición home");
            if (!move_to_named_target(home_position_))
            {
                RCLCPP_ERROR(this->get_logger(), "No se pudo mover a home");
                return;
            }
            
            // Ir a posición de observación
            RCLCPP_INFO(this->get_logger(), "Moviendo a posición de observación");
            if (!move_to_named_target(observation_position_))
            {
                RCLCPP_ERROR(this->get_logger(), "No se pudo mover a posición de observación");
                return;
            }
            
            // Llamar al servicio de detección
            if (!call_detect_service())
            {
                RCLCPP_ERROR(this->get_logger(), "Detección de objeto fallida, abortando secuencia");
                return;
            }
            
            // Continuar con pick and place
            pick(pick_frame_);
            place(place_frame_);
            
            // Ir a home después del place
            RCLCPP_INFO(this->get_logger(), "Volviendo a posición home");
            move_to_named_target(home_position_);
            
            RCLCPP_INFO(this->get_logger(), "Secuencia pick and place completada");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error en secuencia pick and place: %s", e.what());
        }
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
            RCLCPP_ERROR(this->get_logger(), "Planificación fallida");
        }
        return success;
    }

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
            RCLCPP_ERROR(this->get_logger(), "Planificación gripper fallida para %s", position_name.c_str());
        }
        return success;
    }

    bool move_relative_to_frame(const geometry_msgs::msg::PoseStamped &current_pose,
                                const tf2::Vector3 &offset_in_frame)
    {
        tf2::Transform tf_world_current;
        tf2::fromMsg(current_pose.pose, tf_world_current);

        tf2::Transform tf_offset;
        tf_offset.setIdentity();
        tf_offset.setOrigin(offset_in_frame);

        tf2::Transform tf_world_target = tf_world_current * tf_offset;

        geometry_msgs::msg::PoseStamped target_pose = current_pose;

        geometry_msgs::msg::Pose target_pose_msg;
        tf2::toMsg(tf_world_target, target_pose_msg);
        target_pose.pose = target_pose_msg;

        return move_to_pose(target_pose);
    }


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
            RCLCPP_ERROR(this->get_logger(), "PILZ LIN falló");
        }

        arm_->clearPoseTargets();
        return success;
    }

    bool move_lin_relative_to_frame(const geometry_msgs::msg::PoseStamped &current_pose,
                                    const tf2::Vector3 &offset)
    {
        tf2::Transform tf_world_current;
        tf2::fromMsg(current_pose.pose, tf_world_current);

        tf2::Transform tf_offset;
        tf_offset.setIdentity();
        tf_offset.setOrigin(offset);

        tf2::Transform tf_world_target = tf_world_current * tf_offset;

        geometry_msgs::msg::PoseStamped target_pose = current_pose;

        geometry_msgs::msg::Pose target_pose_msg;
        tf2::toMsg(tf_world_target, target_pose_msg);
        target_pose.pose = target_pose_msg;

        return move_lin_to_pose(target_pose);
    }

    void pick(const std::string &target_frame)
    {
        geometry_msgs::msg::PoseStamped pick_pose = get_frame_pose(target_frame);
        RCLCPP_INFO(this->get_logger(), "Moviendo al frame %s", target_frame.c_str());
        move_to_pose(pick_pose);

        // Mover 10 cm en Z del gripper
        RCLCPP_INFO(this->get_logger(), "Moviendo 10cm en Z del gripper");
        move_lin_relative_to_frame(pick_pose, tf2::Vector3(0.0, 0.0, 0.10));

        // Cerrar gripper
        RCLCPP_INFO(this->get_logger(), "Cerrando gripper");
        move_gripper(gripper_closed_position_);

        // Volver a la pose pick
        RCLCPP_INFO(this->get_logger(), "Volviendo a la pose pick");
        move_lin_to_pose(pick_pose);

        // Subir 10 cm en Z del world
        geometry_msgs::msg::PoseStamped lift_pose = pick_pose;
        lift_pose.pose.position.z += 0.10;
        RCLCPP_INFO(this->get_logger(), "Subiendo 10cm en Z world");
        move_lin_to_pose(lift_pose);
    }

    void place(const std::string &target_frame)
    {
        geometry_msgs::msg::PoseStamped place_pose = get_frame_pose(target_frame);
        geometry_msgs::msg::PoseStamped place_pre_pose = place_pose;
        place_pre_pose.pose.position.z += 0.10;
        RCLCPP_INFO(this->get_logger(), "Moviendo al frame %s", target_frame.c_str());
        move_to_pose(place_pre_pose);

        // Bajar 10 cm en Z de world
        RCLCPP_INFO(this->get_logger(), "Bajando 10cm en Z world");
        move_lin_to_pose(place_pose);

        // Abrir gripper
        RCLCPP_INFO(this->get_logger(), "Abriendo gripper");
        move_gripper(gripper_open_position_);
    }

    void set_ompl_planner()
    {
        arm_->setPlanningPipelineId("ompl");
        arm_->setPlannerId("RRTConnectkConfigDefault");
    }

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
    std::string pick_frame_;
    std::string place_frame_;
    std::string home_position_;
    std::string observation_position_;
    std::string gripper_open_position_;
    std::string gripper_closed_position_;
    std::string detect_service_name_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PickAndPlaceNode>();

    node->init_moveit();
    
    // Ejecutar la secuencia pick and place
    node->execute_pick_and_place();

    rclcpp::shutdown();
    return 0;
}