#ifndef AGV_FORWARD_CONTROLLER
#define AGV_FORWARD_CONTROLLER


// C related includes
#include <array>
#include <cmath>
#include <memory>

// All ros2-related stuff, mas que nada utils
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/create_timer.hpp"


//ros2 control relacionado
#include "controller_interface/controller_interface.hpp"


//Math utils
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

// Just for making our lifes easier
using Twist = geometry_msgs::msg::Twist;

//! Creating a namespace for the controller
namespace agv_forward_controller{

    class AgvForwardController : public controller_interface::ControllerInterface{

        public:

            AgvForwardController();

            controller_interface::CallbackReturn
            on_init() override;

            controller_interface::CallbackReturn
            on_configure(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type
            update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            controller_interface::InterfaceConfiguration
            command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration
            state_interface_configuration() const override;



        protected:
            std::vector<std::string> joint_names_;
            std::string interface_name_;
            rclcpp::Subscription<Twist>::SharedPtr twist_subs_;

            float chassis_wb = 0.2;
            float chassis_tw = 0.1;

            Eigen::Matrix<double, 4, 3> world_vel_to_wheel_vel_map;
            Eigen::Matrix<double, 3, 1> world_vel_vector;
            Eigen::Matrix<double, 4, 1> wheel_vel_vector;

            float u_cmd_;
            float v_cmd_;
            float yaw_cmd_;

    };

}

#endif