#include "agv_controllers/agv_foward_controller.hpp" 

namespace agv_forward_controller{

AgvForwardController::AgvForwardController() : controller_interface::ControllerInterface(){

    world_vel_to_wheel_vel_map  << 1, 1, -(chassis_tw + chassis_wb),
                                   1,-1,  chassis_tw + chassis_wb,
                                   1, -1, -(chassis_tw + chassis_wb),
                                   1, 1, chassis_tw + chassis_wb;


}

controller_interface::CallbackReturn
AgvForwardController::on_init(){

    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    interface_name_ = auto_declare<std::string>("interface_name", "actuation_per");



}


controller_interface::CallbackReturn
AgvForwardController::on_configure(const rclcpp_lifecycle::State & previous_state){

 
        (void)previous_state;

        auto callback = [this](const Twist::SharedPtr msg) -> void
        {
            u_cmd_ = msg->linear.x;
            v_cmd_ = msg->linear.y;
            yaw_cmd_ = msg->angular.z;

            world_vel_vector << u_cmd_, v_cmd_, yaw_cmd_;
            wheel_vel_vector << world_vel_to_wheel_vel_map*world_vel_vector;
            
        };

        twist_subs_ = this->get_node()->create_subscription<Twist>("/agv/cmd_vel", 10, callback);
        return controller_interface::CallbackReturn::SUCCESS;

}


controller_interface::CallbackReturn
AgvForwardController::on_activate(const rclcpp_lifecycle::State & previous_state){

    u_cmd_ = 0.;
    v_cmd_ = 0.;
    yaw_cmd_ = 0.;

    wheel_vel_vector << 0., 0., 0.;

    return CallbackReturn::SUCCESS;

}


controller_interface::return_type
AgvForwardController::update(const rclcpp::Time & time, const rclcpp::Duration & period){

    for (int ii = 0; ii < (int)joint_names_.size(); ii++){
        (void)command_interfaces_[ii].set_value(wheel_vel_vector(ii));
    }

}


controller_interface::InterfaceConfiguration
AgvForwardController::command_interface_configuration() const{

    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (auto joint_name : joint_names_)
    {
        config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;


}


controller_interface::InterfaceConfiguration
AgvForwardController::state_interface_configuration() const{

    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (auto joint_name : joint_names_){
        config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;


}

}