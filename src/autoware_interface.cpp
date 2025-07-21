#include "autoware_interface/autoware_interface.hpp"

namespace autoware_interface_ns
{

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface(const rclcpp::NodeOptions & node_options) : Node("Aw_to_interface", node_options)
{
    dyno_mode_ = this->declare_parameter("Dyno_mode", false);
    
    // pub
    AW_velocity_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(1));  
    AW_steer_angle_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(1));    
    AW_control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(1));    
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_cmd", rclcpp::QoS(1));
    TC_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_motor_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steer_cmd", rclcpp::QoS(1));
    TC_steer_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steer_status", rclcpp::QoS(1));
    interface_vehicle_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vehicle/mode_status", rclcpp::QoS(1));
    control_cmd_pub_ = this->create_publisher<autoware_control_msgs::msg::Control>("/control/command/control_cmd_", rclcpp::QoS(1));

    // sub  
    AW_command_sub = this->create_subscription<autoware_control_msgs::msg::Control>(   
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AwCmd_callback, this, std::placeholders::_1));
    TC_throttle_cmd = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/throttle_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCthrottle_callback, this, std::placeholders::_1));
    TC_brake_cmd = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/brake_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCbrake_callback, this, std::placeholders::_1));
    TC_steer_cmd = this->create_subscription<std_msgs::msg::Int16>(
        "/twist_controller/output/steer_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCsteer_callback, this, std::placeholders::_1));

    // Can bridge
    interface_can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/socketcan/interface/from_can_bus", rclcpp::QoS(1), std::bind(&AutowareInterface::interface_can_data_callback, this, std::placeholders::_1));
    interface_can_pub_ = this->create_publisher<can_msgs::msg::Frame>(
        "/socketcan/interface/to_can_bus", rclcpp::QoS(1));
    motor_can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/socketcan/motor/from_can_bus", rclcpp::QoS(1), std::bind(&AutowareInterface::motor_can_data_callback, this, std::placeholders::_1));

    // timer
    timer_ = this->create_wall_timer(10ms, std::bind(&AutowareInterface::TimerCallback, this));
}

void AutowareInterface::interface_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 513) // 201
    {
        float speed_data = msg->data[5] << 8 | msg->data[4];
        
        speed_data *= 0.01;
        autoware_vehicle_msgs::msg::VelocityReport AW_velocity_status_msg;  
        AW_velocity_status_msg.header.stamp = this->now();
        AW_velocity_status_msg.header.frame_id = "base_link";
        AW_velocity_status_msg.longitudinal_velocity = speed_data;

        std_msgs::msg::Float64 TC_velocity_msg;
        TC_velocity_msg.data = speed_data;

        this->get_parameter("Dyno_mode", dyno_mode_);
        if(!dyno_mode_)
        {
            // sim_speed_ = speed_data;
            AW_velocity_pub_->publish(AW_velocity_status_msg);
            TC_velocity_status_pub_->publish(TC_velocity_msg);
        }
    }

    if(msg->id == 273) // 111
    {
        float steer_data = msg->data[0] + (msg->data[1] << 8); 

        if(steer_data > 65535)
        {
            steer_data -= 65535;
        }
        
        steer_data -= 5200.f;
        steer_data *= 0.01071f;
        steer_data /= RAD2DEG;
        steer_angle_ = steer_data;

        autoware_vehicle_msgs::msg::SteeringReport steering_msg;    
        steering_msg.stamp = this->now();
        steering_msg.steering_tire_angle = steer_data;
        AW_steer_angle_pub_->publish(steering_msg);

        std_msgs::msg::Float64 steering_status_msg;
        steering_status_msg.data = steer_data;
        TC_steer_status_pub_->publish(steering_status_msg);
    }

    if (msg->id == 321) // 141 vehicle_mode (8bit)
    {
        uint8_t vehicle_status = msg->data[4];
        bool vehicle_mode = 0;
        uint8_t control_mode = 0;

        if (vehicle_status == MANULAL)
        {
            vehicle_mode = false;
            control_mode = 0;
        }
        else if (vehicle_status == AUTONOMOUS)
        {
            vehicle_mode = true;
            control_mode = 1;
        }

        std_msgs::msg::Bool vehicle_status_msg;
        vehicle_status_msg.data = vehicle_mode;
        autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
        control_mode_msg.mode = control_mode;
        interface_vehicle_status_pub_->publish(vehicle_status_msg);
        AW_control_mode_pub_->publish(control_mode_msg);
    }

}

void AutowareInterface::motor_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id == 402724847) //  Motor_control id 180117EF
    {
        double motor_rpm_raw = msg->data[7] << 8 | msg->data[6];
        double motor_rpm = motor_rpm_raw - 32000;
        
        double vehicle_speed_data = motor_rpm * MY_PI * WHEEL_DIAMETER / GEAR_RATIO;
        vehicle_speed_data /= 60; //m/s
        velocity_ = vehicle_speed_data;

        autoware_vehicle_msgs::msg::VelocityReport AW_velocity_status_msg;
        AW_velocity_status_msg.header.stamp = this->now();
        AW_velocity_status_msg.header.frame_id = "base_link";
        AW_velocity_status_msg.longitudinal_velocity = vehicle_speed_data;
        
        std_msgs::msg::Float64 TC_motor_velocity_msg;
        TC_motor_velocity_msg.data = vehicle_speed_data;

        this->get_parameter("Dyno_mode", dyno_mode_);
        if(dyno_mode_)
        {
            // sim_speed_ = vehicle_speed_data;
            AW_velocity_pub_->publish(AW_velocity_status_msg);
            TC_motor_velocity_status_pub_->publish(TC_motor_velocity_msg);
        }
    }
}

void AutowareInterface::AwCmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)   
{
    std_msgs::msg::Float64 TC_velocity_cmd_msg;
    std_msgs::msg::Float64 TC_steer_cmd_msg;

    TC_velocity_cmd_msg.data = msg->longitudinal.velocity;
    TC_steer_cmd_msg.data = msg->lateral.steering_tire_angle;

    TC_velocity_cmd_pub_->publish(TC_velocity_cmd_msg);
    TC_steer_cmd_pub_->publish(TC_steer_cmd_msg);
}

void AutowareInterface::TCthrottle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    TC_throttle_output_cmd_ = msg->data;
}

void AutowareInterface::TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg)
{   
    TC_brake_output_cmd_ = msg->data;
}

void AutowareInterface::TCsteer_callback(const std_msgs::msg::Int16::SharedPtr msg)
{   
    TC_steer_output_cmd_ = msg->data;
}

void AutowareInterface::TimerCallback()
{
    uint8_t throttle_can = static_cast<uint8_t>(std::clamp(TC_throttle_output_cmd_ * SPEEDCMD2SIG, 0.0, 255.0));
    uint8_t brake_can = static_cast<uint8_t>(std::clamp(TC_brake_output_cmd_ * SPEEDCMD2SIG, 0.0, 255.0));

    can_msgs::msg::Frame can_data;
    can_data.id = 320;
    can_data.dlc = 4;
    can_data.data[0] = throttle_can;
    can_data.data[1] = 0;
    can_data.data[2] = brake_can;
    can_data.data[3] = 1;
    // can_data.data[4] = ;    //gear command
    
    interface_can_pub_->publish(can_data);

    can_msgs::msg::Frame can_data1;

    const int8_t TC_steer_output_cmd_1 = (int8_t)((TC_steer_output_cmd_ & 0xFF00) >> 8);
    const int8_t TC_steer_output_cmd_2 = (int8_t)(TC_steer_output_cmd_ & 0x00FF);

    can_data1.id = 666;
    can_data1.dlc = 2;
    can_data1.data[0] = TC_steer_output_cmd_1;
    can_data1.data[1] = TC_steer_output_cmd_2;
    interface_can_pub_->publish(can_data1);

    autoware_control_msgs::msg::Control control_msg;
    control_msg.lateral.steering_tire_angle = steer_angle_;
    control_msg.longitudinal.velocity = velocity_;
    control_cmd_pub_->publish(control_msg);
}
} // namespace autoware_interface_ns
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_interface_ns::AutowareInterface)