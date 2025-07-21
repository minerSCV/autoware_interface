#ifndef AUTOWARE_INTERFACE__AUTOWARE_INTERFACE_HPP_
#define AUTOWARE_INTERFACE__AUTOWARE_INTERFACE_HPP_

#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_control_msgs/msg/control.hpp"


// motor revolution
#define MY_PI 3.141592
#define WHEEL_DIAMETER 0.51  // m
#define GEAR_RATIO 4.5

// steer angle
#define RAD2DEG 57.3
#define STEERCMD2SIG 242.552
#define SPEEDCMD2SIG 255.0
#define STEERCMD_OFFSET 127.0

// vehicle status
#define MANULAL 1
#define AUTONOMOUS 3

namespace autoware_interface_ns
{
class AutowareInterface : public rclcpp::Node 
{
    public:
        AutowareInterface(const rclcpp::NodeOptions & node_options);

    private:
        // Pub
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr AW_velocity_pub_;   
        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr AW_steer_angle_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr AW_control_mode_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_motor_velocity_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_steer_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_steer_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr interface_vehicle_status_pub_;
        rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;

        // Sub
        rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr AW_command_sub;
        rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr AW_control_mode_sub; 
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_throttle_cmd;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_brake_cmd;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr TC_steer_cmd;

        // CAN
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr interface_can_sub_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr interface_can_pub_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr motor_can_sub_;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;            
        
        float TC_throttle_output_cmd_ = 0.0;
        float TC_brake_output_cmd_ = 0.0;
        int16_t TC_steer_output_cmd_ = 0;
        bool dyno_mode_ = false;
        float steer_angle_ = 0.0;
        float velocity_ = 0.0;

        void interface_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg);
        void motor_can_data_callback(const can_msgs::msg::Frame::SharedPtr msg);
        void AwCmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg);
        void AwCtrlMod_callback(const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg);
        void TCthrottle_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void TCbrake_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void TCsteer_callback(const std_msgs::msg::Int16::SharedPtr msg);
        void TimerCallback();
};
}
#endif  // AUTOWARE_INTERFACE__AUTOWARE_INTERFACE_HPP_
