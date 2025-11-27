/**
 * Created: Nov. 9, 2025
 * Author(s): Obaid
 */

#ifndef AP1_SIM_NODE_HPP
#define AP1_SIM_NODE_HPP

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

namespace ap1::pnc_sim
{

class SimpleVehicleSimNode : public rclcpp::Node
{
public:
    SimpleVehicleSimNode()
        : Node("simple_vehicle_sim_node"),
          current_throttle_(0.0f),
          current_steer_rad_(0.0f)
    {
        cfg_.mass_kg = 1200.0f;
        cfg_.max_force_n = 4000.0f;
        cfg_.drag_coeff = 0.5f;
        cfg_.wheelbase_m = 2.5f;
        cfg_.max_steer_rad = 0.5f;

        // Initial state
        state_.x_m = 0.0f;
        state_.y_m = 0.0f;
        state_.yaw = 0.0f;
        state_.v_mps = 0.0f;

        last_update_ = this->now();

        // ------------------- SUBSCRIBERS -------------------
        throttle_sub_ = this->create_subscription<ap1_msgs::msg::MotorPowerStamped>(
            "/ap1/control/motor_power", 10,
            std::bind(&SimpleVehicleSimNode::on_throttle, this, std::placeholders::_1));

        steer_sub_ = this->create_subscription<ap1_msgs::msg::TurnAngleStamped>(
            "/ap1/control/turn_angle", 10,
            std::bind(&SimpleVehicleSimNode::on_steer, this, std::placeholders::_1));

        // ------------------- PUBLISHERS -------------------
        speed_pub_ = this->create_publisher<ap1_msgs::msg::VehicleSpeedStamped>(
            "/ap1/actuation/speed_actual", 10);

        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/ap1/actuation/turn_angle_actual", 10);

        // ------------------- TIMER -------------------
        using namespace std::chrono_literals;
        sim_timer_ = this->create_wall_timer(
            20ms,
            std::bind(&SimpleVehicleSimNode::on_timer, this));

        // ------------------- RESET SERVICE -------------------
        reset_sim_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/pnc_sim/reset",
            std::bind(&SimpleVehicleSimNode::handle_reset,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Simple Vehicle Sim Node initialized");
    }

    // ------------------------------------------------------
    // 🔥 PUBLIC RESET FUNCTION USED BY THE PYTHON UI
    // ------------------------------------------------------
    bool reset_all(std::string &msg_out)
    {
        reset_simulation();
        msg_out = "✓ PNC Sim reset";
        return true;
    }

private:
    // ------------------- STRUCTS -------------------
    struct CarConfig
    {
        float mass_kg;
        float max_force_n;
        float drag_coeff;
        float wheelbase_m;
        float max_steer_rad;
    };

    struct CarState
    {
        float x_m;
        float y_m;
        float yaw;
        float v_mps;
    };

    // ------------------- CALLBACKS -------------------
    void on_throttle(const ap1_msgs::msg::MotorPowerStamped::SharedPtr msg)
    {
        current_throttle_ = msg->power;
        current_throttle_ = std::clamp(current_throttle_, 0.0f, 1.0f);
    }

    void on_steer(const ap1_msgs::msg::TurnAngleStamped::SharedPtr msg)
    {
        current_steer_rad_ = std::clamp(msg->angle,
                                        -cfg_.max_steer_rad,
                                        cfg_.max_steer_rad);
    }

    void on_timer()
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_update_).seconds();
        last_update_ = now;

        if (dt <= 0.0)
            dt = 0.02;

        step_dynamics(static_cast<float>(dt));
        publish_state();
    }

    void step_dynamics(float dt)
    {
        float engine_force = current_throttle_ * cfg_.max_force_n;
        float drag_force = cfg_.drag_coeff * state_.v_mps;

        float accel = (engine_force - drag_force) / cfg_.mass_kg;

        state_.v_mps = std::max(0.0f, state_.v_mps + accel * dt);

        float beta = std::tan(current_steer_rad_);
        float yaw_rate = (state_.v_mps / cfg_.wheelbase_m) * beta;
        state_.yaw += yaw_rate * dt;

        state_.x_m += state_.v_mps * std::cos(state_.yaw) * dt;
        state_.y_m += state_.v_mps * std::sin(state_.yaw) * dt;
    }

    void publish_state()
    {
        ap1_msgs::msg::VehicleSpeedStamped s;
        s.speed = state_.v_mps;
        speed_pub_->publish(s);

        geometry_msgs::msg::Point p;
        p.x = state_.x_m;
        p.y = state_.y_m;
        p.z = 0.0;
        position_pub_->publish(p);
    }

    // ------------------- SIM RESET -------------------
    void reset_simulation()
    {
        state_.x_m = 0.0f;
        state_.y_m = 0.0f;
        state_.yaw = 0.0f;
        state_.v_mps = 0.0f;

        current_throttle_ = 0.0f;
        current_steer_rad_ = 0.0f;

        last_update_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Simulation state reset to zero");
    }

    void handle_reset(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        reset_simulation();
        response->success = true;
        response->message = "Simulation reset";
    }

    // ------------------- MEMBER VARIABLES -------------------
    CarConfig cfg_;
    CarState state_;

    float current_throttle_;
    float current_steer_rad_;

    rclcpp::Subscription<ap1_msgs::msg::MotorPowerStamped>::SharedPtr throttle_sub_;
    rclcpp::Subscription<ap1_msgs::msg::TurnAngleStamped>::SharedPtr steer_sub_;
    rclcpp::Publisher<ap1_msgs::msg::VehicleSpeedStamped>::SharedPtr speed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_sim_service_;
    rclcpp::Time last_update_;
};

} // namespace ap1::pnc_sim

#endif // AP1_SIM_NODE_HPP
