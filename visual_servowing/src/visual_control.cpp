#include "visual_control.hpp"

#include <cmath>
#include <Eigen/Dense>

using std::placeholders::_1;

ControlNode::ControlNode() : Node("control_node")
{
    this->declare_parameter<int>("x_center", 320);
    this->declare_parameter<double>("height_target", 0.63);
    this->declare_parameter<double>("desired_height", 150.0);
    this->declare_parameter<double>("lamda", 0.5);
    this->declare_parameter<double>("fx", 600.0);
    this->declare_parameter<double>("fy", 600.0);
    this->declare_parameter<double>("cx", 320.0);
    this->declare_parameter<double>("cy", 240.0);
    this->declare_parameter<double>("filter_alpha", 0.4);
    this->declare_parameter<double>("timeout_sec", 5.0);
    this->declare_parameter<double>("control_rate_hz", 20.0);
    this->declare_parameter<double>("Vmax",10.0);

    this->get_parameter("x_center", x_center_);
    this->get_parameter("height_target", height_target_);
    this->get_parameter("desired_height", desired_height_);
    this->get_parameter("lamda", lamda_);
    this->get_parameter("fx", fx_);
    this->get_parameter("fy", fy_);
    this->get_parameter("cx", cx_);
    this->get_parameter("cy", cy_);
    this->get_parameter("filter_alpha", filter_alpha_);
    this->get_parameter("timeout_sec", timeout_sec_);
    this->get_parameter("control_rate_hz", control_rate_hz_);
    this->get_parameter("Vmax",Vmax); 

    has_detection_ = false;
    x_f_ = 0.0;
    h_f_ = 0.0;
    last_detection_time_ = this->now();

    // Création du publisher pour les commandes de vitesse
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // Création du subscriber pour les messages de détection YOLO
    target_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "detections_output", 10, std::bind(&ControlNode::control_callback, this, _1));

    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ControlNode::control_timer_callback, this));
}

void ControlNode::control_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    if (!msg->detections.empty())
    {
        // Supposons que nous prenons la première détection
        auto detection = msg->detections[0];
        double u_px = detection.bbox.center.position.x;
        double h_px = detection.bbox.size_y;

        if (h_px <= 0.0) {
            return;
        }

        double x_norm = (u_px - cx_) / fx_;

        if (!has_detection_) {
            x_f_ = x_norm;
            h_f_ = h_px;
        } else {
            x_f_ = filter_alpha_ * x_norm + (1.0 - filter_alpha_) * x_f_;
            h_f_ = filter_alpha_ * h_px + (1.0 - filter_alpha_) * h_f_;
        }

        has_detection_ = true;
        last_detection_time_ = this->now();
    }
}

void ControlNode::control_timer_callback()
{
    geometry_msgs::msg::Twist twist_msg;

    if (!has_detection_) {
        vel_pub_->publish(twist_msg);
        return;
    }

    auto dt = (this->now() - last_detection_time_).seconds();
    if (dt > timeout_sec_) {
        vel_pub_->publish(twist_msg);
        return;
    }

    if (h_f_ <= 0.0 || fx_ == 0.0 || fy_ == 0.0) {
        vel_pub_->publish(twist_msg);
        return;
    }

    double x = x_f_;
    double h = h_f_;
    double H = static_cast<double>(height_target_);
    double log_h = std::log(h);
    double log_h_des = std::log(static_cast<double>(desired_height_));

    double e1 = x - 0.0;
    double e2 = log_h - log_h_des;

    Eigen::Matrix2d J;
    J(0, 0) = (h * x) / (fy_ * H);
    J(0, 1) = -(1.0 + x * x);
    J(1, 0) = (h) / (fy_ * H);
    J(1, 1) = -x;

    Eigen::Vector2d e;
    e << e1, e2;

    if (std::fabs(J.determinant()) < 1e-6) {
        vel_pub_->publish(twist_msg);
        return;
    }

    Eigen::Vector2d uv = -lamda_ * J.inverse() * e;
    double u_cmd = uv(0);
    double v_cmd = uv(1);
    // Saturer puis normaliser
    u_cmd = std::clamp(u_cmd, -Vmax, Vmax);
    v_cmd = std::clamp(v_cmd, -Vmax, Vmax);
    
    twist_msg.linear.x = u_cmd ;
    twist_msg.angular.z = v_cmd ;

    vel_pub_->publish(twist_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
