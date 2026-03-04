#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

class ControlNode : public rclcpp::Node
{
public:
  
  // Constructeur correct : même nom que la classe
  ControlNode();

private:
  // callback pour traiter les messages de detection yolo
  void control_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void control_timer_callback();
  // suscriber por les messages de detection yolo 
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr target_;
  // publisher pour les commandes de vitesse
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
 
  int x_center_;  // centre de l'image en x
  double height_target_; // vrai hauteur de la cible
  double desired_height_; // la hauteur désirée de la cible dans l'image
  double lamda_; // gain proportionnel pour la commande 
  double fx_; // paramètre intrinsèque de la caméra
  double fy_; // paramètre intrinsèque de la caméra
  double cx_; // paramètre intrinsèque de la caméra
  double cy_; // paramètre intrinsèque de la caméra
  double Vmax; // vitesse maximale du robot 
  double filter_alpha_;
  double timeout_sec_;
  double control_rate_hz_;

  bool has_detection_;
  double x_f_;
  double h_f_;
  rclcpp::Time last_detection_time_;
};

#endif  // CONTROL_NODE_HPP_
