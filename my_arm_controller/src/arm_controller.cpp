#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node {
public:
  // Constructor: Names the node and initializes publisher/timer
  ArmController() : Node("arm_controller") {
    // Create a publisher on the "joint_commands" topic with a queue size of 10
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_commands", 10);
    
    // Trigger the callback function every 500 milliseconds (2Hz)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ArmController::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::Float64MultiArray();
    
    // Define 7 dummy joint angles (in radians) for our 7-DOF arm
    message.data = {0.0, 0.5, -0.2, 0.0, 1.1, 0.0, 0.0}; 
    
    RCLCPP_INFO(this->get_logger(), "Publishing 7-DOF joint commands");
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  // Spin keeps the node alive and listening for events (like our timer)
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}
