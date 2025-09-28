#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

// PID Controller
class PID {
private:
    double kp, ki, kd;
    double prev_error;
    double integral;

public:
    PID(double p, double i, double d) : kp(p), ki(i), kd(d), prev_error(0), integral(0) {}

    double compute(double setpoint, double measured_value, double dt) {
        double error = setpoint - measured_value;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class Compute_controller : public rclcpp::Node
{
public:
  Compute_controller(): Node("controller"), pid(0.5, 0, 0)
  {
    // Subscribe to get_tpr_topic
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "get_tpr_topic", 10, std::bind(&Compute_controller::topic_callback, this, _1));
    // publish to topic control_topic the new controller calculation
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("control_topic", 10);
  }

private:
  // The callback get the temperature and compute the PID
  void topic_callback(const std_msgs::msg::Float64 & msg)
  {
    // Get the temperature
    float current_temp = msg.data;
    RCLCPP_INFO(this->get_logger(), "Current temperature = '%f'", current_temp);
    
    // Compute delta time between 2 temperation to compute the PID 
    rclcpp::Time now = this->now();
    double dt = 0.2; // as it should be 200ms between 2 values
    if (last_time.nanoseconds() > 0) // skip first time
    {   
      dt = (now - last_time).seconds();
    }
    last_time = now;
    float control = pid.compute(100.0, current_temp, dt);
    RCLCPP_INFO(this->get_logger(), "PID result = '%f'", control);

    // Send the controller data to the actuator (which is here the tpr_node node because we don't have a real system)
    auto message = std_msgs::msg::Float64();
    message.data = control;
    publisher_->publish(message);
  }
  
  // Create a subscriber to get the current temperature
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  // Create a publisher to send the controller calculation to the systeme
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
  PID pid;
};

int main(int argc, char * argv[])
{
    // Start ROS
    rclcpp::init(argc, argv);
    // Create a node ROS2 of type Syst_Temp
    auto node = std::make_shared<Compute_controller>();
    // ROS loop
    rclcpp::spin(node);
    // Stop ROS
    rclcpp::shutdown();
    return 0;
}
