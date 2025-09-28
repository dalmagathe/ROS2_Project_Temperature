#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <mutex>

using std::placeholders::_1;

class Syst_Temp : public rclcpp::Node {
public:
    Syst_Temp() : Node("tpr_node") 
    {
        // Create a publisher that will get the temperature through get_tpr_topic
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("get_tpr_topic", 10);

        // Subscribe to control_topic
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "control_topic", 10, std::bind(&Syst_Temp::topic_callback, this, _1));

        // ROS timer to get the current temperature every 200ms
        this->timer_ = this->create_wall_timer
        (
            std::chrono::milliseconds(200),
            // To link the member fct callback timer_callback to the object this
            std::bind(&Syst_Temp::timer_callback, this)
        );
    }

    ~Syst_Temp() override 
    {
        RCLCPP_INFO(this->get_logger(), "Temperature is not read anymore");
    }

private:
    // Send the temperature of the room
    void timer_callback() 
    {
        this->mtx.lock();
        // Simulate a new temperature
        this->temperature += 0.1 * ((rand() % 20 - 10) / 10.0);
        RCLCPP_INFO(this->get_logger(), "Temperature of the room = %.2f", this->temperature);
        auto message = std_msgs::msg::Float64();
        message.data = this->temperature;
        this->mtx.unlock();
        publisher_->publish(message);
    }

    // Get the PID and add it to the temperature so it simulates a controller impact 
    void topic_callback(const std_msgs::msg::Float64 & msg)
    {
        // Get the temperature
        float new_controller = msg.data;
        RCLCPP_INFO(this->get_logger(), "PID received =%.2f", new_controller);
        this->mtx.lock();
        this->temperature += new_controller * 0.2;
        RCLCPP_INFO(this->get_logger(), "New temperature =%.2f", this->temperature);
        this->mtx.unlock();
    }

    float temperature = 20.0;
    std::mutex mtx;            // Mutex pour protéger la variable "temperature"
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    // Creation of the subscriber to obtain the controller result and add it to the temperature, as we do not have a system that allows us to modify it directly. 
    // Otherwise, the controller would have been sent to the system rather than here.
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) 
{
    // Start ROS
    rclcpp::init(argc, argv);
    // Create a node ROS2 of type Syst_Temp
    auto node = std::make_shared<Syst_Temp>();
    // ROS loop
    rclcpp::spin(node);
    // Stop ROS
    rclcpp::shutdown();
    return 0;
}
