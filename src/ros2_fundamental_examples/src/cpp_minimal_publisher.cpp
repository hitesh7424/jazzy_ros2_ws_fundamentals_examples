/**
 * 
 */

#include "rclcpp/rclcpp.hpp"       // ros 2 c++ client library
#include "std_msgs/msg/string.hpp" // standard message type for strings

using namespace std::chrono_literals; // handles time duration

class MinimalCppPublisher : public rclcpp::Node
{
public:
    MinimalCppPublisher() : Node("minimal_cpp_publisher"), count_(0)
    {
        publisher_ = create_publisher<std_msgs::msg::String>(
            "/cpp_example_topic", 10);

        timer_ = create_wall_timer(500ms, std::bind(&MinimalCppPublisher::timerCallback, this));

        // RCLCPP_INFO(get_logger(), "Publishing as 2Hz");


    }
    void timerCallback(){
        auto message = std_msgs::msg::String();
        message.data = "Hello World " + std::to_string(count_++);

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing as 2Hze: %s", message.data.c_str());
    }

private:
    size_t count_; // keep track nuber of messages published
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    publisher_; // publisher object
    rclcpp::TimerBase::SharedPtr timer_; // timer
    
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);

    auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>();

    rclcpp::spin(minimal_cpp_publisher_node);

    rclcpp::shutdown();
    return 0;
}