#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
using namespace std::chrono_literals;
class SimplePublisher : public rclcpp::Node{
public:
    SimplePublisher() : Node("Simple_Publisher"), counter{0}
    {
        pub = create_publisher<std_msgs::msg::String>("Chatter", 10);
        timer = create_wall_timer(1s, std::bind(&SimplePublisher::TimerCallback, this));
        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

private:
    unsigned int counter;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    void TimerCallback(){
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS2 - counter: " + std::to_string(counter++);
        pub->publish(message);
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}