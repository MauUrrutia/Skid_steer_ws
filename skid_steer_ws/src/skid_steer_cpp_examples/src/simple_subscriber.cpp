#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using std::placeholders::_1;
class SimpleSubscriber : public rclcpp::Node{
public:
    SimpleSubscriber() : Node("Simple_subscriber"){
       sub = create_subscription<std_msgs::msg::String>("Chatter", 10, std::bind(&SimpleSubscriber::MsgCallback, this, _1));
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    void MsgCallback(const std_msgs::msg::String &msg) const{
        RCLCPP_INFO_STREAM(get_logger(), "I heard3232: " << msg.data.c_str());
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}