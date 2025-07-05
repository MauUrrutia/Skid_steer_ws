#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>
using namespace std::chrono_literals;
class SimpleSerialReceiver : public rclcpp::Node{
public:
    SimpleSerialReceiver() : Node("Simple_serial_receiver")
    {
        declare_parameter<std::string>("port", "/dev/ttyACM0");
        port = get_parameter("port").as_string();
        esp32S3.Open(port);
        esp32S3.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        pub = create_publisher<std_msgs::msg::String>("serial_receiver", 10);
        timer = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::TimerCallback, this));
        RCLCPP_INFO(get_logger(), "Publishing at 100 Hz");
    }

    ~SimpleSerialReceiver(){
        esp32S3.Close();
    }

    void TimerCallback(){

        auto message = std_msgs::msg::String();
        if(rclcpp::ok() && esp32S3.IsDataAvailable()){
            esp32S3.ReadLine(message.data);
        } 
        pub->publish(message);
    }

private:
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::string port;
    LibSerial::SerialPort esp32S3;

    
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}