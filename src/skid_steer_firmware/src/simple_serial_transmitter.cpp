#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

using std::placeholders::_1;
class SimpleSerialTransmitter : public rclcpp::Node{
public:
    SimpleSerialTransmitter() : Node("Simple_serial_transmitter"){
       declare_parameter<std::string>("port", "/dev/ttyACM0");
       port = get_parameter("port").as_string();
       esp32S3.Open(port);
       esp32S3.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

       sub = create_subscription<std_msgs::msg::String>("serial_transmitter", 10, std::bind(&SimpleSerialTransmitter::MsgCallback, this, _1));
    }

    ~SimpleSerialTransmitter(){
        esp32S3.Close();
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    LibSerial::SerialPort esp32S3;
    std::string port;
    
    void MsgCallback(const std_msgs::msg::String &msg){
        esp32S3.Write(msg.data);
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialTransmitter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}