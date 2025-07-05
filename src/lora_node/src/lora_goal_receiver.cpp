#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <slam_toolbox/srv/save_map.hpp>  
#include <libserial/SerialPort.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class LoraGoalReceiver : public rclcpp::Node {
public:
    LoraGoalReceiver() : Node("lora_goal_receiver") {
        // 
        try {
            serial_port_.Open("/dev/ttyUSB1");
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_.FlushIOBuffers();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }
        // 
        map_client_ = this->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

        // 
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        //
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LoraGoalReceiver::readLoraData, this)
        );

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for LoRa goal messages...");

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10,  
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            current_pose_ = msg;
            });
    }
    ~LoraGoalReceiver() {
        if (serial_port_.IsOpen()) {
            sendSerialMessage("CC");
            serial_port_.Close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed.");
        }
    }
private:
    void readLoraData() {
    if (!serial_port_.IsOpen()) return;

    if (serial_port_.IsDataAvailable()) {
        try {
            std::string raw_message;
            serial_port_.ReadLine(raw_message);

            // Limpiar caracteres de nueva línea
            raw_message.erase(std::remove(raw_message.begin(), raw_message.end(), '\r'), raw_message.end());
            raw_message.erase(std::remove(raw_message.begin(), raw_message.end(), '\n'), raw_message.end());

            RCLCPP_DEBUG(this->get_logger(), "Raw message received: '%s'", raw_message.c_str());

            

            // Comando de guardar mapa
            if (raw_message == "save_map") {
                RCLCPP_INFO(this->get_logger(), "Saving map");
                sendSerialMessage("MS");
                saveMap("/home/chocho1/maps/map_saved");
                return;
            }

            // Parseo de meta tipo x00.00/y00.00/w00.00
            std::stringstream ss(raw_message);
            std::string segment;
            float x = 0.0f, y = 0.0f, theta = 0.0f, h = -1.0f, m = -1.0f;
            bool parse_success = true;
            bool return_to_origin = false; 
            while (std::getline(ss, segment, '/')) {
                if (segment.empty()) continue;

                try {
                    char prefix = segment[0];
                    float value = std::stof(segment.substr(1));

                    switch (prefix) {
                        case 'x': x = value; break;
                        case 'y': y = value; break;
                        case 'w': theta = value; break;
                        case 'h': h = value; break;
                        case 'm': m = value; break;
                        default:
                            RCLCPP_WARN(this->get_logger(), "Unknown prefix '%c' in segment: %s", prefix, segment.c_str());
                            parse_success = false;
                            break;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing segment '%s': %s", segment.c_str(), e.what());
                    parse_success = false;
                }
                // Comandos de movimiento manual
                if (segment.length() == 1) {
                    char command = segment[0];
                    switch (command) {
                        case 'q':
                            RCLCPP_INFO(this->get_logger(), "Comando: Adelante");
                            publishVelocity(0.5, 0.0);
                            return;
                        case 's':
                            RCLCPP_INFO(this->get_logger(), "Comando: Atrás");
                            publishVelocity(-0.5, 0.0);
                            return;
                        case 'a':
                            RCLCPP_INFO(this->get_logger(), "Comando: Izquierda");
                            publishVelocity(0.0, 1.0);
                            return;
                        case 'd':
                            RCLCPP_INFO(this->get_logger(), "Comando: Derecha");
                            publishVelocity(0.0, -1.0);
                            return;
                        case 'x':
                            RCLCPP_INFO(this->get_logger(), "Comando: Parar");
                            publishVelocity(0.0, 0.0);
                            return;
                        default:
                            RCLCPP_WARN(this->get_logger(), "Comando desconocido: %c", command);
                            return;
                    }
                }
            }
            
            if (h >= 0 && m >= 0 && h == 0 && m <= 20) {
                    RCLCPP_WARN(this->get_logger(), "¡Batería baja! Regresando al origen (h=%.0f, m=%.0f)", h, m);
                    return_to_origin = true;
                    x = 0.0f;  // Sobrescribe las coordenadas
                    y = 0.0f;
                    theta = 0.0f;
            }
            
            if (parse_success) {
                RCLCPP_INFO(this->get_logger(), "Goal received - X: %.2f, Y: %.2f, θ: %.2f", x, y, theta);
                sendGoal(x, y, theta, return_to_origin);
            } else {
                RCLCPP_WARN(this->get_logger(), "Partial/invalid message received: %s", raw_message.c_str());
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        }
    }
}


    void sendGoal(float x, float y, float theta, bool is_origin = false) {
        if (is_origin) {           
            nav_to_pose_client_->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "Enviando robot al ORIGEN (batería baja)");
        }
        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "El servidor de acción no está disponible.");
            return;
        }

        theta = theta * M_PI / 180.0;  // grados a radianes

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        goal_msg.pose.pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), "Enviando meta: (%.2f, %.2f, θ=%.2f rad)", x, y, theta);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this, x, y, theta](const GoalHandleNavigate::WrappedResult & result) {
        std::ostringstream oss;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Meta alcanzada.");
                oss << std::fixed << std::setprecision(2)
                    << "AUX" << x 
                    << "Y" << y 
                    << "W" << (theta * 180.0 / M_PI);
                break;

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Meta abortada.");
                if (current_pose_) {
                    auto& pose = current_pose_->pose.pose;
                    double yaw = tf2::getYaw(pose.orientation);  // Convierte cuaternión a ángulo
                    oss << std::fixed << std::setprecision(2)
                        << "AUE1X" << pose.position.x 
                        << "Y" << pose.position.y 
                        << "W" << (yaw * 180.0 / M_PI);  // En grados
                } else {
                    oss << "AUE1";
                }
                break;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Meta cancelada.");
                if (current_pose_) {
                    auto& pose = current_pose_->pose.pose;
                    double yaw = tf2::getYaw(pose.orientation);
                    oss << std::fixed << std::setprecision(2)
                        << "AUE2X" << pose.position.x 
                        << "Y" << pose.position.y 
                        << "W" << (yaw * 180.0 / M_PI);
                } else {
                    oss << "AUE2";
                }
                break;

            default:
                RCLCPP_WARN(this->get_logger(), "Resultado desconocido.");
                if (current_pose_) {
                    auto& pose = current_pose_->pose.pose;
                    double yaw = tf2::getYaw(pose.orientation);
                    oss << std::fixed << std::setprecision(2)
                        << "AUE3X" << pose.position.x 
                        << "Y" << pose.position.y 
                        << "W" << (yaw * 180.0 / M_PI);
                } else {
                    oss << "AUE3";
                }
                break;
        }
            sendSerialMessage(oss.str());
        };
        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void sendSerialMessage(const std::string &message) {
        if (serial_port_.IsOpen()) {
            serial_port_.Write(message + "\n");
            RCLCPP_INFO(this->get_logger(), "Mensaje enviado por serial: %s", message.c_str());
        }
    }

    void saveMap(const std::string &map_path) {
        auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        request->name.data = map_path;

        if (!map_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Servicio de guardar mapa no disponible.");
            return;
        }

        auto result = map_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "¡Mapa guardado exitosamente!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error al guardar el mapa.");
        }
    }

    void publishVelocity(double linear, double angular) {
        if (cmd_vel_timer_ && cmd_vel_timer_->is_canceled() == false) {
            RCLCPP_DEBUG(this->get_logger(), "Comando ya en progreso. Ignorando...");
            return;
        }

        last_twist_.linear.x = linear;
        last_twist_.angular.z = angular;
        cmd_vel_pub_->publish(last_twist_);
        cmd_vel_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(cmd_duration_ms_),
            [this]() {
                geometry_msgs::msg::Twist stop_twist;
                stop_twist.linear.x = 0.0;
                stop_twist.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_twist);
                cmd_vel_timer_->cancel();
                RCLCPP_DEBUG(this->get_logger(), "Comando de velocidad terminado");
            }
        );
    }


    //
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
    geometry_msgs::msg::Twist last_twist_;
    int cmd_duration_ms_ = 3000; 
    //
    LibSerial::SerialPort serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    //
    rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr map_client_; 
    //
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    //
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoraGoalReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}







