#include "skid_steer_firmware/skid_steer_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
namespace skid_steer_firmware{
SkidsteerInterface::SkidsteerInterface(){

}

SkidsteerInterface::~SkidsteerInterface(){
    if(esp32S3.IsOpen()){
        try
        {
            esp32S3.Close();    
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("SkidsteerInterface"), "Something went wrong while disconecting with the port "<< port);
        }
        
    }
}

    CallbackReturn SkidsteerInterface::on_init(const hardware_interface::HardwareInfo & hardware_info){
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS){
            return result;
        }
        try
        {
            port = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("SkidsteerInterface"), "No serial port provided. Aborting task "<< port);
            return CallbackReturn::FAILURE;
        }
        velocity_commands.resize(info_.joints.size(), 0.0);
        position_states.resize(info_.joints.size(), 0.0);
        velocity_states.resize(info_.joints.size(), 0.0);
        velocity_commands.reserve(info_.joints.size());
        position_states.reserve(info_.joints.size());
        velocity_states.reserve(info_.joints.size());
        last_run = rclcpp::Clock().now();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("SkidsteerInterface"), "Joint config:");
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("SkidsteerInterface"),
            "Joint " << i << ": " << info_.joints[i].name << 
            " | Vel Cmd: " << velocity_commands[i] << 
            " | Pos: " << position_states[i] << 
            " | Vel: " << velocity_states[i]
        );
    }

        return CallbackReturn::SUCCESS;
    }
    std::vector<hardware_interface::StateInterface> SkidsteerInterface::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION, &position_states[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_states[i]));
        }
        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> SkidsteerInterface::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++){
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_commands[i]));
        }
        return command_interfaces;
    }
    CallbackReturn SkidsteerInterface::on_activate(const rclcpp_lifecycle::State &previous_state){
        RCLCPP_DEBUG(
            rclcpp::get_logger("SkidsteerInterface"), 
            "Activating from state: %s", 
            previous_state.label().c_str()
        );
        rclcpp::get_logger("SkidsteerInterface").set_level(rclcpp::Logger::Level::Debug);
        RCLCPP_INFO(rclcpp::get_logger("SkidsteerInterface"), "Starting robot hardware...");
        for(size_t i = 0; i < info_.joints.size(); i++){
        velocity_commands[i] = 0;
        position_states[i] = 0;
        velocity_states[i] = 0;
        }
        try
        {
            esp32S3.Open(port);
            esp32S3.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
            esp32S3.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            esp32S3.FlushIOBuffers();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("SkidsteerInterface"), "Something went wrong while interacting with port "<< port);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("SkidsteerInterface"), "hardware started, ready to command");
        return CallbackReturn::SUCCESS;
    }
    
    CallbackReturn SkidsteerInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){
        RCLCPP_DEBUG(
            rclcpp::get_logger("SkidsteerInterface"), 
            "Activating from state: %s", 
            previous_state.label().c_str()
        );
        RCLCPP_INFO(rclcpp::get_logger("SkidsteerInterface"), "Stoping robot hardware...");
        if (esp32S3.IsOpen()){
          try {
            std::string stop_command = "rp00.00,ln00.00,\n";
            esp32S3.Write(stop_command);
            esp32S3.FlushOutputBuffer();     
            esp32S3.Close();
            }
            catch(const std::exception &e) {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("SkidsteerInterface"), 
                "Error in destructor: " << e.what());
            }
            catch(...) {
                RCLCPP_FATAL(rclcpp::get_logger("SkidsteerInterface"), 
                "Unknown error in destructor");
            }
        }
        return CallbackReturn::SUCCESS;   
    }
    hardware_interface::return_type SkidsteerInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    if(esp32S3.IsDataAvailable()) {
        try {
            auto dt = (rclcpp::Clock().now() - last_run).seconds();
            std::string message;
            esp32S3.ReadLine(message);
            
            // Debug: Show message
            // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("SkidsteerInterface"), "Message recibed: '" << message << "'");
            
            std::stringstream ss(message);
            std::string response;
            
            while (std::getline(ss, response, ',')) {
                if(response.empty()) continue;
                //(ex: "rp12.34" Atleast 4 char)
                if(response.size() < 5) {
                    // RCLCPP_WARN_STREAM(rclcpp::get_logger("SkidsteerInterface"), 
                    //                   "Segment too short: '" << response );
                    continue;
                }
                try {
                    char wheel_side = response.at(0);  // 'r' o 'l'
                    char pos = response.at(1);         // 'f', 'm' or 'b'
                    char dir = response.at(2);         // 'p' or 'n'
                    double velocity = std::stod(response.substr(3)) * (dir == 'p' ? 1 : -1);
                    
                    // Debug: parsing
                    // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("SkidsteerInterface"), 
                    //                    "Parsing: " << wheel_side << pos << dir << " = " << velocity);
                    
                    // Vel & Pos
                    if(wheel_side == 'r') {
                        if(pos == 'b') { velocity_states.at(2) = velocity; position_states.at(2) += velocity * dt; }
                        else if(pos == 'f') { velocity_states.at(0) = velocity; position_states.at(0) += velocity * dt; }
                        else if(pos == 'm') { velocity_states.at(1) = velocity; position_states.at(1) += velocity * dt; }
                    } 
                    else if(wheel_side == 'l') {
                        if(pos == 'b') { velocity_states.at(5) = velocity; position_states.at(5) += velocity * dt; }
                        else if(pos == 'f') { velocity_states.at(3) = velocity; position_states.at(3) += velocity * dt; }
                        else if(pos == 'm') { velocity_states.at(4) = velocity; position_states.at(4) += velocity * dt; }
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SkidsteerInterface"), 
                                       "Error processing segment '" << response << "': " << e.what());
                }
            }
            last_run = rclcpp::Clock().now();
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("SkidsteerInterface"), 
                               "Error with serial lecture: " << e.what());
        }
    }
    return hardware_interface::return_type::OK;
}
    
    hardware_interface::return_type SkidsteerInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands.at(3) >= 0 ? 'p' : 'n';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if(std::abs(velocity_commands.at(0)) < 10.0){
            compensate_zeros_right = "0";
        } else{
            compensate_zeros_right = "";
        }
        if(std::abs(velocity_commands.at(3)) < 10.0){
            compensate_zeros_left = "0";
        } else{
            compensate_zeros_left = "";
        }
        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands.at(0)) <<
            ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands.at(3)) << ",\n";
        try{
            esp32S3.Write(message_stream.str());
            esp32S3.FlushOutputBuffer();
        }
        catch(...){
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("SkidsteerInterface"), "Something went wrong while sending the message " << 
            message_stream.str() << " on the port " << port);
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(skid_steer_firmware::SkidsteerInterface, hardware_interface::SystemInterface);

