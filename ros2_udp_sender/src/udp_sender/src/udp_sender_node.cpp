#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define AUTOPILOT_IP "172.18.0.2"  // destination ip
#define UDP_PORT 6005              // Port of communication

class UDPSenderNode : public rclcpp::Node {
public:
    UDPSenderNode() : Node("udp_sender_node") {
        pub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/vehicle/cmd", 10,
            std::bind(&UDPSenderNode::send_udp_message, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "UDP sender node initialized.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pub_;

    void send_udp_message(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error creating UDP socket.");
            return;
        }

        sockaddr_in autopilot_addr;
        autopilot_addr.sin_family = AF_INET;
        autopilot_addr.sin_port = htons(UDP_PORT);
        inet_pton(AF_INET, AUTOPILOT_IP, &autopilot_addr.sin_addr);

        // Prepare message with 16 bytes (8 bytes double for steering + 8 bytes double for acceleration)
        char buffer[16];
        double steering = msg->angular.z;   // Example: 0.01
        double acceleration = msg->linear.x;  // Example: 0.50

        std::memcpy(buffer, &steering, sizeof(double));
        std::memcpy(buffer + 8, &acceleration, sizeof(double));

        sendto(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&autopilot_addr, sizeof(autopilot_addr));

        RCLCPP_INFO(this->get_logger(), "Sent UDP message: Steering=%.2f, Acceleration=%.2f", steering, acceleration);

        close(sock);}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPSenderNode>());
    rclcpp::shutdown();
    return 0;
}
