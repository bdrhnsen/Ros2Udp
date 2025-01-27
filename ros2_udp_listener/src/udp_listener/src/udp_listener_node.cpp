#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define UDP_PORT 5005
#define BUFFER_SIZE 1024

struct ObservationData {
    double ego_x;
    double ego_y;
    double ego_speed;
    struct {
        double x;
        double y;
        double speed;
    } surrounding_vehicles[5];
};

class UDPReceiver : public rclcpp::Node {
public:
    UDPReceiver() : Node("udp_receiver") {
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("udp_observation", 10);
        receive_udp();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

    void receive_udp() {
        int sockfd;
        sockaddr_in server_addr;
        socklen_t addr_len = sizeof(server_addr);
        ObservationData obs;

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(UDP_PORT);

        bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
        RCLCPP_INFO(this->get_logger(), "Listening for UDP messages...");

        while (rclcpp::ok()) {
            recvfrom(sockfd, &obs, sizeof(obs), 0, (struct sockaddr*)&server_addr, &addr_len);
            auto message = std_msgs::msg::Float64MultiArray();

            message.data.push_back(obs.ego_x);
            message.data.push_back(obs.ego_y);
            message.data.push_back(obs.ego_speed);
            for (const auto &veh : obs.surrounding_vehicles) {
                message.data.push_back(veh.x);
                message.data.push_back(veh.y);
                message.data.push_back(veh.speed);
            }

            pub_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Received UDP data: Ego x=%.2f y=%.2f speed=%.2f",
                        obs.ego_x, obs.ego_y, obs.ego_speed);
        }
        close(sockfd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPReceiver>());
    rclcpp::shutdown();
    return 0;
}
