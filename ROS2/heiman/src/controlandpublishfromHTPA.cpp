#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <memory>
#include <heiman/globals.h>

#include "../include/heiman/globals.h"

class HTPAControlAndPublishNode : public rclcpp::Node
{
public:
    HTPAControlAndPublishNode() : Node("HTPAControlAndPublishNode")
    {
        // Socket setup for HTPA
        setup_htpa_socket();

        // Initialize HTPA
        if (initialize_htpa())
        {
            RCLCPP_INFO(this->get_logger(), "HTPA Initialized Successfully");

            // Start HTPA after successful initialization
            if (start_htpa())
            {
                RCLCPP_INFO(this->get_logger(), "HTPA Started Successfully");
                start_publishing(); // Start publishing data
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to Start HTPA");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to Initialize HTPA");
        }
    }

private:
    void setup_htpa_socket()
    {
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        // Setting up HTPA address
        memset(&HTPA_addr_, 0, sizeof(HTPA_addr_));
        HTPA_addr_.sin_family = AF_INET;
        HTPA_addr_.sin_port = htons(PORT_HTPA);
        HTPA_addr_.sin_addr.s_addr = inet_addr(IP_HTPA);

        bind(sock_, (struct sockaddr *)&HTPA_addr_, sizeof(HTPA_addr_));
    }

    bool initialize_htpa()
    {
        bool connected = false;
        int attempt = 0;
        const int max_attempts = 5;  // Maximum number of attempts
        const int retry_interval_ms = 1000;  // Time between retries in milliseconds

        while (!connected && attempt < max_attempts)
        {
            char text[50];
            strcpy(text, "Bind HTPA series device");

            int bytes_sent = sendto(sock_, text, strlen(text), 0, (struct sockaddr *)&HTPA_addr_, sizeof(HTPA_addr_));
            if (bytes_sent != -1)
            {
                // Optionally, check for a response from the device here

                connected = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send message, retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
            }

            attempt++;
        }

        if (!connected) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect after %d attempts.", max_attempts);
        }

        return connected;
    }


    bool start_htpa()
    {
        // Setting up PC address
        memset(&PC_addr_, 0, sizeof(PC_addr_));
        PC_addr_.sin_family = AF_INET;
        PC_addr_.sin_port = htons(PORT_PC);
        PC_addr_.sin_addr.s_addr = inet_addr(IP_PC);

        bool started = false;
        int attempt = 0;
        const int max_attempts = 5;  // Maximum number of attempts
        const int retry_interval_ms = 1000;  // Time between retries in milliseconds

        while (!started && attempt < max_attempts)
        {
            char text = 'K';
            int bytes_sent = sendto(sock_, &text, 1, 0, (struct sockaddr *)&HTPA_addr_, sizeof(HTPA_addr_));
            if (bytes_sent != -1)
            {
                // Optionally, check for a response from the device here

                started = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send start command, retrying...");
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
            }

            attempt++;
        }

        if (!started) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start HTPA after %d attempts.", max_attempts);
        }

        return started;
    }


    void start_publishing()
    {
        auto self = this->get_node_base_interface();
        HTPAoutput_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("HTPAoutput", 10);

        while (rclcpp::ok())
        {
            std_msgs::msg::UInt8MultiArray msg;
            
            char* stbis = (char*)malloc(MTU * sizeof(char));
            int iRc = recvfrom(sock_, stbis, MTU * sizeof(char), 0, NULL, NULL);
            if (iRc > 0)
            {
                msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                msg.layout.dim[0].size = iRc;
                msg.layout.dim[0].stride = iRc;

                msg.data.resize(iRc);
                for (int ii = 0; ii < iRc; ii++)
                    msg.data[ii] = (unsigned char)stbis[ii];
                
                HTPAoutput_pub_->publish(msg);
            }

            rclcpp::spin_some(self);  // Use the node base interface shared pointer
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
            free(stbis);
        }
    }

    int sock_; // Socket for communication
    struct sockaddr_in HTPA_addr_; // Server-side data structure for HTPA
    struct sockaddr_in PC_addr_; // Client-side data structure for PC
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr HTPAoutput_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HTPAControlAndPublishNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
