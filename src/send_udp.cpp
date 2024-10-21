#include <cstdint>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "ros2_udp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#define PACKET_SIZE 32
#define F7_ADDR "192.168.3.60"
#define F7_PORT 45242

using namespace std::chrono;

class SendUDP : public rclcpp::Node
{
  public:
    SendUDP(const std::string& f7_address, int f7_port)
    : Node("send_udp"),f7_udp(f7_address,f7_port)    {

        size = request(packet ,  slmp_request , slmp_request.size());

        // Create the publisher for the received data
        udp_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("udp_received_data", 10);

        //タイマーで時間ごとにコールバックされる関数を設定
        timer_ = this->create_wall_timer(20ms, std::bind(&SendUDP::timer_callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr udp_pub_;  // Publisher
    Ros2UDP f7_udp;
    uint8_t packet[PACKET_SIZE];
    uint8_t response_buffer[128];  // Buffer for receiving the response
    int size;
    std::vector<uint8_t> slmp_request = {
    //Write from address 1(0x01 0x00), of D(0x08), three values 0x03. from 
    //0x01, 0x00, 0x00, 0xa8, 0x03, 0x00, 0x4e, 0x61, 0xbc, 0x00, 0x12, 0x34
    //Read from address 18(0x12 0x00), of D(0xa8), 16 words(0x10 0x00)
	0x12, 0x00, 0x00, 0xa8, 0x10, 0x00 
	};
    
    void timer_callback(){
        //送信処理(ros2_udp.cppで定義)
        f7_udp.send_packet(packet, sizeof(uint8_t)*PACKET_SIZE);
        //送信したデータを表示
        //RCLCPP_INFO(this->get_logger(),"send: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",packet[0],packet[1],packet[2],packet[3],packet[4],packet[5],packet[6],packet[7],packet[8],packet[9],packet[10],packet[11],packet[12],packet[13],packet[14],packet[15],packet[16],packet[17],packet[18],packet[19],packet[20],packet[21]);

        // Receive the UDP response
        try {
            int received_bytes = f7_udp.udp_recv(response_buffer, sizeof(response_buffer),1);
            if (received_bytes > 0) {
                RCLCPP_INFO(this->get_logger(), "Received %u bytes from UDP", received_bytes);

                // Publish the received data
                auto msg = std_msgs::msg::ByteMultiArray();
                msg.data.resize(received_bytes);
                std::copy(response_buffer, response_buffer + received_bytes, msg.data.begin());

                udp_pub_->publish(msg);  // Publish the message
            }
        }    
        catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to receive response: %s", e.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendUDP>(F7_ADDR,F7_PORT);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
