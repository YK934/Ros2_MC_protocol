#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>  // For std::hex and std::setw
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

using namespace std::chrono_literals;

class UDPHexSubscriber : public rclcpp::Node
{
public:
    UDPHexSubscriber()
    : Node("udp_hex_subscriber")
    {
        // Create a subscription to the udp_received_data topic
        subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "udp_received_data",
            10,
            std::bind(&UDPHexSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;

    // Function to swap byte order in the custom pattern
    void custom_swap(uint8_t *data, size_t length) {
        if (length == 4) {
            // Swap 4e 61 bc 00 → 00 bc 61 4e
            std::swap(data[0], data[3]);  // Swap first and fourth bytes
            std::swap(data[1], data[2]);  // Swap second and third bytes
        } else if (length == 4) {
            // Swap 12 34 4e 61 → 61 4e 34 12
            std::swap(data[0], data[3]);  // Swap first and fourth bytes
            std::swap(data[1], data[2]);  // Swap second and third bytes
        }
    }

    // Function to convert a 4-byte array into a 32-bit unsigned integer
    uint32_t convert_to_uint32(uint8_t *data) {
        return static_cast<uint32_t>(data[0]) << 24 |
               static_cast<uint32_t>(data[1]) << 16 |
               static_cast<uint32_t>(data[2]) << 8  |
               static_cast<uint32_t>(data[3]);
    }

    void topic_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        // Print received data in hexadecimal format
        std::ostringstream hex_stream;
        
        int byte_cnt = 0;
        hex_stream << "Raw Hex Data: ";
        for (const auto &byte : msg->data) {
            byte_cnt++;
            hex_stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        //RCLCPP_INFO(this->get_logger(), "%s", hex_stream.str().c_str());

                // Define the indices for the byte sections you want to swap
        size_t start_index = 11;
        size_t num_bytes_to_process = byte_cnt - start_index ;
        size_t num_blocks = num_bytes_to_process / 4;
        //std::cout << "num_blocks: " << num_blocks << std::endl;

        std::vector<uint32_t> values;

        for (size_t i = 0; i < num_blocks; ++i) {
            size_t block_start = start_index + i * 4;
            if (block_start + 4 <= msg->data.size()) {
                // Swap the 4-byte block
                custom_swap(&msg->data[block_start], 4);

                // Convert to 32-bit unsigned integer
                uint32_t value = convert_to_uint32(&msg->data[block_start]);
                values.push_back(value);
            }
        }

        // Print the 32-bit unsigned integer values in decimal
        RCLCPP_INFO(this->get_logger(), "32-bit Unsigned Integers (Decimal):");
        for (const auto& value : values) {
            RCLCPP_INFO(this->get_logger(), "%u", value);
        }

        //RCLCPP_INFO(this->get_logger(), "3Swapped Raw Hex Data:");
        // Print the swapped data in hexadecimal format
        hex_stream.str("");  // Clear the previous stream content
        hex_stream << "Swapped Raw Hex Data: ";
        for (const auto &byte : msg->data) {
            hex_stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        //RCLCPP_INFO(this->get_logger(), "%s", hex_stream.str().c_str());


    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPHexSubscriber>());
    rclcpp::shutdown();
    return 0;
}

//Write
//[INFO] [1728600193.357431551] [udp_hex_subscriber]: Raw Hex Data: d0 00 00 ff ff 03 00 02 00 00 00
//Read
//[INFO] [1728599462.257028200] [udp_hex_subscriber]: Raw Hex Data: d0 00 00 ff ff 03 00 0c 00 00 00 4e 61 bc 00 12 34 4e 61 bc 00 
