#include <cstdint>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <vector>
#include <stdlib.h>     /* strtoul */
#include <iostream>
#include "ros2_udp.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>


Ros2UDP::Ros2UDP(const std::string& f7_address, int f7_port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&f7_addr, 0, sizeof(f7_addr));
    f7_addr.sin_family = AF_INET;
    f7_addr.sin_port = htons(f7_port);
    inet_pton(AF_INET, f7_address.c_str(), &f7_addr.sin_addr);
}


void Ros2UDP::send_packet(uint8_t *packet, uint8_t size) {
    int sent_bytes = sendto(sock, packet, size, 0, (struct sockaddr*)&f7_addr, sizeof(f7_addr));
    if (sent_bytes < 0) {
        std::cerr << "Packet send error: " << strerror(errno) << std::endl;
    } else {
        //std::cout << "Packet sent: " << sent_bytes << " bytes" << std::endl;
    }
}

int Ros2UDP::udp_recv(uint8_t *buf, uint8_t size, int timeout_sec) {
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = 0;

    int select_result = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (select_result < 0) {
        std::cerr << "Select error: " << strerror(errno) << std::endl;
        return -1;
    } else if (select_result == 0) {
        std::cerr << "Receive timeout after " << timeout_sec << " seconds" << std::endl;
        return 0;
    } else {
        int recv_len = recvfrom(sock, buf, size, 0, (struct sockaddr*)&sender_addr, &sender_len);
        if (recv_len < 0) {
            std::cerr << "Receive error: " << strerror(errno) << std::endl;
        } else {
            //std::cout << "Received " << recv_len << " bytes from " << inet_ntoa(sender_addr.sin_addr) << std::endl;
        }

        //std::cout << "Sender address: " << inet_ntoa(sender_addr.sin_addr) << std::endl;
        //std::cout << "Sender port: " << ntohs(sender_addr.sin_port) << std::endl;

        // std::cout << "Raw data received: ";
        // for (int i = 0; i < recv_len; i++) {
        //     std::cout << std::hex << static_cast<int>(buf[i]) << " ";
        // }
        //std::cout << std::dec << std::endl; // Switch back to decimal for future logs

        return recv_len;
    }
}



//Create packet
//size2: number of main arguments
//int size = request(send_buffer, BUFSIZE, argv, argc); in main.cpp 
int request(unsigned char *buffer,  const std::vector<uint8_t>& argv ,int size2) {
    
    std::vector<unsigned char> sub_h = {
	0x50, 0x00, 0x00, 0xff, 0xff, 0x03, 0x00, 
	0x0c, 0x00,
	0x10, 0x00, 0x01, 0x04, 0x00, 0x00
    };
    const std::vector<unsigned char> start_addr = {
	argv[0],argv[1],argv[2]
    }; // LH(LH)
    const std::vector<unsigned char> device_code = {
	argv[3]
    };
    const std::vector<unsigned char> device_cnt = {
	argv[4], argv[5]
    }; // LH

    std::vector<unsigned char> w_value;
    
    //If argc is beigger than 7, means write values.
    if(size2 > 7){
    std::cout << "Write data!" << std::endl;
	sub_h[12] = {0x14};//Change to D write mode. default is read mode 0x04.
	int w_dev_cnt = (size2 - 6)/2;
    //printf("%02x,\n", w_dev_cnt);
	if (w_dev_cnt == 0){
    		std::cout << "No write data!" << std::endl;
        	exit(EXIT_FAILURE);
	} else {
		for (int i = 6; i < size2 ; i++){
			w_value.push_back(argv[i]);
			sub_h[7]++;// sub[7],[8] data length
			//printf("%02x,\n", w_value[ i - 6 ]);
		}
		if (w_dev_cnt != argv[4]){
			printf("Write count not matched!\n");
        		exit(EXIT_FAILURE);
		}
	}
    }

    std::vector<unsigned char> cmd;
    cmd.reserve(sub_h.size() + start_addr.size()
	 + device_code.size() + device_cnt.size() + w_value.size());

    cmd.insert(cmd.end(), sub_h.begin(), sub_h.end());
    cmd.insert(cmd.end(), start_addr.begin(), start_addr.end());
    cmd.insert(cmd.end(), device_code.begin(), device_code.end());
    cmd.insert(cmd.end(), device_cnt.begin(), device_cnt.end());
    cmd.insert(cmd.end(), w_value.begin(), w_value.end());

    std::copy(cmd.begin(), cmd.end(), buffer);

    return cmd.size();
}


