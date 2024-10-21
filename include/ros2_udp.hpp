#ifndef _ROS2_UDP_HPP_
#define _ROS2_UDP_HPP_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <vector>

class Ros2UDP{
    public:
        //コンストラクタ
        Ros2UDP(const std::string& f7_address, int f7_port);
        //~Ros2UDP();

        //ソケットにアドレスをバインド(関連付け)
        void udp_bind();

        //パケットを送信
        void send_packet(uint8_t *packet, uint8_t size);

        void set_port(int new_port);

        //受信する
        int udp_recv(uint8_t *buf, uint8_t size, int timeout_sec);



    private:
        int sock;
        struct sockaddr_in f7_addr;
};

#endif

int request(unsigned char *buffer,  const std::vector<uint8_t>& argv ,int size2);

