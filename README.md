# Ros2_MC_protocol
ROS2 Humble, Ubuntu22.04

Read D18~D33 from FX5 PLC then put on topic, 
/udp_received_data.<br>Display above topic as 8x of 32bit uint values.

ROS2 三菱FX5 PLCからMCプロトコルを使ってDデバイスをD18から１６点読込、トピックに投げる。UDP通信。<br>Subscriberは8点の32bit uint valueとして表示。

## Install
Set PLC IP address and port address in src/send_dup.cpp
```
#define F7_ADDR "192.168.3.60"
#define F7_PORT 45242
```

```
cd ros2_ws/src
git clone
cd ../..
colcon build
```

## Run
On first terminal, run a publisher.
```
cd ros2_ws/
source install/setup.sh
ros2 run cpp_udp udp_send
```
```
[INFO] [1729481370.962604066] [send_udp]: Received 43 bytes from UDP
[INFO] [1729481370.983437179] [send_udp]: Received 43 bytes from UDP
[INFO] [1729481371.003299014] [send_udp]: Received 43 bytes from UDP
[INFO] [1729481371.025419827] [send_udp]: Received 43 bytes from UDP
[INFO] [1729481371.043504530] [send_udp]: Received 43 bytes from UDP
[INFO] [1729481371.063219643] [send_udp]: Received 43 bytes from UDP
```



On second terminal run a subscriber.
```
cd ros2_ws/
source install/setup.sh
ros2 run cpp_udp udp_hex_subscriber
```
```
[INFO] [1729481370.403764101] [udp_hex_subscriber]: 32-bit Unsigned Integers (Decimal):
[INFO] [1729481370.403803750] [udp_hex_subscriber]: 728331980
[INFO] [1729481370.403815558] [udp_hex_subscriber]: 0
[INFO] [1729481370.403825481] [udp_hex_subscriber]: 0
[INFO] [1729481370.415820702] [udp_hex_subscriber]: 0
[INFO] [1729481370.416005163] [udp_hex_subscriber]: 0
[INFO] [1729481370.416017761] [udp_hex_subscriber]: 0
[INFO] [1729481370.416026962] [udp_hex_subscriber]: 0
[INFO] [1729481370.416035767] [udp_hex_subscriber]: 0
```
*An inclimental rotary encoder was connected to my FX5 PLC and 728331980 was count from it. 
