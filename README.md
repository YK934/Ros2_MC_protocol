# Ros2_MC_protocol

Read D18~D33 from FX5 PLC then put on topic, 
/udp_received_data.

Display above topic as 8x of 32bit uint values.

ROS2 三菱FX5 PLCからMCプロトコルを使ってDデバイスをD18から１６点読込。UDP通信。

8点の32bit uint valueとして表示。

## Install
```
cd ros2_ws/src
git clone
cd ../..
colcon build
```

## Run
On first terminal, run publisher.
```
cd ros2_ws/
source install/setup.sh
ros2 run cpp_udp udp_send
```

On second terminal run subscriber.
```
cd ros2_ws/
source install/setup.sh
ros2 run cpp_udp udp_hex_subscriber
```
