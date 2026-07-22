# STM32 Serial Communication Package

ROS2功能包，用于实现与STM32的串口通信。

## 包结构

```
stm32_serial_comm/
├── CMakeLists.txt                    # CMake构建配置
├── package.xml                        # ROS2包描述文件
├── include/
│   └── stm32_serial_comm/
│       └── serial.hpp                 # 串口通信类头文件
└── src/
    ├── serial.cpp                     # 串口通信类实现
    └── main.cpp                       # ROS2节点主程序
```

## 主要文件说明

### 1. serial.hpp
串口通信类的头文件，定义了SerialPort类的接口：
- `open()`: 打开串口
- `close()`: 关闭串口
- `write()`: 发送数据
- `read()`: 接收数据
- `isOpen()`: 检查串口状态

### 2. serial.cpp
串口通信类的实现文件，包含串口操作的具体实现（待实现）。

### 3. main.cpp
ROS2节点主程序，创建SerialCommNode节点：
- 支持通过参数配置串口名称和波特率
- 默认串口: `/dev/ttyUSB0`
- 默认波特率: `115200`

## 依赖项

- `rclcpp`: ROS2 C++客户端库
- `std_msgs`: ROS2标准消息类型

## 编译

```bash
cd ~/stm32_serial_ws
colcon build --packages-select stm32_serial_comm
```

## 运行

```bash
source ~/stm32_serial_ws/install/setup.bash
ros2 run stm32_serial_comm serial_node
```

### 自定义参数运行

```bash
ros2 run stm32_serial_comm serial_node --ros-args -p port:=/dev/ttyUSB1 -p baudrate:=9600
```

## 待实现功能

- [ ] 串口打开和配置
- [ ] 数据发送功能
- [ ] 数据接收功能
- [ ] ROS2话题发布/订阅
- [ ] 数据解析和封装
- [ ] 错误处理机制
