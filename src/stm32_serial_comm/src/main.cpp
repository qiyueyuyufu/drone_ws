#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "stm32_serial_comm/serial.hpp"
#include <chrono>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{
const char* commandName(uint8_t cmd)
{
    switch (cmd)
    {
        case 0x01: return "GS_CMD_TAKEOFF";
        case 0x02: return "GS_CMD_LAND";
        case 0x03: return "GS_CMD_RETURN_HOME";
        case 0x10: return "GS_CMD_FORBIDDEN_MAP";
        case 0x20: return "GS_CMD_PATH_PLAN";
        case 0x7F: return "GS_CMD_ACK";
        case 0x7E: return "GS_CMD_NACK";
        default: return "UNKNOWN";
    }
}

std::string bytesToHex(const std::vector<uint8_t>& data, size_t begin, size_t end)
{
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');

    for (size_t i = begin; i < end; ++i)
    {
        if (i > begin)
        {
            oss << ' ';
        }
        oss << std::setw(2) << static_cast<int>(data[i]);
    }

    return oss.str();
}

std::string payloadToText(const std::vector<uint8_t>& data, size_t begin, size_t end)
{
    std::string text;
    text.reserve(end - begin);
    for (size_t i = begin; i < end; ++i)
    {
        if (data[i] == '\0')
        {
            break;
        }
        text.push_back(static_cast<char>(data[i]));
    }
    return text;
}

std::string toLowerAscii(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

std::vector<uint8_t> buildFixedPathFrameBody(uint8_t type, uint8_t a, uint8_t b)
{
    uint8_t check = static_cast<uint8_t>(type + a + b);
    return {type, a, b, check};
}
}  // namespace

class SerialCommNode : public rclcpp::Node
{
public:
    SerialCommNode() : Node("stm32_serial_comm_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);

        // 获取参数
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // 初始化串口
        serial_port_ = std::make_shared<stm32_serial_comm::SerialPort>();
        command_pub_ = this->create_publisher<std_msgs::msg::String>("/stm32_serial/command", 10);

        RCLCPP_INFO(this->get_logger(), "STM32 Serial Communication Node initialized");
        RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d", port.c_str(), baudrate);

        if (serial_port_->open(port, baudrate))
        {
            timer_ = this->create_wall_timer(20ms, std::bind(&SerialCommNode::pollSerial, this));
            path_test_frames_ = {
                buildFixedPathFrameBody(0x30, 0x00, 0x00),
                buildFixedPathFrameBody(0x31, 0x01, 0x01),
                buildFixedPathFrameBody(0x31, 0x03, 0x05),
                buildFixedPathFrameBody(0x32, 0x00, 0x00)
            };
            test_tx_timer_ = this->create_wall_timer(200ms, std::bind(&SerialCommNode::sendNextPathTestFrame, this));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }
        // TODO: 创建发布者和订阅者
        // TODO: 创建定时器进行串口数据读取
    }

    ~SerialCommNode()
    {
        if (serial_port_)
        {
            serial_port_->close();
        }
    }

private:
    std::shared_ptr<stm32_serial_comm::SerialPort> serial_port_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr test_tx_timer_;
    std::vector<std::vector<uint8_t>> path_test_frames_;
    size_t path_test_frame_index_{0};

    void sendNextPathTestFrame()
    {
        if (path_test_frame_index_ >= path_test_frames_.size())
        {
            test_tx_timer_->cancel();
            return;
        }

        const std::vector<uint8_t>& body = path_test_frames_[path_test_frame_index_];
        std::vector<uint8_t> full_frame = {0xAA};
        full_frame.insert(full_frame.end(), body.begin(), body.end());
        full_frame.push_back(0xFF);

        int bytes_sent = serial_port_->write(body);
        RCLCPP_INFO(
            this->get_logger(),
            "Sent path test frame %zu/%zu, bytes: %d, FRAME=[%s]",
            path_test_frame_index_ + 1,
            path_test_frames_.size(),
            bytes_sent,
            bytesToHex(full_frame, 0, full_frame.size()).c_str());
        path_test_frame_index_++;
    }

    void pollSerial()
    {
        std::vector<uint8_t> frame;

        while (true)
        {
            int bytes_read = serial_port_->read(frame, 256);
            if (bytes_read > 0)
            {
                logFrame(frame);
                continue;
            }

            if (bytes_read < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Serial read failed");
            }
            break;
        }
    }

    void logFrame(const std::vector<uint8_t>& frame)
    {
        if (frame.size() < 5)
        {
            return;
        }

        uint8_t cmd = frame[1];
        uint8_t len = frame[2];
        size_t data_begin = 3;
        size_t data_end = data_begin + len;
        if (data_end > frame.size() - 2)
        {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "RX %s CMD=0x%02X LEN=%u DATA=[%s] FRAME=[%s]",
            commandName(cmd),
            cmd,
            len,
            bytesToHex(frame, data_begin, data_end).c_str(),
            bytesToHex(frame, 0, frame.size()).c_str());

        publishCommand(cmd, frame, data_begin, data_end);
    }

    void publishCommand(uint8_t cmd, const std::vector<uint8_t>& frame, size_t data_begin, size_t data_end)
    {
        std::string command;
        if (cmd == 0x01)
        {
            command = "takeoff";
        }
        else if (cmd == 0x02)
        {
            command = "land";
        }
        else
        {
            command = toLowerAscii(payloadToText(frame, data_begin, data_end));
        }

        if (command == "takeoff" || command == "land")
        {
            std_msgs::msg::String msg;
            msg.data = command;
            command_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published serial command: %s", msg.data.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
