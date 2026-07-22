#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "stm32_serial_comm/serial.hpp"
#include <chrono>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
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

std::vector<uint8_t> buildFrameBody(uint8_t cmd, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> body;
    body.reserve(payload.size() + 3);
    body.push_back(cmd);
    body.push_back(static_cast<uint8_t>(payload.size()));
    body.insert(body.end(), payload.begin(), payload.end());

    uint8_t check = 0;
    for (uint8_t byte : body)
    {
        check = static_cast<uint8_t>(check + byte);
    }
    body.push_back(check);
    return body;
}

void appendInt16Le(std::vector<uint8_t>& data, int16_t value)
{
    data.push_back(static_cast<uint8_t>(value & 0xFF));
    data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
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
            test_tx_timer_ = this->create_wall_timer(1s, std::bind(&SerialCommNode::sendRandomPathPlan, this));
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
    int random_path_tx_count_{0};
    std::mt19937 rng_{std::random_device{}()};

    void sendRandomPathPlan()
    {
        if (random_path_tx_count_ >= 5)
        {
            test_tx_timer_->cancel();
            return;
        }

        constexpr uint8_t gs_cmd_path_plan = 0x20;
        std::uniform_int_distribution<int16_t> x_dist(0, 450);
        std::uniform_int_distribution<int16_t> y_dist(0, 550);
        std::pair<int16_t, int16_t> point0{x_dist(rng_), y_dist(rng_)};
        std::pair<int16_t, int16_t> point1{x_dist(rng_), y_dist(rng_)};

        std::vector<uint8_t> payload = {
            0x00,  // packet_index
            0x01,  // packet_count
            0x02,  // total_points
            0x02   // points_in_this_packet
        };
        appendInt16Le(payload, point0.first);
        appendInt16Le(payload, point0.second);
        appendInt16Le(payload, point1.first);
        appendInt16Le(payload, point1.second);

        std::vector<uint8_t> body = buildFrameBody(gs_cmd_path_plan, payload);
        std::vector<uint8_t> full_frame = {0xAA};
        full_frame.insert(full_frame.end(), body.begin(), body.end());
        full_frame.push_back(0xFF);

        int bytes_sent = serial_port_->write(body);
        random_path_tx_count_++;
        RCLCPP_INFO(
            this->get_logger(),
            "Sent random path plan %d/5, bytes: %d, P0=(%d,%d)cm, P1=(%d,%d)cm, FRAME=[%s]",
            random_path_tx_count_,
            bytes_sent,
            point0.first,
            point0.second,
            point1.first,
            point1.second,
            bytesToHex(full_frame, 0, full_frame.size()).c_str());
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
