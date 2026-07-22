#ifndef STM32_SERIAL_COMM_SERIAL_HPP
#define STM32_SERIAL_COMM_SERIAL_HPP

#include <string>
#include <memory>
#include <vector>
#include <deque>

namespace stm32_serial_comm
{

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    // 打开串口
    bool open(const std::string& port, int baudrate);

    // 关闭串口
    void close();

    // 发送数据
    int write(const std::vector<uint8_t>& data);

    // 接收数据
    int read(std::vector<uint8_t>& data, size_t size);

    // 检查串口是否打开
    bool isOpen() const;

private:
    enum class RxState
    {
        WaitHead,
        ReadCmd,
        ReadLen,
        ReadData,
        ReadCheck,
        ReadTail
    };

    int fd_;  // 文件描述符
    std::string port_name_;
    int baudrate_;
    bool is_open_;
    RxState rx_state_;
    uint8_t rx_cmd_;
    uint8_t rx_len_;
    uint8_t rx_check_;
    std::vector<uint8_t> rx_data_;
    std::deque<std::vector<uint8_t>> rx_frames_;

    // 配置串口参数
    bool configurePort();
};

}  // namespace stm32_serial_comm

#endif  // STM32_SERIAL_COMM_SERIAL_HPP
