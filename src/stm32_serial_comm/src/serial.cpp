#include "stm32_serial_comm/serial.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>

namespace stm32_serial_comm
{

namespace
{
constexpr uint8_t FRAME_HEAD = 0xAA;
constexpr uint8_t FRAME_TAIL = 0xFF;
constexpr uint8_t MAX_PAYLOAD_LEN = 128;

speed_t getBaudrate(int baudrate)
{
    switch (baudrate)
    {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}
}  // namespace

SerialPort::SerialPort()
    : fd_(-1),
      baudrate_(115200),
      is_open_(false),
      rx_state_(RxState::WaitHead),
      rx_cmd_(0),
      rx_len_(0),
      rx_check_(0)
{
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& port, int baudrate)
{
    if (is_open_)
    {
        close();
    }

    port_name_ = port;
    baudrate_ = baudrate;
    fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
    {
        std::cerr << "Failed to open serial port: " << port_name_
                  << ", reason: " << std::strerror(errno) << std::endl;
        return false;
    }

    if (!configurePort())
    {
        close();
        return false;
    }

    is_open_ = true;
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
    is_open_ = false;
    rx_state_ = RxState::WaitHead;
    rx_data_.clear();
    rx_frames_.clear();
}

int SerialPort::write(const std::vector<uint8_t>& data)
{
    if (!is_open_ || fd_ < 0)
    {
        return -1;
    }

    enum class TxState
    {
        SendHead,
        SendData,
        SendTail,
        Done
    };

    TxState state = TxState::SendHead;
    size_t data_index = 0;
    int sent_count = 0;
    const uint8_t frame_head = FRAME_HEAD;
    const uint8_t frame_tail = FRAME_TAIL;

    while (state != TxState::Done)
    {
        const uint8_t* buffer = nullptr;
        size_t size = 0;

        switch (state)
        {
            case TxState::SendHead:
                buffer = &frame_head;
                size = 1;
                break;
            case TxState::SendData:
                buffer = data.data() + data_index;
                size = data.size() - data_index;
                break;
            case TxState::SendTail:
                buffer = &frame_tail;
                size = 1;
                break;
            case TxState::Done:
                break;
        }

        ssize_t written = ::write(fd_, buffer, size);
        if (written < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return -1;
        }
        if (written == 0)
        {
            return -1;
        }

        sent_count += written;

        if (state == TxState::SendHead)
        {
            state = data.empty() ? TxState::SendTail : TxState::SendData;
        }
        else if (state == TxState::SendData)
        {
            data_index += written;
            if (data_index >= data.size())
            {
                state = TxState::SendTail;
            }
        }
        else if (state == TxState::SendTail)
        {
            state = TxState::Done;
        }
    }

    return sent_count;
}

int SerialPort::read(std::vector<uint8_t>& data, size_t size)
{
    data.clear();

    if (!rx_frames_.empty())
    {
        data = std::move(rx_frames_.front());
        rx_frames_.pop_front();
        return static_cast<int>(data.size());
    }

    if (!is_open_ || fd_ < 0)
    {
        return -1;
    }

    uint8_t buffer[256];
    size_t read_size = size == 0 || size > sizeof(buffer) ? sizeof(buffer) : size;
    ssize_t bytes_read = ::read(fd_, buffer, read_size);
    if (bytes_read < 0)
    {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;
        }
        return -1;
    }
    if (bytes_read == 0)
    {
        return 0;
    }

    auto reset_rx = [this]() {
        rx_state_ = RxState::WaitHead;
        rx_cmd_ = 0;
        rx_len_ = 0;
        rx_check_ = 0;
        rx_data_.clear();
    };

    for (ssize_t i = 0; i < bytes_read; ++i)
    {
        uint8_t byte = buffer[i];

        switch (rx_state_)
        {
            case RxState::WaitHead:
                if (byte == FRAME_HEAD)
                {
                    rx_state_ = RxState::ReadCmd;
                    rx_data_.clear();
                }
                break;
            case RxState::ReadCmd:
                rx_cmd_ = byte;
                rx_check_ = byte;
                rx_state_ = RxState::ReadLen;
                break;
            case RxState::ReadLen:
                rx_len_ = byte;
                rx_check_ = static_cast<uint8_t>(rx_check_ + byte);
                rx_data_.clear();
                if (rx_len_ > MAX_PAYLOAD_LEN)
                {
                    reset_rx();
                }
                else
                {
                    rx_data_.reserve(rx_len_);
                    rx_state_ = rx_len_ == 0 ? RxState::ReadCheck : RxState::ReadData;
                }
                break;
            case RxState::ReadData:
                rx_data_.push_back(byte);
                rx_check_ = static_cast<uint8_t>(rx_check_ + byte);
                if (rx_data_.size() >= rx_len_)
                {
                    rx_state_ = RxState::ReadCheck;
                }
                break;
            case RxState::ReadCheck:
                if (byte == rx_check_)
                {
                    rx_state_ = RxState::ReadTail;
                }
                else
                {
                    reset_rx();
                    if (byte == FRAME_HEAD)
                    {
                        rx_state_ = RxState::ReadCmd;
                    }
                }
                break;
            case RxState::ReadTail:
                if (byte == FRAME_TAIL)
                {
                    std::vector<uint8_t> frame;
                    frame.reserve(static_cast<size_t>(rx_len_) + 5);
                    frame.push_back(FRAME_HEAD);
                    frame.push_back(rx_cmd_);
                    frame.push_back(rx_len_);
                    frame.insert(frame.end(), rx_data_.begin(), rx_data_.end());
                    frame.push_back(rx_check_);
                    frame.push_back(FRAME_TAIL);
                    rx_frames_.push_back(std::move(frame));
                }
                reset_rx();
                if (byte == FRAME_HEAD)
                {
                    rx_state_ = RxState::ReadCmd;
                }
                break;
        }
    }

    if (rx_frames_.empty())
    {
        return 0;
    }

    data = std::move(rx_frames_.front());
    rx_frames_.pop_front();
    return static_cast<int>(data.size());
}

bool SerialPort::isOpen() const
{
    return is_open_;
}

bool SerialPort::configurePort()
{
    termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0)
    {
        std::cerr << "Failed to get serial port attributes" << std::endl;
        return false;
    }

    cfmakeraw(&tty);
    speed_t speed = getBaudrate(baudrate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Failed to set serial port attributes" << std::endl;
        return false;
    }

    return true;
}

}  // namespace stm32_serial_comm
