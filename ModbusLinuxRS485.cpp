
// ################################################################################
// Include libraries:

#include "ModbusLinuxRS485.h"
#include <fcntl.h>              // For file control (open, close)
#include <unistd.h>             // For read, write, sleep, usleep
#include <errno.h>              // For error numbers
#include <sstream>
#include <stdexcept>            // For runtime_error
#include <iomanip>
#include <termios.h>            // For terminal control (termios struct, baud rates)
#include <chrono>
#include <poll.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <utility>     // for std::move

// ################################################################################

static inline uint64_t now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

// ################################################################################
// Modbus class:

// ------------------------
// Lifecycle
// ------------------------

Modbus::Modbus()
{   
    errorMessage.clear();

    parameters.PORT = "";
    parameters.BAUDRATE = 9600;
    parameters.TRANSMIT_DELAY_US = 0;
    parameters.RESPONSE_TIMEOUT_MS = 1000;
    parameters.PARITY = Parity::None;
    parameters.STOP_BITS = 1;
    parameters.DATA_BITS = 8;

    _serialPort = -1;
}

Modbus::Modbus(Modbus&& other) noexcept
{
    errorMessage = std::move(other.errorMessage);
    parameters   = other.parameters;
    _serialPort  = other._serialPort;
    other._serialPort = -1;
}

Modbus& Modbus::operator=(Modbus&& other) noexcept
{
    if (this == &other) return *this;

    closePort(); // safe cleanup

    errorMessage = std::move(other.errorMessage);
    parameters   = other.parameters;
    _serialPort  = other._serialPort;
    other._serialPort = -1;
    return *this;
}

Modbus::~Modbus()
{
    if (_serialPort >= 0) 
    {
        ::close(_serialPort);
        _serialPort = -1;
    }
}

void Modbus::closePort() noexcept
{
    if (_serialPort >= 0)
    {
        ::close(_serialPort);
        _serialPort = -1;
    }
}

bool Modbus::isPortOpen() const noexcept
{ 
    return (_serialPort >= 0); 
}

// ------------------------
// Port open/config
// ------------------------

bool Modbus::openPort(void)
{
    if (parameters.PORT.empty())
    {
        errorMessage = "Serial PORT is empty.";
        return false;
    }

    speed_t baud_rate;
    switch (parameters.BAUDRATE) 
    {
        case 9600: baud_rate = B9600; break;
        case 19200: baud_rate = B19200; break;
        case 38400: baud_rate = B38400; break;
        case 57600: baud_rate = B57600; break;
        case 115200: baud_rate = B115200; break;
        default: 
            errorMessage = "Baud rate is not valid.";
            return false;
    }

    // Initialization code remains the same as the working version
    _serialPort = open(parameters.PORT.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_serialPort < 0) 
    {
        std::ostringstream oss;
        oss << "Error " << errno << " opening port: " << strerror(errno);
        errorMessage = oss.str();

        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(_serialPort, &tty); 

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;

    // no hw flow control
    tty.c_cflag &= ~CRTSCTS;

    // Data bits
    switch (parameters.DATA_BITS)
    {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default:
            errorMessage = "Invalid DATA_BITS (must be 5..8).";
            ::close(_serialPort); _serialPort = -1;
            return false;
    }

    // Stop bits
    if (parameters.STOP_BITS == 2) tty.c_cflag |= CSTOPB;
    else if (parameters.STOP_BITS == 1) tty.c_cflag &= ~CSTOPB;
    else
    {
        errorMessage = "Invalid STOP_BITS (must be 1 or 2).";
        ::close(_serialPort); _serialPort = -1;
        return false;
    }

    // Parity
    tty.c_cflag &= ~(PARENB | PARODD);
    switch (parameters.PARITY)
    {
        case Parity::None:
            // nothing
            break;
        case Parity::Even:
            tty.c_cflag |= PARENB;
            break;
        case Parity::Odd:
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            break;
    }

    tty.c_lflag = 0; tty.c_oflag = 0; 

    // raw-ish input
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                    INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    // raw-ish output
    tty.c_oflag &= ~OPOST;

    // raw-ish local
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // // VMIN=1 (wait for first byte), VTIME=10 (1 second timeout for subsequent bytes)
    // tty.c_cc[VMIN]  = 1; 
    // tty.c_cc[VTIME] = (cc_t)(parameters.RESPONSE_TIMEOUT_MS / 100.0); 

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(_serialPort, TCSANOW, &tty) != 0) 
    {
        std::ostringstream oss;
        oss << "Error setting port attributes: " << strerror(errno);
        errorMessage = oss.str();

        ::close(_serialPort); 
        _serialPort = -1; 
        return false;
    }

    if (tcgetattr(_serialPort, &tty) != 0)
    {
        std::ostringstream oss;
        oss << "Error getting port attributes: " << std::strerror(errno);
        errorMessage = oss.str();
        ::close(_serialPort);
        _serialPort = -1;
        return false;
    }

    return true;
}

// ------------------------
// IO helpers
// ------------------------

bool Modbus::_serialSend(const std::vector<uint8_t>& data) 
{ 
    return _serialSend(data.data(), data.size()); 
}

bool Modbus::_serialSend(const uint8_t* data, size_t len)
{
    if (!isPortOpen()) 
    {
        errorMessage = "Send failed. Port is closed.";
        return false;
    }

    // Flush stale input before request (OK for Modbus RTU).
    tcflush(_serialPort, TCIFLUSH); 

    size_t sent = 0;
    while (sent < len)
    {
        ssize_t n = ::write(_serialPort, data + sent, len - sent);
        if (n < 0)
        {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) { ::usleep(1000); continue; } // tiny backoff
            std::ostringstream oss;
            oss << "write failed: " << std::strerror(errno);
            errorMessage = oss.str();
            return false;
        }
        sent += static_cast<size_t>(n);
    }

    // Ensure request is physically transmitted before turnaround delay.
    if (tcdrain(_serialPort) != 0)
    {
        std::ostringstream oss;
        oss << "tcdrain failed: " << std::strerror(errno);
        errorMessage = oss.str();
        return false;
    }

    if (parameters.TRANSMIT_DELAY_US > 0)
    {
        usleep(parameters.TRANSMIT_DELAY_US); // Bus Turnaround Delay
    }
    
    return true;
}

size_t Modbus::_serialReceive(size_t expected_length, uint8_t* buffer)
{
    if (!isPortOpen())
    {
        errorMessage = "Receive failed, port is closed.";
        return 0;
    }

    ssize_t n = ::read(_serialPort, buffer, expected_length);

    if (n > 0) return static_cast<size_t>(n);

    if (n == 0)
    {
        errorMessage = "Serial read returned 0 bytes.";
        return 0;
    }

    // n < 0
    if (errno == EINTR) return 0; // caller may retry
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // no data yet (non-blocking)

    std::ostringstream oss;
    oss << "Error reading serial data: " << std::strerror(errno);
    errorMessage = oss.str();
    return 0;
}

std::vector<uint8_t> Modbus::_serialReceive(size_t n, uint32_t timeout_ms)
{
    if (!isPortOpen())
    {
        errorMessage = "Receive failed: port is closed.";
        return {};
    }

    std::vector<uint8_t> out;
    out.reserve(n);

    const int fd = _serialPort;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (out.size() < n)
    {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            std::ostringstream oss;
            oss << "Timeout while reading (" << timeout_ms << " ms total).";
            errorMessage = oss.str();
            return {};
        }

        int remaining = (int)std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
        if (remaining <= 0) remaining = 1;

        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int rc = ::poll(&pfd, 1, remaining);
        if (rc == 0)
        {
            std::ostringstream oss;
            oss << "Timeout while reading (" << timeout_ms << " ms total).";
            errorMessage = oss.str();
            return {};
        }
        if (rc < 0)
        {
            if (errno == EINTR) continue;
            std::ostringstream oss;
            oss << "poll failed: " << std::strerror(errno);
            errorMessage = oss.str();
            return {};
        }

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            errorMessage = "Serial port error/hangup while reading.";
            return {};
        }

        uint8_t buf[256];
        const size_t want = std::min(sizeof(buf), n - out.size());

        ssize_t got = ::read(fd, buf, want);
        if (got < 0)
        {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) continue;
            std::ostringstream oss;
            oss << "read failed: " << std::strerror(errno);
            errorMessage = oss.str();
            return {};
        }
        if (got == 0) continue;

        out.insert(out.end(), buf, buf + got);
    }

    return out;
}

bool Modbus::_validateBasicResponse(const std::vector<uint8_t>& resp,
                                  uint8_t expected_slave,
                                  uint8_t expected_function)
{
    if (resp.size() < 5)
    {
        errorMessage = "Response too short.";
        return false;
    }
    if (resp[0] != expected_slave)
    {
        errorMessage = "Slave ID mismatch.";
        return false;
    }
    // Exception response?
    if (resp[1] == static_cast<uint8_t>(expected_function | 0x80))
    {
        std::ostringstream oss;
        oss << "Modbus exception, code=0x" << std::hex << std::uppercase
            << static_cast<int>(resp[2]);
        errorMessage = oss.str();
        return false;
    }
    if (resp[1] != expected_function)
    {
        errorMessage = "Function code mismatch.";
        return false;
    }
    if (!_verifyCrc(resp))
    {
        errorMessage = "CRC mismatch in response.";
        return false;
    }
    return true;
}

std::vector<uint8_t> Modbus::_receiveRtuResponse(uint8_t expected_slave,
                                                uint8_t expected_function,
                                                size_t expected_fixed_len,
                                                uint32_t timeout_ms)
{
    const uint64_t deadline_ms = now_ms() + (uint64_t)timeout_ms;

    auto remaining_ms = [&]() -> uint32_t
    {
        uint64_t now = now_ms();
        if (now >= deadline_ms) return 0;
        uint64_t rem = deadline_ms - now;
        if (rem > 0xFFFFFFFFULL) rem = 0xFFFFFFFFULL;
        return (uint32_t)rem;
    };

    // For read functions, expected_fixed_len = 0; we will read header to compute full len.
    // For write echoes/acks, expected_fixed_len is known (usually 8).
    if (expected_fixed_len != 0)
    {
        uint32_t rem = remaining_ms();
        if (rem == 0) { errorMessage = "Timeout before reading fixed response."; return {}; }

        auto resp = _serialReceive(expected_fixed_len, rem);
        if (resp.empty()) return {};
        if (!_validateBasicResponse(resp, expected_slave, expected_function)) return {};
        return resp;
    }

    uint32_t rem = remaining_ms();
    if (rem == 0) { errorMessage = "Timeout waiting for response header."; return {}; }

    // Variable-length: read first 2 bytes to detect exception, then 3rd byte (byte-count) for normal response.
    auto first2 = _serialReceive(2, rem);
    if (first2.size() != 2) return {};

    // If function indicates exception, read remaining 3 bytes (exc code + CRC2) => total 5 bytes
    if (first2.size() == 2 && first2[1] == static_cast<uint8_t>(expected_function | 0x80))
    {
        rem = remaining_ms();
        if (rem == 0) { errorMessage = "Timeout waiting for exception tail."; return {}; }

        auto rest3 = _serialReceive(3, rem);
        if (rest3.empty()) return {};
        std::vector<uint8_t> resp;
        resp.reserve(5);
        resp.insert(resp.end(), first2.begin(), first2.end());
        resp.insert(resp.end(), rest3.begin(), rest3.end());
        // validateBasicResponse will format the exception nicely (and CRC)
        (void)_validateBasicResponse(resp, expected_slave, expected_function);
        return {};
    }

    // Normal response: need 3rd byte = byte count

    rem = remaining_ms();
    if (rem == 0) { errorMessage = "Timeout waiting for byteCount."; return {}; }

    auto third = _serialReceive(1, rem);
    if (third.empty()) return {};
    const uint8_t byteCount = third[0];

    // Remaining = byteCount data + CRC2

    rem = remaining_ms();
    if (rem == 0) { errorMessage = "Timeout waiting for data/CRC."; return {}; }

    auto rest = _serialReceive(static_cast<size_t>(byteCount) + 2, rem);
    if (rest.empty()) return {};

    std::vector<uint8_t> resp;
    resp.reserve(3 + byteCount + 2);
    resp.insert(resp.end(), first2.begin(), first2.end());
    resp.push_back(byteCount);
    resp.insert(resp.end(), rest.begin(), rest.end());

    if (!_validateBasicResponse(resp, expected_slave, expected_function)) return {};
    return resp;
}

// ------------------------
// Modbus operations
// ------------------------

std::vector<uint8_t> Modbus::readCoils(uint8_t slaveID, uint16_t starting_address, uint16_t quantity)
{
    if (!isPortOpen()) { errorMessage = "Port is closed."; return {}; }
    if (quantity == 0 || quantity > 2000) { errorMessage = "Invalid quantity (1..2000)."; return {}; }

    const uint16_t addr = _normalizeCoilAddress(starting_address);

    std::vector<uint8_t> req{
        slaveID, 0x01,
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(quantity >> 8), static_cast<uint8_t>(quantity & 0xFF)
    };
    _appendCrc(req);

    if (!_serialSend(req)) return {};

    auto resp = _receiveRtuResponse(slaveID, 0x01, 0, parameters.RESPONSE_TIMEOUT_MS);
    if (resp.size() < 5) { errorMessage = "Response too short."; return {}; }

    const uint8_t byteCount = resp[2];
    const size_t expectedByteCount = (quantity + 7u) / 8u;
    if (byteCount != expectedByteCount)
    {
        errorMessage = "Byte-count mismatch in response.";
        return {};
    }

    // return ONLY data bytes
    return std::vector<uint8_t>(resp.begin() + 3, resp.end() - 2);
}

std::vector<uint8_t> Modbus::readDiscreteInputs(uint8_t slaveID, uint16_t starting_address, uint16_t quantity)
{
    if (!isPortOpen()) { errorMessage = "Port is closed."; return {}; }
    if (quantity == 0 || quantity > 2000) { errorMessage = "Invalid quantity (1..2000)."; return {}; }

    const uint16_t addr = _normalizeDiscreteInputAddress(starting_address);

    std::vector<uint8_t> req{
        slaveID, 0x02,
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(quantity >> 8), static_cast<uint8_t>(quantity & 0xFF)
    };
    _appendCrc(req);

    if (!_serialSend(req)) return {};

    auto resp = _receiveRtuResponse(slaveID, 0x02, 0, parameters.RESPONSE_TIMEOUT_MS);
    if (resp.size() < 5) { errorMessage = "Response too short."; return {}; }

    const uint8_t byteCount = resp[2];
    const size_t expectedByteCount = (quantity + 7u) / 8u;
    if (byteCount != expectedByteCount)
    {
        errorMessage = "Byte-count mismatch in response.";
        return {};
    }

    return std::vector<uint8_t>(resp.begin() + 3, resp.end() - 2);
}

bool Modbus::writeSingleCoil(uint8_t slaveID, uint16_t address, bool on)
{
    if (!isPortOpen()) { errorMessage = "Port is closed."; return false; }

    const uint16_t addr = _normalizeCoilAddress(address);
    const uint16_t val  = on ? 0xFF00 : 0x0000;

    std::vector<uint8_t> req{
        slaveID, 0x05,
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(val >> 8),  static_cast<uint8_t>(val & 0xFF)
    };
    _appendCrc(req);

    if (!_serialSend(req)) return false;

    // Function 05 echoes the request (8 bytes)
    auto resp = _receiveRtuResponse(slaveID, 0x05, 8, parameters.RESPONSE_TIMEOUT_MS);

    if (resp.empty()) return false;

    const uint16_t raddr = (static_cast<uint16_t>(resp[2]) << 8) | resp[3];
    const uint16_t rval  = (static_cast<uint16_t>(resp[4]) << 8) | resp[5];
    if (raddr != addr || rval != val)
    {
        errorMessage = "0x05 echo mismatch.";
        return false;
    }

    return !resp.empty();
}

std::vector<uint8_t> Modbus::readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers)
{
    if (!isPortOpen()) { errorMessage = "Read failed. Port is closed."; return {}; }
    if (num_registers == 0 || num_registers > 125) { errorMessage = "Invalid quantity (1..125)."; return {}; }

    // 1. Build the Modbus Request (6 bytes)
    // Modbus 4XXXX registers use 0-based addressing (40001 -> 0x0000)
    const uint16_t addr = _normalizeHoldingRegisterAddress(starting_address);

    std::vector<uint8_t> req
    {
        slaveID, 0x03,
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(num_registers >> 8), static_cast<uint8_t>(num_registers & 0xFF)
    };

    // 2. Calculate and Append CRC-16
    _appendCrc(req);

    if (!_serialSend(req)) return {};

    // Variable-length RTU response (handles exception frames properly)
    auto resp = _receiveRtuResponse(slaveID, 0x03, 0, parameters.RESPONSE_TIMEOUT_MS);
    if (resp.empty()) return {};

    if (resp.size() < 5) { errorMessage = "Response too short."; return {}; }

    const uint8_t byteCount = resp[2];
    if (byteCount != static_cast<uint8_t>(num_registers * 2))
    {
        errorMessage = "Byte-count mismatch in response.";
        return {};
    }

    // return ONLY data bytes
    return std::vector<uint8_t>(resp.begin() + 3, resp.end() - 2);
}

bool Modbus::readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers, uint16_t* buffer)
{
    if (!buffer) { errorMessage = "Output buffer is null."; return false; }
    if (!isPortOpen()) { errorMessage = "Read failed. Port is closed."; return false; }
    if (num_registers == 0 || num_registers > 125) { errorMessage = "Invalid quantity (1..125)."; return false; }

    auto data = readHoldingRegisters(slaveID, starting_address, num_registers);
    if (data.size() != static_cast<size_t>(num_registers) * 2)
    {
        errorMessage = "Holding register data length mismatch.";
        return false;
    }

    for (uint16_t i = 0; i < num_registers; ++i)
    {
        buffer[i] = (static_cast<uint16_t>(data[2*i]) << 8) |
                    static_cast<uint16_t>(data[2*i + 1]);
    }
    return true;
}

std::vector<uint8_t> Modbus::readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity)
{
    if (!isPortOpen()) { errorMessage = "Port is closed."; return {}; }
    if (quantity == 0 || quantity > 125) { errorMessage = "Invalid quantity (1..125)."; return {}; }

    const uint16_t addr = _normalizeInputRegisterAddress(starting_address);

    std::vector<uint8_t> req{
        slaveID, 0x04,
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(quantity >> 8), static_cast<uint8_t>(quantity & 0xFF)
    };
    _appendCrc(req);

    if (!_serialSend(req)) return {};

    auto resp = _receiveRtuResponse(slaveID, 0x04, 0, parameters.RESPONSE_TIMEOUT_MS);
    if (resp.empty()) return {};
    // resp = [id][fc][byteCount][data...][crc][crc]
    if (resp.size() < 5) { errorMessage = "Response too short."; return {}; }

    const uint8_t byteCount = resp[2];
    if (byteCount != quantity * 2)
    {
        errorMessage = "Byte-count mismatch in response.";
        return {};
    }

    return std::vector<uint8_t>(resp.begin() + 3, resp.end() - 2);
}

bool Modbus::readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, uint16_t* out)
{
    if (!out) { errorMessage = "Output buffer is null."; return false; }
    if (quantity == 0 || quantity > 125) { errorMessage = "Invalid quantity (1..125)."; return false; }

    auto data = readInputRegisters(slaveID, starting_address, quantity);
    if (data.size() != static_cast<size_t>(quantity) * 2)
    {
        errorMessage = "Input register data length mismatch.";
        return false;
    }

    for (uint16_t i = 0; i < quantity; ++i)
    {
        out[i] = (static_cast<uint16_t>(data[2*i]) << 8) |
                 static_cast<uint16_t>(data[2*i + 1]);
    }
    return true;
}

bool Modbus::writeSingleRegister(uint8_t slaveID, uint16_t starting_address, uint16_t value) 
{
    if (!isPortOpen()) 
    {
        errorMessage = "Modbus port is closed.";
        return false;
    }

    // 1. Modbus uses 0-based addressing (Manual Register 40030 -> 0x001D)
    uint16_t address_0based = _normalizeHoldingRegisterAddress(starting_address);

    // 2. Build the Modbus Request (6 bytes + 2 CRC)
    std::vector<uint8_t> request_data = 
    {
        slaveID,
        0x06,                           // Function Code: 06 (Write Single Register)
        (uint8_t)(address_0based >> 8),
        (uint8_t)(address_0based & 0xFF),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };

    // 3. Append CRC-16
    _appendCrc(request_data);

    // std::cout << "Sending Write Command: ";
    // printHex(request_data);

    if (!_serialSend(request_data)) 
    {
        return false;
    }

    // 4. Receive Response (Function 06 echoes the request back exactly)
    // ✅ Use RTU receiver so exception frames (5 bytes) don't cause a timeout hang.

    // size_t expected_length = 8; 
    // std::vector<uint8_t> response = _serialReceive(expected_length, parameters.RESPONSE_TIMEOUT_MS);

    // if (response.size() != expected_length)
    // {
    //     errorMessage = "ERROR: Write response timeout or incomplete.";
    //     return false;
    // }

    auto response = _receiveRtuResponse(slaveID, 0x06, 8, parameters.RESPONSE_TIMEOUT_MS);
    if (response.empty()) return false;   // errorMessage already set


    // if (!_verifyCrc(response))
    // {
    //     errorMessage = "CRC mismatch in response.";
    //     return false;
    // }

    // // Exception first
    // if (response[1] == (0x06 | 0x80))
    // {
    //     std::ostringstream oss;
    //     oss << "MODBUS ERROR: Exception code 0x" << std::hex << (int)response[2] << std::dec;
    //     errorMessage = oss.str();
    //     return false;
    // }

    // if (response[0] != slaveID || response[1] != 0x06)
    // {
    //     errorMessage = "Write single register response header mismatch.";
    //     return false;
    // }

    // Extra echo validation (optional but good)
    const uint16_t raddr = (static_cast<uint16_t>(response[2]) << 8) | response[3];
    const uint16_t rval  = (static_cast<uint16_t>(response[4]) << 8) | response[5];
    if (raddr != address_0based || rval != value)
    {
        errorMessage = "0x06 echo mismatch.";
        return false;
    }


    // std::cout << "Write Successful!" << std::endl;

    return true;
}

bool Modbus::writeMultipleCoils(uint8_t slaveID, uint16_t starting_address, const std::vector<bool>& values)
{
    if (!isPortOpen()) { errorMessage = "Port is closed."; return false; }
    if (values.empty() || values.size() > 1968) { errorMessage = "Invalid quantity (1..1968)."; return false; }

    const uint16_t addr = _normalizeCoilAddress(starting_address);
    const uint16_t qty  = static_cast<uint16_t>(values.size());
    const uint16_t byteCount = static_cast<uint16_t>((qty + 7) / 8);

    std::vector<uint8_t> req{
        slaveID, 0x0F,
        static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(qty >> 8),  static_cast<uint8_t>(qty & 0xFF),
        static_cast<uint8_t>(byteCount)
    };

    // Pack bits (LSB first per Modbus)
    req.resize(req.size() + byteCount, 0);
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (values[i])
            req[7 + (i / 8)] |= static_cast<uint8_t>(1u << (i % 8));
    }

    _appendCrc(req);

    if (!_serialSend(req)) return false;

    auto resp = _receiveRtuResponse(slaveID, 0x0F, 8, parameters.RESPONSE_TIMEOUT_MS);
    return !resp.empty();
}

bool Modbus::writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, const std::vector<uint16_t>& values) 
{
    if (!isPortOpen()) { errorMessage = "Port is closed."; return false; }
    if (values.empty() || values.size() > 123) { errorMessage = "Invalid quantity (1..123)."; return false; }

    uint16_t num_registers = values.size();
    uint16_t address_0based = _normalizeHoldingRegisterAddress(starting_address);

    // 1. Build the Modbus Request Header
    std::vector<uint8_t> request_data = 
    {
        slaveID,
        0x10,                           // Function Code: 0x10 (Write Multiple Registers)
        (uint8_t)(address_0based >> 8),
        (uint8_t)(address_0based & 0xFF),
        (uint8_t)(num_registers >> 8),
        (uint8_t)(num_registers & 0xFF),
        (uint8_t)(num_registers * 2)    // Byte Count (2 bytes per register)
    };

    // 2. Append the Register Data (Big Endian)
    for (uint16_t val : values) 
    {
        request_data.push_back((uint8_t)(val >> 8));
        request_data.push_back((uint8_t)(val & 0xFF));
    }

    // 3. Append CRC-16
    _appendCrc(request_data);

    // std::cout << "Sending Write Multiple: ";
    // printHex(request_data);

    if (!_serialSend(request_data)) return false;

    // 4. Receive Response (8 bytes for Function 0x10)
    // Response format: [ID][0x10][AddrH][AddrL][QtyH][QtyL][CRCL][CRCH]
    // ✅ Use RTU receiver so exception frames (5 bytes) don't cause a timeout hang.
    auto response = _receiveRtuResponse(slaveID, 0x10, 8, parameters.RESPONSE_TIMEOUT_MS);
    if (response.empty()) return false;

    // Optional: validate echo fields (addr + qty)
    const uint16_t raddr = (static_cast<uint16_t>(response[2]) << 8) | response[3];
    const uint16_t rqty  = (static_cast<uint16_t>(response[4]) << 8) | response[5];
    if (raddr != address_0based || rqty != num_registers)
    {
        errorMessage = "0x10 response echo mismatch.";
        return false;
    }

    // std::cout << "Multi-Register Write Successful!" << std::endl;
    return true;
}

bool Modbus::writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers, const uint16_t* values)
{
    if (!isPortOpen() || (values == nullptr)) return false;

    uint16_t address_0based = _normalizeHoldingRegisterAddress(starting_address);

    // 1. Build the Modbus Request Header
    std::vector<uint8_t> request_data = 
    {
        slaveID,
        0x10,                           // Function Code: 0x10 (Write Multiple Registers)
        (uint8_t)(address_0based >> 8),
        (uint8_t)(address_0based & 0xFF),
        (uint8_t)(num_registers >> 8),
        (uint8_t)(num_registers & 0xFF),
        (uint8_t)(num_registers * 2)    // Byte Count (2 bytes per register)
    };

    // 2. Append the Register Data (Big Endian)
    for (uint16_t i = 0; i < num_registers; i++) 
    {
        request_data.push_back((uint8_t)(values[i] >> 8));
        request_data.push_back((uint8_t)(values[i] & 0xFF));
    }

    // 3. Append CRC-16
    _appendCrc(request_data);

    // std::cout << "Sending Write Multiple: ";
    // printHex(request_data);

    if (!_serialSend(request_data)) return false;

    // 4. Receive Response (8 bytes for Function 0x10)
    // Response format: [ID][0x10][AddrH][AddrL][QtyH][QtyL][CRCL][CRCH]
    auto response = _receiveRtuResponse(slaveID, 0x10, 8, parameters.RESPONSE_TIMEOUT_MS);
    if (response.empty()) return false;

    // Optional: validate echo fields (addr + qty)
    const uint16_t raddr = (static_cast<uint16_t>(response[2]) << 8) | response[3];
    const uint16_t rqty  = (static_cast<uint16_t>(response[4]) << 8) | response[5];
    if (raddr != address_0based || rqty != num_registers)
    {
        errorMessage = "0x10 response echo mismatch.";
        return false;
    }

    // std::cout << "Multi-Register Write Successful!" << std::endl;
    return true;
}

// ------------------------
// Debug
// ------------------------

void Modbus::printHex(const std::vector<uint8_t>& data) 
{
    for (const auto& byte : data) 
    {
        std::cout << "{" << std::hex << std::setw(2) << std::setfill('0') << (int)byte << "}";
    }
    std::cout << std::dec << std::endl;
}

// ------------------------
// CRC16 (Modbus RTU)
// ------------------------

uint16_t Modbus::_crc16_modbus(const std::vector<uint8_t>& data) 
{
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) 
    {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) 
        {
            if (crc & 0x0001) 
            {
                crc >>= 1;
                crc ^= 0xA001;
            } 
            else 
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t Modbus::_crc16_modbus(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
        {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}

bool Modbus::_verifyCrc(const std::vector<uint8_t>& frame)
{
    if (frame.size() < 4) return false;
    const size_t n = frame.size();
    const uint16_t got = static_cast<uint16_t>(frame[n-2]) | (static_cast<uint16_t>(frame[n-1]) << 8);
    const uint16_t calc = _crc16_modbus(frame.data(), n - 2);
    return got == calc;
}

void Modbus::_appendCrc(std::vector<uint8_t>& frame)
{
    const uint16_t crc = Modbus::_crc16_modbus(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc >> 8));
}

// ------------------------
// Address normalization
// ------------------------
// Default: 0-based. Best-effort auto-normalize common human notations.
uint16_t Modbus::_normalizeHoldingRegisterAddress(uint16_t addr)
{
    if (addr >= 40001 && addr <= 49999) return static_cast<uint16_t>(addr - 40001);
    return addr;
}
uint16_t Modbus::_normalizeInputRegisterAddress(uint16_t addr)
{
    if (addr >= 30001 && addr <= 39999) return static_cast<uint16_t>(addr - 30001);
    return addr;
}
uint16_t Modbus::_normalizeCoilAddress(uint16_t addr)
{
    // Some people use 00001-based coil numbers
    if (addr >= 1 && addr <= 9999) return static_cast<uint16_t>(addr - 1);
    return addr;
}
uint16_t Modbus::_normalizeDiscreteInputAddress(uint16_t addr)
{
    if (addr >= 10001 && addr <= 19999) return static_cast<uint16_t>(addr - 10001);
    return addr;
}