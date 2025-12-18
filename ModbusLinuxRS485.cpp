
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
#include <poll.h>

// ################################################################################
// Modbus class:

// ------------------------
// Lifecycle
// ------------------------

Modbus::Modbus()
{
    parameters.PORT = "";
    parameters.BAUDRATE = 9600;
    parameters.TRANSMIT_DELAY_US = 3000;

    _serialPort = -1;
}

Modbus::~Modbus()
{
    if (_serialPort >= 0) 
    {
        close(_serialPort);
        _serialPort = -1;
    }
}

void Modbus::closePort()
{
    if (_serialPort >= 0)
    {
        ::close(_serialPort);
        _serialPort = -1;
    }
}

bool Modbus::isPortOpen() const
{ 
    return (_serialPort >= 0); 
}

// ------------------------
// Port open/config
// ------------------------

bool Modbus::openPort(void)
{
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
    _serialPort = open(parameters.PORT.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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

    tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL; 
    tty.c_lflag = 0; tty.c_oflag = 0; 

    // VMIN=1 (wait for first byte), VTIME=10 (1 second timeout for subsequent bytes)
    tty.c_cc[VMIN]  = 1; 
    tty.c_cc[VTIME] = (cc_t)(parameters.RESPONSE_TIMEOUT_MS / 100.0); 

    if (tcsetattr(_serialPort, TCSANOW, &tty) != 0) 
    {
        std::ostringstream oss;
        oss << "Error setting port attributes: " << strerror(errno);
        errorMessage = oss.str();

        close(_serialPort); 
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

    // Flush any stale input before a request.
    tcflush(_serialPort, TCIFLUSH); 
    tcdrain(_serialPort); 

    ssize_t n = write(_serialPort, data, len);
    if (n < 0 || (size_t)n != len) 
    {
        std::ostringstream oss;
        oss << "Error writing data: " << strerror(errno);
        errorMessage = oss.str();

        return false;
    }

    // Ensure request is physically transmitted before turnaround delay.
    tcdrain(_serialPort);

    if (parameters.TRANSMIT_DELAY_US > 0)
    {
        usleep(parameters.TRANSMIT_DELAY_US); // Bus Turnaround Delay
    }
    
    return true;
}

// std::vector<uint8_t> Modbus::_serialReceive(size_t expected_length)
// {
//     if (!isPortOpen()) 
//     {
//         errorMessage = "Receive failed, port is closed.";
//         return {};
//     }

//     std::vector<uint8_t> buffer(expected_length);

//     // This single call blocks until VMIN bytes or VTIME timeout occurs
//     ssize_t n = read(_serialPort, buffer.data(), expected_length);

//     if (n > 0) 
//     {
//         buffer.resize(n); 
//         return buffer;
//     } 
//     else if (n == 0) 
//     {
//         std::ostringstream oss;
//         oss << "Timeout during serial read (VTIME expired).";
//         errorMessage = oss.str();
//     } 
//     else 
//     {
//         std::ostringstream oss;
//         oss << "Error reading serial data: " << strerror(errno);
//         errorMessage = oss.str();
//     }

//     return {};
// }

size_t Modbus::_serialReceive(size_t expected_length, uint8_t* buffer)
{
if (!isPortOpen()) 
    {
        errorMessage = "Receive failed, port is closed.";
        return 0;
    }

    // This single call blocks until VMIN bytes or VTIME timeout occurs
    ssize_t n = read(_serialPort, buffer, expected_length);

    if (n > 0) 
    {
        return n;
    } 
    else if (n == 0) 
    {
        std::ostringstream oss;
        oss << "Timeout during serial read (VTIME expired).";
        errorMessage = oss.str();
    } 
    else 
    {
        std::ostringstream oss;
        oss << "Error reading serial data: " << strerror(errno);
        errorMessage = oss.str();
    }

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
    int remaining_timeout = static_cast<int>(timeout_ms);

    while (out.size() < n)
    {
        pollfd pfd{fd, POLLIN, 0};
        const int rc = ::poll(&pfd, 1, remaining_timeout);
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

        uint8_t buf[256];
        const size_t want = std::min(sizeof(buf), n - out.size());
        const ssize_t got = ::read(fd, buf, want);
        if (got < 0)
        {
            if (errno == EINTR) continue;
            std::ostringstream oss;
            oss << "read failed: " << std::strerror(errno);
            errorMessage = oss.str();
            return {};
        }
        if (got == 0)
        {
            // Shouldn't happen with poll(POLLIN), but handle defensively.
            continue;
        }

        out.insert(out.end(), buf, buf + got);
        // remaining_timeout: we keep total timeout, not per-chunk
        // poll() already consumed time; easiest is to re-poll with same remaining_timeout.
        // A more precise implementation could track elapsed time with clock_gettime.
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
    // For read functions, expected_fixed_len = 0; we will read header to compute full len.
    // For write echoes/acks, expected_fixed_len is known (usually 8).
    if (expected_fixed_len != 0)
    {
        auto resp = _serialReceive(expected_fixed_len, timeout_ms);
        if (resp.empty()) return {};
        if (!_validateBasicResponse(resp, expected_slave, expected_function)) return {};
        return resp;
    }

    // Variable-length: read first 2 bytes to detect exception, then 3rd byte (byte-count) for normal response.
    auto first2 = _serialReceive(2, timeout_ms);
    if (first2.empty()) return {};

    // If function indicates exception, read remaining 3 bytes (exc code + CRC2) => total 5 bytes
    if (first2.size() == 2 && first2[1] == static_cast<uint8_t>(expected_function | 0x80))
    {
        auto rest3 = _serialReceive(3, timeout_ms);
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
    auto third = _serialReceive(1, timeout_ms);
    if (third.empty()) return {};
    const uint8_t byteCount = third[0];

    // Remaining = byteCount data + CRC2
    auto rest = _serialReceive(static_cast<size_t>(byteCount) + 2, timeout_ms);
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
    return _receiveRtuResponse(slaveID, 0x01, 0, parameters.RESPONSE_TIMEOUT_MS);
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
    return _receiveRtuResponse(slaveID, 0x02, 0, parameters.RESPONSE_TIMEOUT_MS);
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
    return !resp.empty();
}

std::vector<uint8_t> Modbus::readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers)
{
    if (!isPortOpen()) 
    {
        errorMessage = "Read failed. Port is closed.";
        return {};
    }

    if (num_registers == 0 || num_registers > 125) 
    {
        std::ostringstream oss;
        oss << "ERROR: Invalid number of registers (" << num_registers << ").";
        errorMessage = oss.str();
        
        return {};
    }

    // 1. Build the Modbus Request (6 bytes)
    // Modbus 4XXXX registers use 0-based addressing (40001 -> 0x0000)
    uint16_t address_0based = starting_address - 40001; 

    std::vector<uint8_t> request_data = 
    {
        slaveID,
        0x03,                           // Function Code: 03 (Read Holding Registers)
        (uint8_t)(address_0based >> 8),
        (uint8_t)(address_0based & 0xFF),
        (uint8_t)(num_registers >> 8),
        (uint8_t)(num_registers & 0xFF)
    };

    // 2. Calculate and Append CRC-16
    uint16_t crc = _crc16_modbus(request_data);
    request_data.push_back((uint8_t)(crc & 0xFF)); // CRC Low Byte
    request_data.push_back((uint8_t)(crc >> 8));  // CRC High Byte

    if (!_serialSend(request_data)) return {};

    // 3. Calculate Expected Response Length: ID(1), Func(1), ByteCnt(1), Data(2*N), CRC(2)
    size_t expected_length = 5 + (size_t)num_registers * 2; 

    // 4. Receive Response
    std::vector<uint8_t> response = _serialReceive(expected_length);

    if (response.size() != expected_length) 
    {
        std::ostringstream oss;
        oss << "ERROR: Incomplete response (Expected " << expected_length 
                << " bytes, Received " << response.size() << " bytes).";
        errorMessage = oss.str();
        
        return {};
    }

    // 5. Basic Validation and Error Check
    if (response[0] != slaveID) 
    {
        std::ostringstream oss;
        oss << "ERROR: Slave ID mismatch.";
        errorMessage = oss.str();

        return {};
    }

    // Check for exception response (Function code + 0x80)
    if (response[1] == (0x03 | 0x80)) 
    { 
        std::ostringstream oss;
        oss << "ERROR: Modbus Exception Code 0x" << std::hex << (int)response[2] << std::dec;
        errorMessage = oss.str();

        // Refer to manual for exception codes (e.g., 0x02 = Illegal Data Address)
        return {};
    }

    // Check function code and byte count
    if (response[1] != 0x03 || response[2] != (uint8_t)(num_registers * 2)) 
    {
        std::ostringstream oss;
        oss << "ERROR: Response format error (Func Code: " << (int)response[1] << ", Byte Count: " << (int)response[2] << ").";
        errorMessage = oss.str();

        return {};
    }

    // 6. Extract and Return ONLY the data bytes
    return std::vector<uint8_t>(response.begin() + 3, response.end() - 2);
}

bool Modbus::readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers, uint16_t* buffer)
{
    if (!isPortOpen()) 
    {
        errorMessage = "Read failed. Port is closed.";
        return false;
    }

    if (num_registers == 0 || num_registers > 125) 
    {
        std::ostringstream oss;
        oss << "ERROR: Invalid number of registers (" << num_registers << ").";
        errorMessage = oss.str();
        
        return false;
    }

    // 1. Build the Modbus Request (6 bytes)
    // Modbus 4XXXX registers use 0-based addressing (40001 -> 0x0000)
    uint16_t address_0based = starting_address - 40001; 

    std::vector<uint8_t> request_data = 
    {
        slaveID,
        0x03,                           // Function Code: 03 (Read Holding Registers)
        (uint8_t)(address_0based >> 8),
        (uint8_t)(address_0based & 0xFF),
        (uint8_t)(num_registers >> 8),
        (uint8_t)(num_registers & 0xFF)
    };

    // 2. Calculate and Append CRC-16
    uint16_t crc = _crc16_modbus(request_data);
    request_data.push_back((uint8_t)(crc & 0xFF)); // CRC Low Byte
    request_data.push_back((uint8_t)(crc >> 8));  // CRC High Byte

    if (!_serialSend(request_data)) return false;

    // 3. Calculate Expected Response Length: ID(1), Func(1), ByteCnt(1), Data(2*N), CRC(2)
    size_t expected_length = 5 + (size_t)num_registers * 2; 

    // 4. Receive Response
    // std::vector<uint8_t> response = _serialReceive(expected_length);
    uint8_t response[expected_length];
    size_t nRead = _serialReceive(expected_length, response);

    if (nRead != expected_length) 
    {
        std::ostringstream oss;
        oss << "ERROR: Incomplete response (Expected " << expected_length 
                << " bytes, Received " << nRead << " bytes).";
        errorMessage = oss.str();
        
        return false;
    }

    // 5. Basic Validation and Error Check
    if (response[0] != slaveID) 
    {
        std::ostringstream oss;
        oss << "ERROR: Slave ID mismatch.";
        errorMessage = oss.str();

        return false;
    }

    // Check for exception response (Function code + 0x80)
    if (response[1] == (0x03 | 0x80)) 
    { 
        std::ostringstream oss;
        oss << "ERROR: Modbus Exception Code 0x" << std::hex << (int)response[2] << std::dec;
        errorMessage = oss.str();

        // Refer to manual for exception codes (e.g., 0x02 = Illegal Data Address)
        return false;
    }

    // Check function code and byte count
    if (response[1] != 0x03 || response[2] != (uint8_t)(num_registers * 2)) 
    {
        std::ostringstream oss;
        oss << "ERROR: Response format error (Func Code: " << (int)response[1] << ", Byte Count: " << (int)response[2] << ").";
        errorMessage = oss.str();

        return false;
    }

    // 6. Extract and Return ONLY the data bytes
    for(uint8_t i = 0; i < num_registers; i++)
    {
        buffer[i] = ( (static_cast<uint16_t>(response[2*i]) << 8) | static_cast<uint16_t>(response[2*i + 1]) );
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

    return _receiveRtuResponse(slaveID, 0x04, 0, parameters.RESPONSE_TIMEOUT_MS);
}

bool Modbus::readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, uint16_t* out)
{
    if (!out) { errorMessage = "Output buffer is null."; return false; }
    auto resp = readInputRegisters(slaveID, starting_address, quantity);
    if (resp.empty()) return false;

    const uint8_t byteCount = resp[2];
    if (byteCount != quantity * 2) { errorMessage = "Byte-count mismatch in response."; return false; }

    for (uint16_t i = 0; i < quantity; ++i)
        out[i] = static_cast<uint16_t>(resp[3 + 2*i] << 8) | resp[3 + 2*i + 1];

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
    uint16_t address_0based = starting_address - 40001; 

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
    uint16_t crc = _crc16_modbus(request_data);
    request_data.push_back((uint8_t)(crc & 0xFF)); // CRC Low
    request_data.push_back((uint8_t)(crc >> 8));   // CRC High

    // std::cout << "Sending Write Command: ";
    // printHex(request_data);

    if (!_serialSend(request_data)) 
    {
        return false;
    }

    // 4. Receive Response (Function 06 echoes the request back exactly)
    size_t expected_length = 8; 
    std::vector<uint8_t> response = _serialReceive(expected_length);

    if (response.size() != expected_length) 
    {
        std::ostringstream oss;
        oss << "ERROR: Write response timeout or incomplete.";
        errorMessage = oss.str();

        return false;
    }

    // 5. Validation: A successful write returns the same data sent
    if (response[1] == (0x06 | 0x80)) 
    {
        std::ostringstream oss;
        oss << "MODBUS ERROR: Exception code 0x" << std::hex << (int)response[2] << std::dec;
        errorMessage = oss.str();

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
    if (!isPortOpen() || values.empty()) return false;

    uint16_t num_registers = values.size();
    uint16_t address_0based = starting_address - 40001; 

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
    uint16_t crc = _crc16_modbus(request_data);
    request_data.push_back((uint8_t)(crc & 0xFF)); // CRC Low
    request_data.push_back((uint8_t)(crc >> 8));   // CRC High

    // std::cout << "Sending Write Multiple: ";
    // printHex(request_data);

    if (!_serialSend(request_data)) return false;

    // 4. Receive Response (8 bytes for Function 0x10)
    // Response format: [ID][0x10][AddrH][AddrL][QtyH][QtyL][CRCL][CRCH]
    size_t expected_length = 8; 
    std::vector<uint8_t> response = _serialReceive(expected_length);

    if (response.size() != expected_length) 
    {
        std::ostringstream oss;
        oss << "ERROR: Multi-write response timeout.";
        errorMessage = oss.str();

        return false;
    }

    // 5. Validation: Check for Modbus Exceptions
    if (response[1] == (0x10 | 0x80)) 
    {
        std::ostringstream oss;
        oss << "MODBUS ERROR: Exception code 0x" << std::hex << (int)response[2] << std::dec;
        errorMessage = oss.str();

        return false;
    }

    // std::cout << "Multi-Register Write Successful!" << std::endl;
    return true;
}

bool Modbus::writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers, const uint16_t* values)
{
    if (!isPortOpen() || (values == nullptr)) return false;

    uint16_t address_0based = starting_address - 40001; 

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
    uint16_t crc = _crc16_modbus(request_data);
    request_data.push_back((uint8_t)(crc & 0xFF)); // CRC Low
    request_data.push_back((uint8_t)(crc >> 8));   // CRC High

    // std::cout << "Sending Write Multiple: ";
    // printHex(request_data);

    if (!_serialSend(request_data)) return false;

    // 4. Receive Response (8 bytes for Function 0x10)
    // Response format: [ID][0x10][AddrH][AddrL][QtyH][QtyL][CRCL][CRCH]
    size_t expected_length = 8; 
    std::vector<uint8_t> response = _serialReceive(expected_length);

    if (response.size() != expected_length) 
    {
        std::ostringstream oss;
        oss << "ERROR: Multi-write response timeout.";
        errorMessage = oss.str();

        return false;
    }

    // 5. Validation: Check for Modbus Exceptions
    if (response[1] == (0x10 | 0x80)) 
    {
        std::ostringstream oss;
        oss << "MODBUS ERROR: Exception code 0x" << std::hex << (int)response[2] << std::dec;
        errorMessage = oss.str();

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