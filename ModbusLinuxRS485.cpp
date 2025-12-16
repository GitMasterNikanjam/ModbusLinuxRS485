
// ################################################################################
// Include libraries:

#include "ModbusLinuxRS485.h"

// ################################################################################
// Define macros:

#define RESPONSE_TIMEOUT_S      1               // Serial read timeout in seconds

// ################################################################################
// Helper functions:

// --- Helper function to print byte vectors in hex format ---
void print_hex(const std::vector<uint8_t>& data) 
{
    for (const auto& byte : data) 
    {
        std::cout << "{" << std::hex << std::setw(2) << std::setfill('0') << (int)byte << "}";
    }
    std::cout << std::dec << std::endl;
}

// --- CRC-16 (Modbus) Calculation Function ---
uint16_t calculate_crc16(const std::vector<uint8_t>& data) 
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

// ################################################################################
// Modbus class:

Modbus::Modbus()
{
    parameters.PORT = "";
    parameters.BAUDRATE = 9600;
    parameters.TRANSMIT_DELAY = 3000;

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

bool Modbus::openPort(void)
{
    speed_t baud_rate;
    switch (parameters.BAUDRATE) 
    {
        case 9600: baud_rate = B9600; break;
        case 19200: baud_rate = B19200; break;
        case 38400: baud_rate = B19200; break;
        case 57600: baud_rate = B19200; break;
        case 115200: baud_rate = B19200; break;
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
    tty.c_cc[VTIME] = (cc_t)(RESPONSE_TIMEOUT_S * 10); 

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

bool Modbus::_serialSend(const std::vector<uint8_t>& data)
{
    if (!isPortOpen()) 
    {
        errorMessage = "Send failed. Port is closed.";
        return false;
    }

    tcflush(_serialPort, TCIFLUSH); 
    tcdrain(_serialPort); 

    ssize_t n = write(_serialPort, data.data(), data.size());
    if (n < 0 || (size_t)n != data.size()) 
    {
        std::ostringstream oss;
        oss << "Error writing data: " << strerror(errno);
        errorMessage = oss.str();

        return false;
    }

    usleep(parameters.TRANSMIT_DELAY); // Bus Turnaround Delay

    return true;
}

std::vector<uint8_t> Modbus::_serialReceive(size_t expected_length)
{
    if (!isPortOpen()) 
    {
        errorMessage = "Receive failed, port is closed.";
        return {};
    }

    std::vector<uint8_t> buffer(expected_length);

    // This single call blocks until VMIN bytes or VTIME timeout occurs
    ssize_t n = read(_serialPort, buffer.data(), expected_length);

    if (n > 0) 
    {
        buffer.resize(n); 
        return buffer;
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

    return {};
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
    uint16_t crc = calculate_crc16(request_data);
    request_data.push_back((uint8_t)(crc & 0xFF)); // CRC Low Byte
    request_data.push_back((uint8_t)(crc >> 8));  // CRC High Byte

    std::cout << "Sending Command: ";
    print_hex(request_data);

    if (!_serialSend(request_data)) return {};

    // 3. Calculate Expected Response Length: ID(1), Func(1), ByteCnt(1), Data(2*N), CRC(2)
    size_t expected_length = 5 + (size_t)num_registers * 2; 

    // 4. Receive Response
    std::vector<uint8_t> response = _serialReceive(expected_length);

    if (response.size() != expected_length) {
        std::cerr << "ERROR: Incomplete response (Expected " << expected_length 
                << " bytes, Received " << response.size() << " bytes)." << std::endl;
        return {};
    }

    std::cout << "Received Response: ";
    print_hex(response);

    // 5. Basic Validation and Error Check
    if (response[0] != slaveID) 
    {
        std::cerr << "ERROR: Slave ID mismatch." << std::endl;
        return {};
    }

    // Check for exception response (Function code + 0x80)
    if (response[1] == (0x03 | 0x80)) 
    { 
        std::cerr << "ERROR: Modbus Exception Code 0x" << std::hex << (int)response[2] << std::dec << std::endl;
        // Refer to manual for exception codes (e.g., 0x02 = Illegal Data Address)
        return {};
    }

    // Check function code and byte count
    if (response[1] != 0x03 || response[2] != (uint8_t)(num_registers * 2)) 
    {
        std::cerr << "ERROR: Response format error (Func Code: " << (int)response[1] << ", Byte Count: " << (int)response[2] << ")." << std::endl;
        return {};
    }

    // 6. Extract and Return ONLY the data bytes
    return std::vector<uint8_t>(response.begin() + 3, response.end() - 2);
}

