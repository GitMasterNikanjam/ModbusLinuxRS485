/*
mkdir -p ./bin && g++ ex1.cpp ../*.cpp -o ./bin/ex1
sudo ./bin/ex1
*/

#include "../ModbusLinuxRS485.h"
#include <iostream>
#include <vector>
#include <cstdint>

#define DEVICE_PORT   "/dev/ttyS1"
#define BAUD_RATE     115200
#define SLAVE_ID      1

int main()
{
    Modbus modbus;
    modbus.parameters.PORT = DEVICE_PORT;
    modbus.parameters.BAUDRATE = BAUD_RATE;
    modbus.parameters.TRANSMIT_DELAY_US = 3000;   // <-- FIX: correct field name (microseconds)

    std::cout << "Attempting to open port " << DEVICE_PORT << "...\n";
    if (!modbus.openPort())
    {
        std::cerr << "FATAL: Could not open serial port.\n";
        std::cerr << modbus.errorMessage << "\n";
        return 1;
    }

    // Read 2 holding registers starting at 40351
    auto response = modbus.readHoldingRegisters(SLAVE_ID, 40351, 2);

    if (!response.empty())
    {
        std::cout << "Received data bytes: ";
        modbus.printHex(response);

        if (response.size() == 4)
        {
            uint16_t r0 = (uint16_t(response[0]) << 8) | response[1];
            uint16_t r1 = (uint16_t(response[2]) << 8) | response[3];
            std::cout << "Decoded: r0=" << r0 << " r1=" << r1 << "\n";
        }
    }
    else
    {
        std::cerr << "Read failed: " << modbus.errorMessage << "\n";
    }

    // Write -1000 as signed 32-bit across 2 registers (high word, low word)
    int32_t absolute_pulses = 10000;
    uint32_t u = static_cast<uint32_t>(absolute_pulses);

    std::vector<uint16_t> regs = {
        static_cast<uint16_t>((u >> 16) & 0xFFFF),
        static_cast<uint16_t>(u & 0xFFFF)
    };

    if (!modbus.writeMultipleRegisters(SLAVE_ID, 40351, regs))
        std::cerr << "WriteMultipleRegisters failed: " << modbus.errorMessage << "\n";

    if (!modbus.writeSingleRegister(SLAVE_ID, 40125, 0x0067))
        std::cerr << "WriteSingleRegister failed: " << modbus.errorMessage << "\n";

    return 0;
}
