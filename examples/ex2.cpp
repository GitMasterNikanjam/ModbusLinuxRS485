/*
mkdir -p ./bin && g++ ex2.cpp ../*.cpp -o ./bin/ex2
sudo ./bin/ex2
*/

#include "../ModbusLinuxRS485.h"
#include <iostream>
#include <vector>
#include <cstdint>
#include <chrono>

#define DEVICE_PORT   "/dev/ttyS1"
#define BAUD_RATE     115200
#define SLAVE_ID      1

#define REG_ENCODER_POSITION                    40011
#define LEN_ENCODER_POSITION                    2

int main()
{
    Modbus modbus;
    modbus.parameters.PORT = DEVICE_PORT;
    modbus.parameters.BAUDRATE = BAUD_RATE;
    modbus.parameters.TRANSMIT_DELAY_US = 0;   // <-- FIX: correct field name (microseconds)
    modbus.parameters.RESPONSE_TIMEOUT_MS = 1;

    std::cout << "Attempting to open port " << DEVICE_PORT << "...\n";
    if (!modbus.openPort())
    {
        std::cerr << "FATAL: Could not open serial port.\n";
        std::cerr << modbus.errorMessage << "\n";
        return 1;
    }

    while(1)
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        uint16_t buffer[2] = {0, 0};

        if(modbus.readHoldingRegisters(SLAVE_ID, REG_ENCODER_POSITION, LEN_ENCODER_POSITION, buffer) == false)
        {
            std::cout << modbus.errorMessage << std::endl;
        }

        int32_t raw = (int32_t)(((uint32_t)buffer[0] << 16) | buffer[1]);

        float pos = (float)raw * 360.0 / (float)10000 / 1.0;
        
        auto t_end = std::chrono::high_resolution_clock::now();

        auto us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

        std::cout << "pos=" << pos << ", " << "loop_us=" << us << std::endl;
    }

    return 0;
}
