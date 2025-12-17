/*
mkdir -p ./bin && g++ ex1.cpp ../*.cpp -o ./bin/ex1
sudo ./bin/ex1
*/

// #############################################################################
// Include libraries:

#include "../ModbusLinuxRS485.h"

// #############################################################################
// Parameters:

#define DEVICE_PORT             "/dev/ttyS1"    // Confirmed working port
#define BAUD_RATE               115200         // Confirmed working baud rate
#define SLAVE_ID                1               // Confirmed working Slave ID

// ############################################################################
// --- Main Execution ---

int main() 
{
    Modbus modbus;
    modbus.parameters.PORT = DEVICE_PORT;
    modbus.parameters.BAUDRATE = BAUD_RATE;
    modbus.parameters.TRANSMIT_DELAY = 3000;

    std::cout << "Attempting to open port " << DEVICE_PORT << "..." << std::endl;
    if(modbus.openPort() == false)
    {
        std::cerr << "\nFATAL: Could not open serial port." << std::endl;
        std::cout << modbus.errorMessage << std::endl;
        return -1;
    }

    std::vector<uint8_t> response = modbus.readHoldingRegisters(SLAVE_ID, 40351, 2);

    if (response.size() > 0) 
    {
        std::cout << "Received Response: ";
        modbus.printHex(response);
    }

    int32_t absolute_pulses;
    std::vector<uint16_t> registers;
    
    absolute_pulses = -1000;
    registers = 
    {  
        (uint16_t)((uint32_t)absolute_pulses >> 16),    // Register 40126: High Word
        (uint16_t)((uint32_t)absolute_pulses & 0xFFFF)  // Register 40127: Low Word
    };

    modbus.writeMultipleRegisters(SLAVE_ID, 40351, registers);

    modbus.writeSingleRegister(SLAVE_ID, 40125, 0X67);

    return 0;
}