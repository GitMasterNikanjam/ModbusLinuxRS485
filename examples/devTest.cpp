/*
mkdir -p ./bin && g++ ex1.cpp -o ./bin/ex1
sudo ./bin/ex1
*/

// #############################################################################
// Include libraries:

#include "../ModbusLinuxRS485.h"

// #############################################################################
// Parameters:

#define DEVICE_PORT             "/dev/ttyS1"    // Confirmed working port
#define BAUD_RATE               B115200         // Confirmed working baud rate
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

    std::cout << "\n=========================================" << std::endl;
    std::cout << "1. Reading Alarm code (40001, 40002)" << std::endl;
    std::cout << "=========================================" << std::endl;

    std::vector<uint8_t> status_data = modbus.readHoldingRegisters(SLAVE_ID, 40001, 2);

    if (status_data.size() == 2) 
    {
        uint16_t status_word = (uint16_t)status_data[0] << 8 | status_data[1];
        std::cout << "\nSTATUS REGISTER 40002: 0x" 
                  << std::hex << std::setw(4) << std::setfill('0') << status_word << std::dec << std::endl;
    }

    std::cout << "\n==========================================================" << std::endl;
    std::cout << "2. Reading Status Code (40003 and 40004)" << std::endl;
    std::cout << "==========================================================" << std::endl;
    
    std::vector<uint8_t> sc_data = modbus.readHoldingRegisters(SLAVE_ID, 40003, 2);
    
    if (sc_data.size() == 4) 
    {
        // Data is received as 40003 (High Word) then 40004 (Low Word)
        uint32_t status_code = (uint32_t)sc_data[0] << 24 | (uint32_t)sc_data[1] << 16 |
                               (uint32_t)sc_data[2] << 8 | sc_data[3];
        
        std::cout << "\nSTATUS CODE 40003/40004 (32-bit): 0x" 
                  << std::hex << std::setw(8) << std::setfill('0') << status_code << std::dec << std::endl;

        // Example Interpretation (refer to manual section 11.6.1 for bit definitions)
        if (status_code & 0x00000001) 
        {
            std::cout << " -> Bit 2 (0x00000004) is set: Servo is ON." << std::endl;
        } 
        else 
        {
            std::cout << " -> Servo is OFF." << std::endl;
        }
    }
    
    return 0;
}