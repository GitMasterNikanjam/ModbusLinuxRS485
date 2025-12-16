#pragma once

// ######################################################################################
// Include libraries:

#include <iostream>
#include <vector>
#include <iomanip>
#include <cstdint>
#include <termios.h>    // For terminal control (termios struct, baud rates)
#include <fcntl.h>      // For file control (open, close)
#include <unistd.h>     // For read, write, sleep, usleep
#include <cstring>      // For strerror, memset
#include <errno.h>      // For error numbers
#include <sstream>
#include <stdexcept>    // For runtime_error


// ######################################################################################
// Modbus class:

// --- Modbus Utility Class (Generalized) ---
class Modbus 
{
    public:

        /// @brief Last error message encountered.
        std::string errorMessage; 

        /**
         * @struct ParametersStructure
         * @brief Structure to store user-defined communication parameters.
         */   
        struct ParametersStructure
        {
            /**
             * @brief Serial port path (e.g., "/dev/ttyS0").
             * @note An example value: "/dev/ttyS0"
             */
            std::string PORT;     

            /**
             * @brief Baud rate for serial communication.
             * @note (Allowed: 1200, 2400, 4800, 9600, 19200).
             */
            uint32_t BAUDRATE;

            /**
             * @brief Delay value after send data. [us]
             * @note This delay is need for slave response time. 
             */
            uint32_t TRANSMIT_DELAY;
        }parameters;

        /**
         * @brief Constructor.
         */
        Modbus();

        ~Modbus();

        bool openPort(void);

        bool isPortOpen() const { return _serialPort >= 0; }

        /**
         * @brief Generalized Modbus Read Function (Function Code 0x03)
         * @note Returns a vector containing ONLY the data bytes (registers)
         */
        std::vector<uint8_t> readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

    private:

        /// @brief The file descriptor for RS485 serial port communication.
        int _serialPort;

        bool _serialSend(const std::vector<uint8_t>& data);

        std::vector<uint8_t> _serialReceive(size_t expected_length); 
};

// ##############################################################################