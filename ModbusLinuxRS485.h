#pragma once

// ######################################################################################
// Includes
// ######################################################################################

#include <vector>
#include <cstdint>
#include <cstring>              // For strerror, memset
#include <iostream>

// ######################################################################################
// Modbus class
// ######################################################################################

/**
 * @class Modbus
 * @brief Generic Modbus RTU communication utility for RS-485 serial devices.
 *
 * This class provides a lightweight abstraction for Modbus RTU master operations
 * over an RS-485 serial interface. It supports common Modbus function codes for
 * reading and writing coils and registers.
 *
 * @note
 * - This implementation assumes Modbus RTU framing.
 * - CRC handling is expected to be internal to the implementation.
 * - All register addresses are **zero-based Modbus addresses** (not 4xxxx notation).
 */
class Modbus 
{
    public:

        /**
         * @brief Human-readable description of the last error.
         *
         * This string is updated whenever an operation fails. It may contain
         * system-level errors (e.g., serial I/O failures) or protocol-level errors.
         */
        std::string errorMessage; 

        /**
         * @struct ParametersStructure
         * @brief Serial communication configuration parameters.
         *
         * These parameters must be configured before calling openPort().
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
             * @note (Allowed: 9600, 19200, 38400, 57600, 115200).
             */
            uint32_t BAUDRATE;

            /**
             * @brief Inter-frame transmit delay in microseconds.
             *
             * This delay is applied after sending a Modbus request to allow
             * the slave device sufficient processing and response time.
             *
             * @note Required for some slow or heavily loaded slaves.
             */
            uint32_t TRANSMIT_DELAY;
        }parameters;

        /**
         * @brief Construct a Modbus communication object.
         *
         * The serial port is not opened automatically. Call openPort()
         * after configuring @ref parameters.
         */
        Modbus();
        
        /**
         * @brief Destructor.
         *
         * Automatically closes the serial port if it is open.
         */
        ~Modbus();

        /**
         * @brief Open and configure the RS-485 serial port.
         *
         * @return true if the port was successfully opened and configured.
         * @return false on failure (see @ref errorMessage).
         */
        bool openPort(void);

        /**
         * @brief Check whether the serial port is open.
         *
         * @return true if the serial port file descriptor is valid.
         */
        bool isPortOpen() const { return _serialPort >= 0; }

        // -------------------------------------------------------------------------
        // Modbus Read Operations
        // -------------------------------------------------------------------------

        /**
         * @brief Read the status of coils (Function Code 0x01).
         *
         * Coils represent discrete digital outputs (ON/OFF).
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First coil address (zero-based).
         * @param num_registers Number of coils to read.
         *
         * @return Vector containing raw response data bytes.
         */
        std::vector<uint8_t> ReadCoils(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

        /**
         * @brief Read discrete inputs (Function Code 0x02).
         *
         * Discrete inputs represent read-only digital inputs such as switches.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First input address (zero-based).
         * @param num_registers Number of inputs to read.
         *
         * @return Vector containing raw response data bytes.
         */
        std::vector<uint8_t> ReadDiscreteInputs(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

        /**
         * @brief Read holding registers (Function Code 0x03).
         *
         * Holding registers are read/write registers typically used for
         * configuration parameters or output values.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First register address (zero-based).
         * @param num_registers Number of registers to read.
         *
         * @return Vector containing ONLY the data bytes (big-endian).
         */
        std::vector<uint8_t> readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

        /**
         * @brief Read input registers (Function Code 0x04).
         *
         * Input registers are read-only registers typically used for
         * sensor data such as temperature, pressure, or feedback values.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First register address (zero-based).
         * @param num_registers Number of registers to read.
         *
         * @return Vector containing raw response data bytes.
         */
        std::vector<uint8_t> ReadInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

        // -------------------------------------------------------------------------
        // Modbus Write Operations
        // -------------------------------------------------------------------------

        /**
         * @brief Write a single coil (Function Code 0x05).
         *
         * Used to turn a single digital output ON or OFF.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address Coil address (zero-based).
         * @param value Coil value (0x0000 = OFF, 0xFF00 = ON).
         *
         * @return true on success.
         */
        bool WriteSingleCoil(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

        /**
         * @brief Write a single holding register (Function Code 0x06).
         *
         * Writes one 16-bit value to a holding register.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address Register address (zero-based).
         * @param value 16-bit value to write.
         *
         * @return true on success.
         */
        bool writeSingleRegister(uint8_t slaveID, uint16_t starting_address, uint16_t value);

        /**
         * @brief Write multiple coils (Function Code 0x0F).
         *
         * Used to control multiple digital outputs simultaneously.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First coil address (zero-based).
         * @param num_registers Number of coils to write.
         *
         * @return true on success.
         */
        bool WriteMultipleCoils(uint8_t slaveID, uint16_t starting_address, uint16_t num_registers);

        /**
         * @brief Write multiple holding registers (Function Code 0x10).
         *
         * Writes multiple consecutive 16-bit values to holding registers.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First register address (zero-based).
         * @param values Vector of 16-bit values to write.
         *
         * @return true on success.
         */
        bool writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, const std::vector<uint16_t>& values);

        // -------------------------------------------------------------------------
        // Utility
        // -------------------------------------------------------------------------

        /**
         * @brief Print a byte vector in hexadecimal format.
         *
         * Useful for debugging Modbus frames and responses.
         *
         * @param data Vector of bytes to print.
         */
        void printHex(const std::vector<uint8_t>& data); 

    private:

        /**
         * @brief File descriptor for the RS-485 serial port.
         *
         * A value < 0 indicates that the port is not open.
         */
        int _serialPort;

        /**
         * @brief Send raw data over the serial port.
         *
         * @param data Vector containing the full Modbus RTU frame.
         *
         * @return true if transmission succeeded.
         */
        bool _serialSend(const std::vector<uint8_t>& data);

        /**
         * @brief Receive raw data from the serial port.
         *
         * @param expected_length Expected number of bytes to receive.
         *
         * @return Vector containing received bytes.
         */
        std::vector<uint8_t> _serialReceive(size_t expected_length); 
};

// ##############################################################################