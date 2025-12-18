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

        enum class Parity : uint8_t { None, Even, Odd };

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
             * @brief Inter-frame transmit delay in microseconds. [us]
             *
             * This delay is applied after sending a Modbus request to allow
             * the slave device sufficient processing and response time.
             *
             * @note Required for some slow or heavily loaded slaves.
             */
            uint32_t TRANSMIT_DELAY_US;

            // Total response timeout for a complete RTU response
            uint32_t    RESPONSE_TIMEOUT_MS {1000};

            // Serial format (common Modbus RTU defaults are 8E1 or 8N1)
            Parity      PARITY {Parity::None};
            uint8_t     STOP_BITS {1};   // 1 or 2
            uint8_t     DATA_BITS {8};   // typically 8
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

        void closePort();

        /**
         * @brief Check whether the serial port is open.
         *
         * @return true if the serial port file descriptor is valid.
         */
        bool isPortOpen() const;

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
        std::vector<uint8_t> readCoils(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

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
        std::vector<uint8_t> readDiscreteInputs(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

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
        std::vector<uint8_t> readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

        /**
         * @brief Read holding registers (Function Code 0x03).
         *
         * Holding registers are read/write registers typically used for
         * configuration parameters or output values.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First register address (zero-based).
         * @param num_registers Number of registers to read.
         * @param buffer Array pointer to store ONLY the data registers (big-endian).
         *
         * @return true on success.
         */
        bool readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, uint16_t* buffer);

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
        std::vector<uint8_t> readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

        bool readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, uint16_t* out);

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
        bool writeSingleCoil(uint8_t slaveID, uint16_t address, bool state);

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
        bool writeSingleRegister(uint8_t slaveID, uint16_t address, uint16_t value);

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
        bool writeMultipleCoils(uint8_t slaveID, uint16_t starting_address, const std::vector<bool>& values);

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

        /**
         * @brief Write multiple holding registers (Function Code 0x10).
         *
         * Writes multiple consecutive 16-bit values to holding registers.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First register address (zero-based).
         * @param num_registers Number of registers in values array.
         * @param values Array of 16-bit values to write.
         *
         * @return true on success.
         */
        bool writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, const uint16_t* values);

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

        uint16_t _crc16_modbus(const std::vector<uint8_t>& data);
        static uint16_t _crc16_modbus(const uint8_t* data, size_t len);
        static bool     _verifyCrc(const std::vector<uint8_t>& frame);
        static void _appendCrc(std::vector<uint8_t>& frame);

        static uint16_t _normalizeHoldingRegisterAddress(uint16_t addr);
        static uint16_t _normalizeInputRegisterAddress(uint16_t addr);
        static uint16_t _normalizeCoilAddress(uint16_t addr);
        static uint16_t _normalizeDiscreteInputAddress(uint16_t addr);

        /**
         * @brief Send raw data over the serial port.
         *
         * @param data Vector containing the full Modbus RTU frame.
         *
         * @return true if transmission succeeded.
         */
        bool _serialSend(const std::vector<uint8_t>& data);

        /**
         * @brief Send raw data over the serial port.
         *
         * @param num is number of elemnts in data array.
         * @param data array containing the full Modbus RTU frame.
         * 
         * @return true if transmission succeeded.
         */
        bool _serialSend(const uint8_t* data, size_t len);

        // /**
        //  * @brief Receive raw data from the serial port.
        //  *
        //  * @param expected_length Expected number of bytes to receive.
        //  *
        //  * @return Vector containing received bytes.
        //  */
        // std::vector<uint8_t> _serialReceive(size_t expected_length); 

        // Reads exactly n bytes (unless timeout); returns empty on failure and sets errorMessage.
        std::vector<uint8_t> _serialReceive(size_t n, uint32_t timeout_ms = 1000);

        /**
         * @brief Receive raw data from the serial port.
         *
         * @param expected_length Expected number of bytes to receive.
         * @param buffer pointer to store received bytes.
         * 
         * @return number of read data.
         */
        size_t _serialReceive(size_t expected_length, uint8_t* buffer);

        // Reads Modbus RTU response for:
        // - fixed length responses (write single/multiple)
        // - variable length responses where byte-count is in the 3rd byte (read regs/coils)
        std::vector<uint8_t> _receiveRtuResponse(uint8_t expected_slave, uint8_t expected_function, size_t expected_fixed_len, uint32_t timeout_ms);

        bool _validateBasicResponse(const std::vector<uint8_t>& resp, uint8_t expected_slave, uint8_t expected_function);
};

// ##############################################################################