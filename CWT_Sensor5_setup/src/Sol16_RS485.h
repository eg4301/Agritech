#ifndef SOL16RS485_H
#define SOL16RS485_H

// RS485 Constants
#define RS485_TRANSMIT      HIGH
#define RS485_RECEIVE       LOW
#define RETURN_ADDRESS_IDX      0
#define RETURN_FUNCTIONCODE_IDX 1

#include "SoftwareSerial.h"

/*
 * Base class for RS485 Sensors
 */
class Sol16_RS485Sensor : virtual public SoftwareSerial{
  public:
    // Copy base constructor
    using SoftwareSerial::SoftwareSerial;

    /**
     * @brief Calculates the CRC of the received data bytes
     * 
     * @param reading Array of all data bytes received, including last 2 bytes of CRC
     * @param start Index of first data byte
     * @param size Size of array of data bytes, excluding the last 2 bytes of CRC
     * @return Calculated CRC of the data array usind Modbus
     */
    unsigned short calc_crc(byte reading[], int start, int size);

    /**
     * @brief Checks the received CRC of data bytes against the calculated CRC
     * 
     * @param data Array of all data bytes received, including last 2 bytes of CRC
     * @param size Size of array of data bytes, including the last 2 bytes of CRC
     * @return True if received CRC is equal to calculated CRC, false otherwise
     */
    bool check_crc(byte data[], int size);

    /**
     * @brief Flushes any remaining bytes left in the serial
     * 
     */
    void serial_flush();

    /**
     * @brief Initilizes the baud rate, protocol and setting of RS485
     * control pin for RX and TX. 
     * 
     * @param control_pin GPIO for RS485 direction control, connected to RE and DE
     * @param rx_pin GPIO for receiving serial output from sensor, connected to RO
     * @param tx_pin GPIO for transmiting serial data to sensor, connected to DI
     * @param address Address of sensor in hexadecimal format
     * @param baud_rate Baud rate of sensor
     * @param protocol Serial protocol of sensor
     */
    void setup(int control_pin, int rx_pin, int tx_pin, byte address, uint32_t baud_rate, Config protocol);

    /**
     * @brief Request the data from the sensor. Pulling of pins high and low to switch 
     * beteen receive and transmit mode called within function.
     * 
     * @param command Array of bytes to be sent out, including the calculated CRC
     * @param size Size of array to be sent
     */
    void request_reading(byte command[], int size);

    /**
     * @brief Receive serial data from the sensor. Calculated CRC is checked against received CRC.
     * If CRC does not match, byte array will be all set to 0x00.
     * 
     * @param num_bytes Expected number of bytes
     * @param return_address_idx Index of byte representing the address
     * @param function_code_idx Index of byte representing the function code
     * @param reading Byte array for the data to be saved into
     * @return True if data is received with correct CRC, false otherwise
     */
    bool receive_reading(int num_bytes, int return_address_idx, int function_code_idx, byte reading[]);

    // Helper objects and variables
    byte _address;           // Address of sensor for communication
    int _rx_pin = -1;               // Soft Serial Receive pin, connected to RO
    int _tx_pin = -1;               // Soft Serial Transmit pin, connected to DI
    int _ctrl_pin = -1;             // RS485 Direction control, connected to RE and DE
    unsigned short _actual_crc;     // For checking actual CRC recevied
    unsigned short _calc_crc;       // For checking CRC calculated from data bytes
    byte _response_address;         // For checking address code returned from meter
    byte _response_function_code;   // For checking function code returned from meter
    int _num_received_bytes;        // For checking number of bytes returned from meter
};


#endif