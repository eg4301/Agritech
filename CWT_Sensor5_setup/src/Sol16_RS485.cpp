#include "Sol16_RS485.h"
#include "Crc16.h"
#include <SoftwareSerial.h>

unsigned short Sol16_RS485Sensor::calc_crc(byte reading[], int start, int size) {
  Crc16 _crc;
  unsigned short calculated_crc = _crc.Modbus(reading, start, size);
  calculated_crc = (lowByte(calculated_crc) << 8) | highByte(calculated_crc); // swap the byte pos
  
  // Debugging print
  Serial.printf("Calculated CRC is: 0x%02x\n", calculated_crc);

  return calculated_crc;
}

void Sol16_RS485Sensor::add_crc(byte reading[], int start, int size) {
  Crc16 crc;
  unsigned short calculated_crc = crc.Modbus(reading, start, size);
  reading[size] = lowByte(calculated_crc) << 8;
  reading[size + 1] = highByte(calculated_crc);
  // calculated_crc = (lowByte(calculated_crc) << 8) | highByte(calculated_crc); // swap the byte pos
}

void Sol16_RS485Sensor::serial_flush() {
  Serial.println("Flushing sensor serial");
  while(this->available() > 0) {
    char t = this->read();
  }
}

void Sol16_RS485Sensor::setup(int control_pin, int rx_pin, int tx_pin, byte address, uint32_t baud_rate, Config protocol) {
  _ctrl_pin = control_pin;
  Serial.printf("Setting control pin to GPIO %i\n", _ctrl_pin);
  _rx_pin = rx_pin;
  Serial.printf("Setting rx pin to GPIO %i\n", _rx_pin);
  _tx_pin = tx_pin;
  Serial.printf("Setting tx pin to GPIO %i\n", _tx_pin);
  _address = address;
  Serial.printf("Setting address to 0x%02x\n", _address);
  Serial.printf("Baud rate with meter is %i\n", baud_rate);

  pinMode(_ctrl_pin, OUTPUT);
  digitalWrite(_ctrl_pin, RS485_RECEIVE); // Put RS485 in receive mode
  Serial.println("Setting RS485 to receive mode");
  this->begin(baud_rate, protocol); // Start the RS485 software serial port
}

void Sol16_RS485Sensor::request_reading(byte command[], int size) {
  // Check if setup is completed first
  if (_ctrl_pin == -1 || _rx_pin == -1 || _tx_pin == -1) {
    Serial.println("GPIO pins not declared yet. Call setup() first");
    return;
  }

  Serial.println("Requesting data from sensor");
  digitalWrite(_ctrl_pin, RS485_TRANSMIT); // Put RS485 in Trasmit mode
  Serial.println("RS485 put into transmit mode");

  // Debugging print
  Serial.println("Command send over:");
  for (int i = 0; i < size; i++) {
    Serial.printf("Byte[%i]: 0x%04x\n", i, command[i]);
  }

  this->write(command, size);
  digitalWrite(_ctrl_pin, RS485_RECEIVE); // Put RS485 back into Receive mode
  Serial.println("RS485 put back into receive mode");
}

bool Sol16_RS485Sensor::receive_reading(int num_bytes, int return_address_idx, int function_code_idx, byte reading[]) {
  // Check if setup is completed first
  if (_ctrl_pin == -1 || _rx_pin == -1 || _tx_pin == -1) {
    Serial.println("GPIO pins not declared yet. Call setup() first");
    return false;
  }

  Serial.printf("Number of expected bytes is %i\n", num_bytes);
  int num_sec = 0;
  while (this->available() < num_bytes && num_sec < 3) {
    Serial.println("Waiting for more bytes");
    Serial.printf("Number of bytes received currently is %i\n", this->available());
    num_sec++;
    delay(1000);
  }

  _num_received_bytes = this->available(); // For debugging
  Serial.printf("Total number of response bytes is %i\n", _num_received_bytes);
  if (num_sec == 3) {
    Serial.println("Timeout waiting for response from meter");
    this->serial_flush();
    return false;
  }

  Serial.println("Receiving data from sensor");
  for(int i = 0; i < num_bytes; ++i) {
    reading[i] = this->read();
    Serial.printf("Byte[%i]: 0x%02x\n", i, reading[i]);
  }
  
  // Checking if got any excess byte
  if (this->available() > 0) {
    Serial.println("Excess bytes received, clearing serial buffer");
    this->serial_flush();
  }

  // Check CRC of data received
  int start_idx = 0;
  int data_length = num_bytes-2;
  _response_address = reading[return_address_idx];      // For debugging
  _response_function_code = reading[function_code_idx]; // For debugging
  _calc_crc = calc_crc(reading, start_idx, data_length);
  _actual_crc = (reading[num_bytes-2] << 8 | reading[num_bytes-1]);
  if (_calc_crc == _actual_crc) {
    Serial.println("CRC match, data received correctly");
  } else {
    Serial.println("Corrupted data, CRC does not match. Resetting data array to 0x00");
    for (int i = 0; i < num_bytes; i++) {
      reading[i] = 0x00;
    }
    return false;
  }
  return true;
}