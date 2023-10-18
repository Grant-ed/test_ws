#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <iomanip>
#include <sstream>

// Communications Defines
#define START_BYTE 0x11 // Device Control 1
#define DGRAM_MAX_LENGTH 10 // bytes
#define CRC_8_POLY 0x97 // CRC-8 polynomial
#define DEBUG_COMMS 0 // 1 for debug, 0 for no debug

/**
 * @brief Enum for addresses
 */
enum Address
{
  AD_MOTORS = 16,
  AD_MOTOR_L = 18,
  AD_MOTOR_R = 17,
  AD_MULTI = 19
};

/**
 * @brief Enum for opCodes
 */
enum OpCode
{
  MULTI_CLEAR_DATA = 3,
  MOTOR_SET_ENC_ZERO = 4,
  MOTOR_GET_KVI = 132,
  MOTOR_GET_VEL = 129,
  MOTOR_GET_ENC = 130,
  MOTOR_SET_ENC_MODE = 5,
  MULTI_SET_VEL_GET_ENC = 130,
  MOTOR_GET_CONTROL_MODE = 134,
  MULTI_GET_ENC = 129,
  MOTOR_SET_KVP = 2,
  MOTOR_GET_KVP = 131,
  MOTOR_SET_KVI = 3,
  MOTOR_SET_CONTROL_MODE = 6,
  MOTOR_GET_ENC_MODE = 133,
  MULTI_SET_VEL = 1,
  MOTOR_SET_VEL = 1,
  MULTI_ALL_STOP = 2
};

/**
 * @brief Convert a baud rate to a LibSerial::BaudRate
 * @param baud_rate baud rate to convert
 * @return LibSerial::BaudRate
 */
LibSerial::BaudRate convert_baud_rate(int baud_rate);

/**
 * @brief Convert a DataBuffer to a string
 * @param data DataBuffer to convert
 * @return string representation of DataBuffer
 */
std::string databuffer_to_string(const LibSerial::DataBuffer &data);

/**
 * @brief Class for communicating with the PenguinPi HAT
 */
class ArduinoComms
{

public:
  /**
   * @brief Constructor
   */
  ArduinoComms();

  /**
   * @brief Destructor
   */
  ~ArduinoComms();

  /**
   * @brief Connect to the serial port
   * @param serial_device serial device to connect to
   * @param baud_rate baud rate to use
   * @param timeout_ms timeout in milliseconds
   */
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  /**
   * @brief Disconnect from the serial port
   */
  void disconnect();

  /**
   * @brief Check if the serial connection is open
   * @return true if serial connection is open, false otherwise
   */
  bool connected() const;

  /**
   * @brief Get the encoder values from the HAT
   * @param left_encoder left encoder value
   * @param right_encoder right encoder value
   */
  void read_encoders(uint16_t &left_encoder, uint16_t &right_encoder);

  /**
   * @brief Set the motor target velocities, from -100 to 100 (5 * encoder ticks per 20 ms)
   * @param left_motor left motor velocity
   * @param right_motor right motor velocity
   */
  void set_motor_vel(int8_t left_motor, int8_t right_motor);

  /**
   * @brief reset the encoder values to 0
   */ 
  void clear_data();

  /**
   * @brief Send a datagram to the HAT and wait for a response
   * @param address address of the HAT object
   * @param opCode opCode of the message
   * @param data DataBuffer to send to HAT
   * @return Response from HAT. Empty if no response expected, or if response is invalid.
   */
  LibSerial::DataBuffer send_datagram(Address address, OpCode opCode, LibSerial::DataBuffer &data);

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  /**
   * @brief cyclic redundancy check
   * @param data DataBuffer to calculate crc8 on
   * @return crc8 value
   */
  uint8_t crc8(const LibSerial::DataBuffer &data);

  /**
   * @brief Send a message to the HAT
   * @param data_to_send DataBuffer to send to HAT
   */
  void send_bytes(const LibSerial::DataBuffer &data_to_send);

  /**
   * @brief Recieve a message from the HAT
   * @return response from HAT
   */
  LibSerial::DataBuffer recieve_bytes();

  /**
   * @brief Validate a payload from the HAT
   * @param address address of the HAT object
   * @param opCode opCode of the message
   * @return true if payload is valid, false otherwise
   */
  bool validate_payload(LibSerial::DataBuffer &payload, Address address, OpCode opCode);
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP