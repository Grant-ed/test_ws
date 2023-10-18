#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <iomanip>

// Communications Defines
#define START_BYTE 0x11 // Device Control 1

// #define BIG_ENDIAN 1 // 1 for big endian, 0 for little endian

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
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

/**
 * @brief Convert a DataBuffer to a string
 * @param data DataBuffer to convert
 * @return string representation of DataBuffer
 */
std::string databuffer_to_string(const LibSerial::DataBuffer &data)
{
  std::stringstream ss;

  for (auto it = data.begin(); it != data.end(); ++it)
  {
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(*it) << ".";
  }

  // remove last period
  std::string s = ss.str();
  s.pop_back();

  return s;
}

/**
 * @brief Class for communicating with the PenguinPi HAT
 */
class ArduinoComms
{

public:
  /**
   * @brief Constructor
   */
  ArduinoComms() = default;

  /**
   * @brief Destructor
   */
  ~ArduinoComms()
  {
    if (connected())
    {
      disconnect();
    }
  }

  /**
   * @brief Connect to the serial port
   * @param serial_device serial device to connect to
   * @param baud_rate baud rate to use
   * @param timeout_ms timeout in milliseconds
   */
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    if (connected())
    {
      serial_conn_.FlushIOBuffers();
      disconnect();
    }

#if DEBUG_COMMS
    // print available serial ports
    std::vector<std::string> serial_ports = serial_conn_.GetAvailableSerialPorts();
    std::cout << "Available serial ports:" << std::endl;
    for (auto it = serial_ports.begin(); it != serial_ports.end(); ++it)
    {
      std::cout << *it << std::endl;
    }
#endif
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));

    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  }

  /**
   * @brief Disconnect from the serial port
   */
  void disconnect()
  {
    serial_conn_.Close();
  }

  /**
   * @brief Check if the serial connection is open
   * @return true if serial connection is open, false otherwise
   */
  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  /**
   * @brief Get the encoder values from the HAT
   * @param left_encoder left encoder value
   * @param right_encoder right encoder value
   */
  void read_encoders(uint16_t &left_encoder, uint16_t &right_encoder)
  {
    LibSerial::DataBuffer empty;
    LibSerial::DataBuffer response = send_datagram(AD_MULTI, MULTI_GET_ENC, empty);

    // first two bytes are left encoder, second two bytes are right encoder
    left_encoder = (response[0] << 8) | response[1];
    right_encoder = (response[2] << 8) | response[3];
  }

  /**
   * @brief Set the motor target velocities, from -100 to 100 (5 * encoder ticks per 20 ms)
   * @param left_motor left motor velocity
   * @param right_motor right motor velocity
   */
  void set_motor_vel(int8_t left_motor, int8_t right_motor)
  {
    LibSerial::DataBuffer data_to_send;
    data_to_send.push_back(left_motor);
    data_to_send.push_back(right_motor);
    send_datagram(AD_MULTI, MULTI_SET_VEL, data_to_send);
  }

  /**
   * @brief reset the encoder values to 0
   */ 
  void clear_data()
  {
    LibSerial::DataBuffer empty;
    send_datagram(AD_MULTI, MULTI_CLEAR_DATA, empty);
  }

  /**
   * @brief Send a datagram to the HAT and wait for a response
   * @param address address of the HAT object
   * @param opCode opCode of the message
   * @param data DataBuffer to send to HAT
   * @return Response from HAT. Empty if no response expected, or if response is invalid.
   */
  LibSerial::DataBuffer send_datagram(Address address, OpCode opCode, LibSerial::DataBuffer &data)
  {
    // data to send starts with address, opCode, then data
    LibSerial::DataBuffer data_to_send = {static_cast<uint8_t>(address), static_cast<uint8_t>(opCode)};
    data_to_send.insert(data_to_send.end(), data.begin(), data.end());
    send_bytes(data_to_send);

    // if no response is expected, return empty DataBuffer
    if ((opCode & 0x80) == 0)
    {
      return LibSerial::DataBuffer();
    }

    // wait for response
    LibSerial::DataBuffer response = recieve_bytes();

    // validate response
    if (!validate_payload(response, address, opCode))
    {
      return LibSerial::DataBuffer();
    }

    // return payload from byte 3 to end (first byte is length, second is address, third is opCode)
    return LibSerial::DataBuffer(response.begin() + 3, response.end());
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  /**
   * @brief cyclic redundancy check
   * @param data DataBuffer to calculate crc8 on
   * @return crc8 value
   */
  uint8_t crc8(const LibSerial::DataBuffer &data)
  {
    uint8_t crc = 0;
    for (auto it = data.begin(); it != data.end(); ++it)
    {
      crc = crc ^ *it;
      for (int j = 0; j < 8; j++)
      {
        if (crc & 1)
        {
          crc = (crc >> 1) ^ CRC_8_POLY;
        }
        else
        {
          crc = (crc >> 1);
        }
      }
    }
    return crc;
  }

  /**
   * @brief Send a message to the HAT
   * @param data_to_send DataBuffer to send to HAT
   */
  void send_bytes(const LibSerial::DataBuffer &data_to_send)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    // first byte needs to be the start byte, second byte is the length of the message (including the crc byte, but not the start byte), then the data, then the crc byte (crc needs to be calculated on the length byte, and the data)
    LibSerial::DataBuffer dgram = {static_cast<uint8_t>(data_to_send.size() + 2)};
    dgram.insert(dgram.end(), data_to_send.begin(), data_to_send.end());
    dgram.push_back(crc8(dgram));
    dgram.insert(dgram.begin(), START_BYTE);

    serial_conn_.Write(dgram);

#if DEBUG_COMMS
    std::cout << "Sent: " << databuffer_to_string(dgram) << std::endl;
#endif
  }

  /**
   * @brief Recieve a message from the HAT
   * @return response from HAT
   */
  LibSerial::DataBuffer recieve_bytes()
  {
    LibSerial::DataBuffer response;
    serial_conn_.Read(response, 1, timeout_ms_);
    if (response.size() == 0)
    {
      std::cout << "Error! Read time out!" << std::endl;
      return LibSerial::DataBuffer();
    }

    if (response[0] != START_BYTE)
    {
      // text from the Atmel, perhaps an error message
      if (response[0] == 0x0a)
      {
        // it's a linefeed
        std::cout << std::endl;
      }
      else
      {
        std::cout << static_cast<char>(response[0]);
      }
      return LibSerial::DataBuffer();
    }

    // read the rest
    LibSerial::DataBuffer paylen;
    serial_conn_.Read(paylen, 1, timeout_ms_);
    if (paylen.size() == 0)
    {
      std::cout << "Error! Read time out!" << std::endl;
      return LibSerial::DataBuffer();
    }
    int paylenint = paylen[0];

    LibSerial::DataBuffer dgram;
    serial_conn_.Read(dgram, paylenint - 1, timeout_ms_);
    if (dgram.size() != paylenint - 1)
    {
      std::cout << "Error! Short read!" << std::endl;
      return LibSerial::DataBuffer();
    }

    // datagram is length + the rest
    dgram.insert(dgram.begin(), paylen.begin(), paylen.end());

    // remove the crc byte
    uint8_t crcDgram = dgram.back();
    dgram.pop_back();

    // run crcCalc to ensure correct data
    uint8_t crcCalc = crc8(dgram);
    if (crcCalc != crcDgram)
    {
      std::cout << "Error! CRC Failed!" << std::endl;
      return LibSerial::DataBuffer();
    }

#if DEBUG_COMMS
    LibSerial::DataBuffer full_dgram = {START_BYTE};
    full_dgram.insert(full_dgram.end(), dgram.begin(), dgram.end());
    full_dgram.push_back(crcDgram);

    std::cout << "Recieved: " << databuffer_to_string(full_dgram) << std::endl;
#endif

    return dgram;
  }

  /**
   * @brief Validate a payload from the HAT
   * @param address address of the HAT object
   * @param opCode opCode of the message
   * @return true if payload is valid, false otherwise
   */
  bool validate_payload(LibSerial::DataBuffer &payload, Address address, OpCode opCode)
  {
    // check size
    if (payload.size() == 0)
    {
      return false;
    }

    // check address
    if (payload[1] != address)
    {
      std::cout << "Error! Response from HAT has wrong address!" << std::endl;
      return false;
    }

    // check opCode
    if (payload[2] != opCode)
    {
      std::cout << "Error! Response from HAT has wrong opCode!" << std::endl;
      return false;
    }

    return true;
  }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP