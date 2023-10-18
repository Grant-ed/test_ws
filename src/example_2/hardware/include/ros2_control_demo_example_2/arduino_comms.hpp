#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

//Communications Defines
#define START_BYTE 0x11 // Device Control 1

#define BIG_ENDIAN 1 // 1 for big endian, 0 for little endian

#define DGRAM_MAX_LENGTH 10 // bytes

#define CRC_8_POLY 0x97 // CRC-8 polynomial

#define DEBUG_COMMS 1 // 1 for debug, 0 for no debug

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  /**
   * @brief Send a message to the HAT
   * @param data_to_send DataBuffer to send to HAT
   * @return response from HAT
   */
  bool send_bytes(const LibSerial::DataBuffer &data_to_send)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    // first byte needs to be the start byte, second byte is crc8
    LibSerial::DataBuffer data_to_send_with_start_byte;
    data_to_send_with_start_byte.push_back(START_BYTE);
    data_to_send_with_start_byte.push_back(crc8(data_to_send));

    serial_conn_.Write(data_to_send_with_start_byte);

#if DEBUG_COMMS
    std::cout << "Sent: " << databuffer_to_string(data_to_send_with_start_byte) << std::endl;
#endif
  }


  /**
   * @brief Send a message to the HAT and wait for a response
   * @param msg string to send to HAT
   * @return response from HAT
   */
  LibSerial::DataBuffer send_recv_bytes(const LibSerial::DataBuffer &data_to_send)
  {
    send_bytes(data_to_send);

    // Wait for response
    LibSerial::DataBuffer response;
    serial_conn_.Read(response, 1, timeout_ms_);
  
  }


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
          crc =  (crc >> 1);
        }
      }
    }
    return crc;
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    
  }

  void set_motor_values(int val_1, int val_2)
  {
    
  }


#if DEBUG_COMMS
  // helper function to convert DataBuffer to string
  std::string databuffer_to_string(const LibSerial::DataBuffer &data)
  {
    std::string msg = "";
    // convert DataBuffer (which is std::vector<uint8_t>) to string in for loop
    for (auto it = data.begin(); it != data.end(); ++it)
    {
        msg += std::to_string(*it) + ".";
    }
    return msg;
  }
#endif

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

// main method for testing
int main()
{
  ArduinoComms comms;
  comms.connect("/dev/ttyACM0", 115200, 1000);
  std::cout << "Connected: " << comms.connected() << std::endl;

  // test sending 0x13.0x1.0x0.0x0.0x21
  LibSerial::DataBuffer data_to_send = {0x13, 0x1, 0x0, 0x0, 0x0, 0x21};
  comms.send_bytes(data_to_send);

  // test sending 0x13.0x1.0xa.0xa.0x18
  LibSerial::DataBuffer data_to_send_2 = {0x13, 0x1, 0xa, 0xa, 0x18};
  comms.send_bytes(data_to_send_2);
}


#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP