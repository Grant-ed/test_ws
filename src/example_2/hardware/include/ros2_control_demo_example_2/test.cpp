#include "arduino_comms.hpp"

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