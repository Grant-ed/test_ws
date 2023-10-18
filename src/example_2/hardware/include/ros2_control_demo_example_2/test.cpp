#include "arduino_comms.hpp"

// main method for testing, two int arguments can be passed in for the motor velocities
int main(int argc, char **argv)
{
  ArduinoComms comms;
  comms.connect("/dev/serial0", 115200, 1000);
  std::cout << "Connected: " << comms.connected() << "\n" << std::endl;

  // if two arguments are passed in, send them as the motor velocities
  if (argc == 3)
  {
    int8_t motor_1_vel = atoi(argv[1]);
    int8_t motor_2_vel = atoi(argv[2]);
    comms.set_motor_vel(motor_1_vel, motor_2_vel);
  }
  else if (argc == 2)
  {
    int motor_vel = atoi(argv[1]);
    comms.set_motor_vel(motor_vel, motor_vel);
  }
  else
  {
    comms.set_motor_vel(0, 0);
  }

  comms.clear_data();

  uint16_t left_encoder;
  uint16_t right_encoder;
  comms.read_encoders(left_encoder, right_encoder);

  // print as a signed integer, not as a hex
  std::cout << "Encoders: (" << std::dec << left_encoder << ", " << std::dec << right_encoder << ")\n" << std::endl;

  comms.disconnect();
}