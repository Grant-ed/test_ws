#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    uint16_t enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    /**
     * @brief Constructor for the Wheel class, see setup() for more details
     */
    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    /**
     * @brief Function to setup the wheel
     * @param wheel_name The name of the wheel (should match the name in the URDF, e.g. "left_wheel")
     * @param counts_per_rev The number of encoder counts per revolution
     */    
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    /**
     * @brief Function to calculate the encoder angle in radians
     * @return The encoder angle in radians
     */
    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }

    /**
     * @brief Function to calculate the encoder angle in radians
     * @param encoder The encoder value
     * @return The encoder angle in radians
     */
    double calc_enc_angle(uint16_t encoder)
    {
      return encoder * rads_per_count;
    }

    /**
     * @brief Function to calculate the difference between two encoder values (taking into account overflow)
     * @param new_val The new encoder value
     * @param old_val The old encoder value
     * @return The difference between the two encoder values
     */
    int calc_delta_enc(uint16_t new_val, uint16_t old_val)
    {
      // convert to signed int
      int delta = new_val - old_val;
      // check for overflow
      if (delta > 32767)
      {
        delta -= 65536;
      }
      else if (delta < -32768)
      {
        delta += 65536;
      }
      return delta;
    }
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
