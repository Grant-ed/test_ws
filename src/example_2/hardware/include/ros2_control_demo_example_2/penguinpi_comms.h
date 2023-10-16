#ifndef DIFFDRIVE_ARDUINO_PENGUINPI_COMMS_H
#define DIFFDRIVE_ARDUINO_PENGUINPI_COMMS_H

#include <cstdint>
#include <Python.h>

class PenguinPiComms {
public:
    PenguinPiComms();
    ~PenguinPiComms();

    void init();
    void close();
    bool connected();
    void set_velocity(int8_t left, int8_t right);
    uint16_t* get_encoders();
    uint16_t* set_velocity_get_encoders(int8_t left, int8_t right);
    void stop_all();
    void clear_data();

private:
    PyObject *pModule;
    PyObject *pInstance;
    bool initialised;
};

#endif // DIFFDRIVE_ARDUINO_PENGUINPI_COMMS_H
