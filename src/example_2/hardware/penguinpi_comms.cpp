#include "ros2_control_demo_example_2/penguinpi_comms.h"

namespace ros2_control_demo_example_2
{
PenguinPiComms::PenguinPiComms() {
    pModule = NULL;
    pInstance = NULL;
    initialised = false;
}

PenguinPiComms::~PenguinPiComms() {
    Py_XDECREF(pInstance);
    Py_XDECREF(pModule);
    Py_Finalize();
}

void PenguinPiComms::init() {
    Py_Initialize();

    // Add the provided directory to sys.path
    PyObject *sysPath = PySys_GetObject((char*)"path");
    PyObject *path = PyUnicode_FromString("/home/pi/test_ws/install/ros2_control_demo_example_2/include/ros2_control_demo_example_2/ros2_control_demo_example_2/PenguinPiLib");
    PyList_Append(sysPath, path);
    Py_DECREF(path);

    PyObject *pName = PyUnicode_DecodeFSDefault("penguinPi");
    pModule = PyImport_Import(pName);
    Py_XDECREF(pName);
    
    if (pModule) {
        PyObject *pClass = PyObject_GetAttrString(pModule, "Multi");
        PyObject *pAddress = PyUnicode_FromString("AD_MULTI");
        PyObject *pArgs = PyTuple_Pack(1, pAddress);
        pInstance = PyObject_CallObject(pClass, pArgs);
        Py_XDECREF(pArgs);
        initialised = true;
    } else {
        PyErr_Print();
    }
}

void PenguinPiComms::close() {
    PyObject_CallMethod(pInstance, "close", NULL);
    initialised = false;
}

bool PenguinPiComms::connected() {
    // PyObject *result = PyObject_CallMethod(pInstance, "connected", NULL);
    // return PyObject_IsTrue(result);
    return initialised;
}

void PenguinPiComms::set_velocity(int8_t left, int8_t right) {
    PyObject *pVelocity = PyList_New(0);
    PyList_Append(pVelocity, PyLong_FromLong(left));
    PyList_Append(pVelocity, PyLong_FromLong(right));
    PyObject_CallMethod(pInstance, "set_velocity", "(O)", pVelocity);
}

uint16_t* PenguinPiComms::get_encoders() {
    static uint16_t encoders[2];
    PyObject *result = PyObject_CallMethod(pInstance, "get_encoders", NULL);
    if (PyList_Check(result) && PyList_Size(result) == 2) {
        encoders[0] = PyLong_AsUnsignedLong(PyList_GetItem(result, 0));
        encoders[1] = PyLong_AsUnsignedLong(PyList_GetItem(result, 1));
    }
    return encoders;
}

uint16_t* PenguinPiComms::set_velocity_get_encoders(int8_t left, int8_t right) {
    static uint16_t encoders[2];
    PyObject *pVelocity = PyList_New(0);
    PyList_Append(pVelocity, PyLong_FromLong(left));
    PyList_Append(pVelocity, PyLong_FromLong(right));
    PyObject *result = PyObject_CallMethod(pInstance, "setget_velocity_encoders", "(O)", pVelocity);
    if (PyList_Check(result) && PyList_Size(result) == 2) {
        encoders[0] = PyLong_AsUnsignedLong(PyList_GetItem(result, 0));
        encoders[1] = PyLong_AsUnsignedLong(PyList_GetItem(result, 1));
    }
    return encoders;
}

void PenguinPiComms::stop_all() {
    PyObject_CallMethod(pInstance, "stop_all", NULL);
}

void PenguinPiComms::clear_data() {
    PyObject_CallMethod(pInstance, "clear_data", NULL);
}

}

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(
//   ros2_control_demo_example_2::PenguinPiComms)