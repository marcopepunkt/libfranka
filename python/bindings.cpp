#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include "examples_common.h"


namespace py = pybind11;

PYBIND11_MODULE(franka_py, m){
    m.doc() = "Franka Emika Panda robot bindings for Python. This is a simple example";
    py::class_<franka::Robot>(m, "Robot", "Some description of the class")
      .def(py::init<const std::string&>())
      .def("stop", &franka::Robot::stop, "Stop the robot")
      .def("read_once", &franka::Robot::readOnce, "Read robot state once")
      .def("read", &franka::Robot::read, "Read with callback");
      //.def("get_joint_positions", &robot::read, "Get the joint states")
    m.def("set_default_behavior", &setDefaultBehavior, "Set default behavior");

    py::class_<franka::RobotState>(m, "RobotState", "Get the Stateoftherobot")
      .def(py::init<>())
      .def_readonly("q", &franka::RobotState::q)
      .def_readonly("q_d", &franka::RobotState::q_d)
      .def_readonly("dq", &franka::RobotState::dq)
      .def_readonly("dq_d", &franka::RobotState::dq_d)
      .def_readonly("ddq_d", &franka::RobotState::ddq_d);

    m.def("move_to_joint_position", &moveToJointPosition, "Move to joint position");
  }
  