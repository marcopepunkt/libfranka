#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <pybind11/eigen.h>
#include "examples_common.h"


namespace py = pybind11;

PYBIND11_MODULE(franka_py, m){
    m.doc() = "Franka Emika Panda robot bindings for Python. This is a simple example";
    py::class_<franka::Robot>(m, "Robot", "Maintains a network connection to the robot, provides the current robot state, and allows execution of commands")
      .def(py::init<const std::string&>(), "Connect to a robot at the given IP/hostname")
      .def("stop", &franka::Robot::stop, "Stop the robot")
      .def("read_once", &franka::Robot::readOnce, "Read robot state once and return a RobotState object")
      .def("read", &franka::Robot::read, "Read robot state continuously with a callback function");      //.def("get_joint_positions", &robot::read, "Get the joint states")
    
    m.def("set_default_behavior", &setDefaultBehavior, "Set default behavior");

    py::class_<franka::RobotState>(m, "RobotState", "Get the Stateoftherobot")
      .def(py::init<>())
      .def_readonly("q", &franka::RobotState::q)
      .def_readonly("q_d", &franka::RobotState::q_d)
      .def_readonly("dq", &franka::RobotState::dq)
      .def_readonly("dq_d", &franka::RobotState::dq_d)
      .def_readonly("ddq_d", &franka::RobotState::ddq_d);
    //py::class_<franka::ActiveControl
    m.def("move_to_joint_position", &moveToJointPosition, "Move to joint position");

    py::class_<PDController>(m, "PDController")
      .def(py::init<franka::Robot&, franka::Gripper&, const Eigen::Matrix<double, 7, 1>&>(), "Initialize PD controller")
      .def("start", &PDController::start, "Start the PD controller")
      .def("stop", &PDController::stop, "Stop the PD controller")
      .def("update_target", &PDController::updateTarget, "Update target joint position")
      .def("close_gripper", &PDController::closeGripper, "Close the gripper")
      .def("open_gripper", &PDController::openGripper, "Open the gripper");
      
    // Gripper bindings
    py::class_<franka::GripperState>(m, "GripperState", "State of the gripper")
      .def(py::init<>())
      .def_readonly("width", &franka::GripperState::width, "Current gripper opening width in meters")
      .def_readonly("max_width", &franka::GripperState::max_width, "Maximum gripper opening width in meters")
      .def_readonly("is_grasped", &franka::GripperState::is_grasped, "Indicates whether an object is currently grasped")
      .def_readonly("temperature", &franka::GripperState::temperature, "Current gripper temperature in degrees Celsius")
      .def_readonly("time", &franka::GripperState::time, "Timestamp (Duration object) since robot start");
      
    py::class_<franka::Gripper>(m, "Gripper", "Maintains a network connection to the gripper and allows execution of commands")
      .def(py::init<const std::string&>(), "Connect to a gripper at the given IP/hostname")
      .def("homing", &franka::Gripper::homing, "Performs homing of the gripper")
      .def("grasp", &franka::Gripper::grasp, 
           "Grasps an object",
           py::arg("width"), py::arg("speed"), py::arg("force"), 
           py::arg("epsilon_inner") = 0.005, py::arg("epsilon_outer") = 0.005)
      .def("move", &franka::Gripper::move, 
           "Moves the gripper fingers to a specified width",
           py::arg("width"), py::arg("speed"))
      .def("stop", &franka::Gripper::stop, "Stops a currently running gripper move or grasp")
      .def("read_once", &franka::Gripper::readOnce, "Waits for a gripper state update and returns it");
  }
  