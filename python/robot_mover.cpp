#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include "examples_common.h"


// Here i want to exend the robot class with a function to move the robot to a specific joint position
//
// This function will be called from python and will use the franka::Robot class to move the robot
// to a specific joint position
// The function will take a vector of joint positions as input and will use the franka::Robot class
// to move the robot to the specified joint position

// The function will return a boolean value indicating whether the operation was successful or not
// The function will also take a duration as input to specify the time it should take to move to the
// specified joint position

bool move_robot_to_joint_position(franka::Robot& robot, const std::array<double, 7>& joint_positions,
                                     double duration) {
  // Set the joint position to move to
  franka::JointPositions target_joint_positions(joint_positions);
  
  // Set the duration for the motion
  franka::Duration motion_duration(duration);
  
  // Move the robot to the specified joint position
  try {
    robot.moveToJointPosition(target_joint_positions, motion_duration);
    return true;
  } catch (const franka::Exception& e) {
    std::cerr << "Error moving robot: " << e.what() << std::endl;
    return false;
  }
}