// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "examples_common.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <thread>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>  
#include <pybind11/gil.h>
#include <mutex>


void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  //robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  //robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void moveToJointPosition(franka::Robot& robot, const std::array<double, 7>& target,
                     double speed_factor) {
  MotionGenerator motion_generator(speed_factor, target);
  
  robot.control(motion_generator, franka::ControllerMode::kJointImpedance);
  
}

PDController::PDController(franka::Robot& robot, franka::Gripper& gripper, const Eigen::Matrix<double, 7, 1>& start_angles)
  : robot_(robot),
    gripper_(gripper),
    q_target_(start_angles),
    gripper_state_(Eigen::Matrix<double, 2, 1>::Zero()), // Initialize with zeros since we don't have gripper state
    running_(false),
    franka_robot_model_(robot.loadModel()) {
      kp_ << 200, 200, 200, 40, 30, 20, 6;
      kd_ = 2.0 * kp_.cwiseSqrt();  // Critical damping
      controller_ = Eigen::Matrix<double, 7, 1>::Zero();
    }



void PDController::start() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      std::cout << "Control loop is already running!" << std::endl;
      return;  // Prevent multiple invocations
    }   
    // Mark as running
    running_ = true;
  }

  pybind11::gil_scoped_release release;




  // Start the control loop in a background thread
  std::thread control_thread([this]() {
      franka::RobotState initial_state = robot_.readOnce();
      q_current_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        filtered_targets_ = q_current_;
      }

      robot_.control(
          [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
              Eigen::Matrix<double, 7, 1> tau_d = this->controlCallback(robot_state, period);
              
              std::cout << "Torques: [";
              for (int i = 0; i < 7; ++i) {
                  std::cout << tau_d(i);
                  if (i < 6) std::cout << ", ";
              }
              std::cout << "]" << std::endl;


              std::array<double, 7> tau_d_array;
              for (int i = 0; i < 7; ++i) {
                  tau_d_array[i] = tau_d(i);
              }

              {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_) {
                  return franka::MotionFinished(franka::Torques(tau_d_array));
              }

              }
              return franka::Torques(tau_d_array);
          },
          false,  // limit_rate
          franka::kMaxCutoffFrequency  // disable low-pass filter
      );
  });

    control_thread.detach();  // Detach the thread to run in the background
}
    

void PDController::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
}

void PDController::updateTarget(const Eigen::Matrix<double, 7, 1>& angles) {
  std::lock_guard<std::mutex> lock(mutex_);
  q_target_ = angles;
  // Note: We're not updating gripper_state_ anymore since we're only using a 7x1 matrix
}  

 Eigen::Matrix<double, 7, 1>  PDController::controlCallback(const franka::RobotState& robot_state, franka::Duration period) {
  (void) period;  // Unused variable, can be removed if not needed
  // Get robot state data from franka_robot_model
    std::array<double, 49> mass = franka_robot_model_.mass(robot_state);
    std::array<double, 7> coriolis_array = franka_robot_model_.coriolis(robot_state);
    std::array<double, 42> jacobian_array = franka_robot_model_.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    
    // Map the arrays to Eigen matrices
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());

    q_current_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dq_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
    
    // Update joint states using the state interfaces
    // updateJointStates();

    // Initialize torque vector
    Eigen::VectorXd tau_d(7);  
    
    
    // Calculate the filtered target positions
    for (size_t i = 0; i < 7; ++i) {
      // Calculate the delta between policy output and filtered target, policy_joint_positions_ is the target from the policy and filtered_targets_ is the previous target
      double delta;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        delta = q_target_(i) - filtered_targets_(i);
      }

      // Clamp the delta to the maximum allowable range
      delta = std::clamp(delta, -max_delta_, max_delta_);

      // Update the filtered target with the clamped delta -> q_desired_t = q_desired_t-1 + alpha*delta
      filtered_targets_(i) = filtered_targets_(i) + filter_factor_ * delta;
    }

    // Set the desired joint position (always use the latest filtered_targets_)
    q_desired = filtered_targets_;

    // Compute joint position error, q_ is the current joint position obtained from the robot state
    Eigen::Matrix<double, 7, 1> position_error = q_desired - q_current_;

    // Calculate joint torques using PD control law with gravity compensation
    // tau = kp * (q* - q) - kd * q_dot + coriolis + controller
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tau_d = kp_.cwiseProduct(position_error) - kd_.cwiseProduct(dq_) + coriolis + controller_;
    }

    // Apply torque rate limiting for smooth control regardless of control mode,
    tau_d = saturateTorqueRate(tau_d, tau_J_d_M);
    tau_J_d_M = tau_d;

    // Send joint torque commands to the robot via the command interfaces
    return tau_d;
  }

  Eigen::Matrix<double, 7, 1> PDController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    
    for (size_t i = 0; i < 7; i++) {
      double difference = tau_d_calculated[i] - tau_J_d_M[i];
      tau_d_saturated[i] = tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    
    return tau_d_saturated;
  }


void PDController::closeGripper() {
  // Release the GIL during gripper operations to prevent blocking Python
  pybind11::gil_scoped_release release;


  // Run gripper close operation in a separate thread to make it asynchronous
  std::thread gripper_thread([this]() {
    try {
      // Use grasp with small width to close the gripper
      double width = 0.0;  // Minimum width
      double speed = 0.1;   // Speed in m/s
      double force = 60.0;  // Force in N
      gripper_.grasp(width, speed, force);
    } catch (const std::exception& e) {
      std::cerr << "Error during asynchronous gripper close operation: " << e.what() << std::endl;
    }
  });
  gripper_thread.detach();  // Detach thread to run independently
}

void PDController::openGripper() {

  // Release the GIL during gripper operations to prevent blocking Python
  pybind11::gil_scoped_release release;

  // Run gripper open operation in a separate thread to make it asynchronous
  std::thread gripper_thread([this]() {
    try {
      // Use move with maximum width to open the gripper
      // Get the current state to determine max width
      franka::GripperState state = gripper_.readOnce();
      double width = state.max_width;  // Maximum opening width
      double speed = 0.1;              // Speed in m/s
      gripper_.move(width, speed);
    } catch (const std::exception& e) {
      std::cerr << "Error during asynchronous gripper open operation: " << e.what() << std::endl;
    }
  });
  gripper_thread.detach();  // Detach thread to run independently
}








MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
  franka::Duration period) {
time_ += period.toSec();

if (time_ == 0.0) {
q_start_ = Vector7d(robot_state.q.data());
delta_q_ = q_goal_ - q_start_;
calculateSynchronizedValues();
}

Vector7d delta_q_d;
bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

std::array<double, 7> joint_positions;
Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
franka::JointPositions output(joint_positions);
output.motion_finished = motion_finished;
return output;
}