// Copyright (c) 2023 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <mutex>
#include <thread>

#include <Poco/DateTimeFormatter.h>
#include <Poco/File.h>
#include <Poco/Path.h>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <pybind11/gil.h>

#include "examples_common.h"


namespace {
// 
// Forward declaration of the writeLogToFile function
void writeLogToFile(const std::vector<franka::Record>& log);

class Controller {
 public:
  Controller(franka::Robot& robot,
             const Eigen::Matrix<double, 7, 1>& initial_target,
            const Eigen::Matrix<double, 7, 1>& kp) 
  : robot_(robot),
    running_(false),
    franka_robot_model_(robot.loadModel()),
    q_target_(initial_target),
    kp_(kp),
    kd_(2.0 * kp.cwiseSqrt())
  {}

  void start() {
    {   
        std::lock_guard<std::mutex> lock(mutex_);
        if (running_) {
            return;
        }
        running_ = true;
    }

    pybind11::gil_scoped_release release;

    control_thread = std::thread([this]() {
        try {
            // Initialize filtered targets with current joint positions
            franka::RobotState initial_state = robot_.readOnce();
            q_current_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(initial_state.q.data());
            filtered_targets_ = q_current_;
    
            auto callback_control = [this](const franka::RobotState& robot_state,
                                      franka::Duration) -> franka::Torques {
                return this->step(robot_state);
            };
    
            // startTorqueControl() doesn't take any controller mode parameter
            auto active_control = robot_.startTorqueControl();
            while (running_) {
                auto read_once_return = active_control->readOnce();
                auto robot_state = read_once_return.first;
                auto duration = read_once_return.second;
                auto torques = callback_control(robot_state, duration);
                // Print torques for plotting
                std::cout << "Torques: ";
                for (size_t i = 0; i < 7; ++i) {
                  std::cout << torques.tau_J[i];
                  if (i < 6) std::cout << ", ";
                }
                std::cout << std::endl;

                active_control->writeOnce(torques);
            }
    
        } catch (const franka::ControlException& e) {
            std::cout << e.what() << std::endl;
            writeLogToFile(e.log);
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
        }
    });

    //control_thread.detach();
  }

  void stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
  }

 

  // Method to update the target position
  void updateTarget(const Eigen::Matrix<double, 7, 1>& target) {
    std::lock_guard<std::mutex> lock(mutex_);
    q_target_ = target;
  }

 private:
  franka::Robot& robot_;
  std::thread control_thread;
  std::mutex mutex_;
  bool running_;
  franka::Model franka_robot_model_;
  Eigen::Matrix<double, 7, 1> q_target_;
  Eigen::Matrix<double, 2, 1> gripper_state_;
  Eigen::Matrix<double, 7, 1> q_current_, kp_, kd_, filtered_targets_, q_desired, dq_;

  double filter_factor_ = 0.005; // Original was 0.1
  double max_delta_ = 0.1; // Original was 0.1 [rad] ~1.72 degrees
  Eigen::Matrix<double, 7, 1> tau_J_d_M = Eigen::Matrix<double, 7, 1>::Zero();
  const double delta_tau_max_{1};

  // Saturate torque rate to avoid discontinuities
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
      double difference = tau_d_calculated[i] - tau_J_d_M[i];
      tau_d_saturated[i] = tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

   inline franka::Torques step(const franka::RobotState& robot_state) {
    std::array<double, 49> mass = franka_robot_model_.mass(robot_state);
    std::array<double, 7> coriolis_array = franka_robot_model_.coriolis(robot_state);
    std::array<double, 42> jacobian_array = franka_robot_model_.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    
    // Map the arrays to Eigen matrices
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(mass.data());

    q_current_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    dq_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
    
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
    Eigen::Matrix<double, 7, 1> tau_d;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tau_d = kp_.cwiseProduct(position_error) - kd_.cwiseProduct(dq_) + coriolis;
    }

    // Apply torque rate limiting for smooth control regardless of control mode,
    tau_d = saturateTorqueRate(tau_d, tau_J_d_M);

    if (!tau_d.allFinite()) {
      std::cerr << "NaN detected in torque command!" << std::endl;
      this->stop();
    }
    tau_J_d_M = tau_d;

    // Convert Eigen vector to std::array for franka::Torques
    std::array<double, 7> tau_d_array;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;
    // Send joint torque commands to the robot via the command interfaces
    return franka::Torques(tau_d_array);
  }
};

void writeLogToFile(const std::vector<franka::Record>& log) {
  if (log.empty()) {
    return;
  }
  try {
    Poco::Path temp_dir_path(Poco::Path::temp());
    temp_dir_path.pushDirectory("libfranka-logs");

    Poco::File temp_dir(temp_dir_path);
    temp_dir.createDirectories();

    std::string now_string =
        Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
    std::string filename = std::string{"log-" + now_string + ".csv"};
    Poco::File log_file(Poco::Path(temp_dir_path, filename));
    if (!log_file.createFile()) {
      std::cout << "Failed to write log file." << std::endl;
      return;
    }
    std::ofstream log_stream(log_file.path().c_str());
    log_stream << franka::logToCSV(log);

    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}

} // namespace
