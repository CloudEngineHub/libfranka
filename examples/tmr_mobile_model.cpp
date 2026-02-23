// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <iterator>

#include <franka/exception.h>
#include <franka/mobile_model.h>
#include <franka/robot.h>

/**
 * @example tmr_mobile_model.cpp
 * An example showing how to use the MobileModel for a TMR mobile robot.
 *
 * The MobileModel is constructed from the URDF obtained via Robot::getRobotModel().
 * It provides forward kinematics (pose) for the drive module frames.
 */

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);

    if (!robot.isMobileRobot()) {
      std::cerr << "Error: Connected robot is not a mobile robot." << std::endl;
      return -1;
    }

    // Get URDF from the robot and construct the mobile robot model
    std::string urdf = robot.getRobotModel();
    franka::MobileModel model(urdf);

    // Zero configuration for all mobile robot joints
    franka::MobileJointPositions joint_positions{};

    std::cout << "Mobile robot drive module poses at zero configuration:" << std::endl;
    std::cout << "Front drive module: "
              << model.pose(franka::MobileFrame::kFrontDriveModule, joint_positions) << std::endl;
    std::cout << "Rear drive module:  "
              << model.pose(franka::MobileFrame::kRearDriveModule, joint_positions) << std::endl;

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
