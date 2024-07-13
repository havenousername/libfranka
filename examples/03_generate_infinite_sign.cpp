// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <vector>
#include <string>
#include <tuple>
#include <array>
#include <chrono>
#include <fstream>

enum MotionStrategy {
  // use sin to generate motion
  SIN_BASED,
  // use cos to generate motion
  COS_BASED
};

#define PI_2 180

/**
 * Generating infinite sign trajectory infinitely (until timeframe t)
 * @warning Please specify the robot hostname before running the program
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // Initialize robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Initialise variables
    std::array<double, 16> initial_pose;
    double time = 0.0;
    double angle = 0.0;
    constexpr double kRadius = 0.04;
    const double end_time = 36.0d;
    MotionStrategy strategy = SIN_BASED;

    // period of motion for the sin movement
    constexpr double period_x = 20.0d;
    constexpr double period_y = 40.0d;

    constexpr double angle_thickness = 10.0d;

    // control loop callback
    robot.control([&](
                      const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      std::array<double, 16> new_pose = initial_pose;

      // sinus based movement, a repeating cycling movement which results in a infinite shape
      if (strategy === SIN_BASED) {
        double delta_x = kRadius * std::sin(period_x * time*M_PI/PI_2) + initial_pose[12];
        double delta_y = kRadius * (std::sin(period_y * time*M_PI/PI_2)) + initial_pose[13];

        new_pose[12] = delta_x;
        new_pose[13] = delta_y;
      // alternative cos movement, taken as an inspiration from generate_cartesian_pose_motion.cpp file
      } else if (strategy === COS_BASED) {
        angle = M_PI * (1 - std::cos(MPI / angle_thickness * time));

        new_pose[12] = kRadius * std::sin(angle) + initial_pose[12];
        new_pose[13] = kRadius * std::sin(2 * angle) + initial_pose[13];
      }

      // if time has ended finish movement
      if (time >= end_time) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
