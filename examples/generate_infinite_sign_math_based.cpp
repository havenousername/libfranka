// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#define PI 3.1415926535897932384626

/**
 * @example generate_cartesian_pose_motion.cpp
 * An example showing how to generate a Cartesian motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
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

    std::array<double, 16> initial_pose;
    double time = 0.0;
    double angle = 0.0;
    
    //double angle_step = 0.001;
    robot.control([&time, &initial_pose, &angle](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      constexpr double kRadius = 0.04;
      //double angle = M_PI * (1 - std::cos(M_PI / 10 * time));
      // double delta_x = kRadius * std::sin(angle);
      // double delta_z = kRadius * (std::cos(angle) - 1);

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }


      std::array<double, 16> new_pose = initial_pose;
      // new_pose[12] += delta_x;
      // new_pose[14] += delta_z;
      //angle = M_PI * (1 - std::cos(M_PI / 10.0 * time));

      //angle = M_PI * (time - std::sin(M_PI / .0 * time) / (M_PI / (5.0 * time)));
      //double radAngle = angle * PI / 180;
      // double angle = M_PI / 4 * (std::sin(M_PI / 5.0 * time));
      //double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      //new_pose[12] = kRadius * std::sin(angle) + initial_pose[12];
      //ew_pose[13] = kRadius * std::sin(2 * angle) + initial_pose[13];

      // double delta_x = kRadius * std::sin(angle);
      // double delta_y = kRadius * (std::sin(2*angle));

      double delta_x = kRadius * std::sin(20*time*M_PI/180) + initial_pose[12];
      double delta_y = kRadius * (std::sin(40*time*M_PI/180)) + initial_pose[13];

      new_pose[12] = delta_x;
      new_pose[13] = delta_y;

      if (time >= 36.0) {
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
