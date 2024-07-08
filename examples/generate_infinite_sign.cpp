// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#define PI 3.1415926535897932384626
#include <vector>
#include <string>
#include <tuple>
#include <array>
#include <chrono>
#include <fstream>

using PosTuple = std::tuple<std::array<double, 16>, std::array<double, 7>>;
enum State {INIT, END};


std::string toJson(const  std::vector<PosTuple>& vec) {
  std::string json = "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    const auto& arr1 = std::get<0>(vec[i]);
    const auto& arr2 = std::get<1>(vec[i]);

    json += "{";
    // O_T_EE, robot_state.q/
    json += "\"O_T_EE\":[";
    for (size_t j = 0; j < arr1.size(); ++j) {
      json += std::to_string(arr1[j]);tput-5.
      if (j < arr1.size() - 1) {
        json += ",";
      }
    }
    json += "],";

    json += "\"q\":[";
    for (size_t j = 0; j < arr2.size(); ++j) {
      json += std::to_string(arr2[j]);
      if (j < arr2.size() - 1) {
        json += ",";
      }
    }
    json += "]";

    json += "}";
    if (i < vec.size() - 1) {
      json += ",";
    }
  }
  json += "]";
  return json;
}

void writeToFile(const std::string& filename, const std::string& data) {
  std::ofstream file(filename);
  if (file.is_open()) {
    std::cout << "Write to the file " << filename << std::endl;
    file << data;
    file.close();
  } else {
    std::cerr << "Unable to open file";
  }
}

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
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }


      std::array<double, 16> new_pose = initial_pose;

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
