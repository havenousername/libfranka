#include <iostream>
#include <cmath>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <vector>
#include <string>
#include <tuple>
#include <array>
#include <chrono>
#include <fstream>
#include "json_utils.h"

// types for the recording
/**
 * PosTuple a type which includes O_T_EE and q fields
 */
using PosTuple = std::tuple<std::array<double, 16>, std::array<double, 7>>;

/**
 * State of the recording
 */
enum State {INIT, END};


int main(int argc, char** argv) {
    // Check whether the required arguments were passed
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

      // Set high collision thresholds to enable hand guiding
      robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
      // Read previous state
      franka::RobotState previous_state = robot.readOnce();
      const double movement_threshold = 1e-2;

      // Initialise variables
      std::vector<PosTuple> trajectory;
      const std::chrono::duration<double> interval(0.01);
      const double duration_increment = 0.001;
      auto start_time = std::chrono::steady_clock::now();
      std::cout << "Started robot free control" << std::endl;
      State state = INIT;
      double duration = 0.0d;
      double total_duration = 5.0d;

      // initialize reading from the robot state callback
      robot.read([&](const franka::RobotState& robot_state) {
        // Check for movement by comparing current state with previous state
        duration += duration_increment;
        double position_difference = 0.0;
        for (size_t i = 0; i < robot_state.q.size(); i++) {
          position_difference += std::pow(robot_state.q[i] - previous_state.q[i], 2);
        }
        position_difference = std::sqrt(position_difference);
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);
        if (elapsed_time > interval) {
          // record trajectory data
          PosTuple posTuple = std::make_tuple(robot_state.O_T_EE, robot_state.q);
          trajectory.push_back(posTuple);
          start_time = current_time;
          previous_state = robot_state;

          // if state is end and the change of position does not happen stop
          if (
            position_difference < movement_threshold &&
            state == END 
          ) {
            return false;
          }
        }

        // stop after total_duration time
        if (duration > total_duration) {
          return false;
        }

        // move to the end state if the movement has started
        if (position_difference > movement_threshold && state == INIT) {
          state = END;
        }

        return true;
      });

      // transform to json string (deserialize) and record trajectory
      std::string json = toJson(trajectory);
      writeToFile("output.json", json);
      std::cout << "Here is the recording size " << trajectory.size() << std::endl;

      // Control loop for free hand guiding
      robot.control([](const franka::RobotState&, franka::Duration) -> franka::Torques {
        // Return zero torques to allow free movement
        return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      });
  } catch (const franka::Exception& ex) {
    // Print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
