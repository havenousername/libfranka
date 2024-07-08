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

using PosTuple = std::tuple<std::array<double, 16>, std::array<double, 7>>;
enum State {INIT, END};


std::string toJson(const std::vector<PosTuple>& vec) {
  std::string json = "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    const auto& arr1 = std::get<0>(vec[i]);
    const auto& arr2 = std::get<1>(vec[i]);

    json += "{";
    // O_T_EE, robot_state.q/
    json += "\"O_T_EE\":[";
    for (size_t j = 0; j < arr1.size(); ++j) {
      json += std::to_string(arr1[j]);
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


int main(int argc, char** argv) {
  // Check whether the required arguments were passed
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

      // Set high collision thresholds to enable hand guiding
      robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

      franka::RobotState previous_state = robot.readOnce();
      const double movement_threshold = 1e-2;
      std::vector<PosTuple> trajectory; 
      const std::chrono::duration<double> interval(0.01);
      auto start_time = std::chrono::steady_clock::now();
      std::cout << "Started robot free control" << std::endl;
      State state = INIT;
      double duration = 0.0d;
      robot.read([&previous_state, &duration, &state, &start_time, interval, movement_threshold, &trajectory](const franka::RobotState& robot_state) {
        // Check for movement by comparing current state with previous state
        duration += 0.001;
        double position_difference = 0.0;
        for (size_t i = 0; i < robot_state.q.size(); i++) {
          position_difference += std::pow(robot_state.q[i] - previous_state.q[i], 2);
        }
        position_difference = std::sqrt(position_difference);
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);
        if (elapsed_time > interval) {
          PosTuple posTuple = std::make_tuple(robot_state.O_T_EE, robot_state.q);
          trajectory.push_back(posTuple);
          start_time = current_time;
          previous_state = robot_state;

          if (
            position_difference < movement_threshold &&
            state == END 
          ) {
            return false;
          }
        }

        if (duration > 5.0) {
          return false;
        }


        if (position_difference > movement_threshold && state == INIT) {
          state = END;
        }
    

        return true;
      });

      std::string json = toJson(trajectory);
      writeToFile("output.json", json);



      // std::cout << "Here is the recording size " << trajectory[] << std::endl;


      std::cout << "Here is the recording size " << trajectory.size() << std::endl;


      // Control loop for hand guiding
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


// #include <iostream>

// #include <cmath>

// #include <franka/exception.h>
// #include <franka/robot.h>

// #include "examples_common.h"

// #include <iostream>
// #include <franka/exception.h>
// #include <franka/robot.h>

// int main(int argc, char** argv) {
//   // Check whether the required arguments were passed
//   if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//     return -1;
//   }

//   try {
//     // Connect to the robot
//     franka::Robot robot(argv[1]);
//     setDefaultBehavior(robot);

//     // Set high collision thresholds to enable hand guiding
//     robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//     std::cout << "WARNING: Collision thresholds are set to high values. "
//               << "Make sure you have the user stop at hand!" << std::endl
//               << "After starting try to push the robot and see how it reacts." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     size_t count = 0;
//     // Control loop for hand guiding and printing robot state
//     robot.control([&count](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
//       // Print the robot's state
//       std::cout << "Robot state iteration " << count << ":" << std::endl;
//       std::cout << "q: ";
//       for (const auto& q_val : robot_state.q) {
//         std::cout << q_val << " ";
//       }
//       std::cout << std::endl;

//       std::cout << "dq: ";
//       for (const auto& dq_val : robot_state.dq) {
//         std::cout << dq_val << " ";
//       }
//       std::cout << std::endl;

//       std::cout << "O_T_EE: ";
//       for (const auto& val : robot_state.O_T_EE) {
//         std::cout << val << " ";
//       }
//       std::cout << std::endl;

//       std::cout << "----------------------------" << std::endl;

//       // Continue printing for the first 100 movements
//       if (count++ < 100) {
//         return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//       } else {
//         throw franka::ControlException("Stopped after 100 iterations");
//       }
//     });

//   } catch (const franka::Exception& ex) {
//     // Print exception
//     std::cout << ex.what() << std::endl;
//   }

//   return 0;
// }














// int main(int argc, char** argv) {
//   // Check whether the required arguments were passed
//   if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//     return -1;
//   }

//   try {
//     // Connect to the robot
//     franka::Robot robot(argv[1]);int main(int argc, char** argv) {
//   // Check whether the required arguments were passed
//   if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//     return -1;
//   }

//   try {
//     // Connect to the robot
//     franka::Robot robot(argv[1]);
//     setDefaultBehavior(robot);

//     // Set high collision thresholds to enable hand guiding
//     robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//     std::cout << "WARNING: Collision thresholds are set to high values. "
//               << "Make sure you have the user stop at hand!" << std::endl
//               << "After starting try to push the robot and see how it reacts." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     // Empty control loop for hand guiding
//     robot.control([](const franka::RobotState&, franka::Duration) -> franka::Torques {
//       // Return zero torques to allow free movement
//       return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     });




//   } catch (const franka::Exception& ex) {
//     // Print exception
//     std::cout << ex.what() << std::endl;
//   }

//   return 0;
// }


//     setDefaultBehavior(robot);

//     // Set high collision thresholds to enable hand guiding
//     robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//     std::cout << "WARNING: Collision thresholds are set to high values. "
//               << "Make sure you have the user stop at hand!" << std::endl
//               << "After starting try to push the robot and see how it reacts." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     // Empty control loop for hand guiding
//     robot.control([](const franka::RobotState&, franka::Duration) -> franka::Torques {
//       // Return zero torques to allow free movement
//       return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     });




//   } catch (const franka::Exception& ex) {
//     // Print exception
//     std::cout << ex.what() << std::endl;
//   }

//   return 0;
// }






// #include <array>
// #include <cmath>
// #include <functional>
// #include <iostream>

// #include <Eigen/Dense>

// #include <franka/duration.h>
// #include <franka/exception.h>
// #include <franka/model.h>
// #include <franka/robot.h>

// #include "examples_common.h"
// /**
//  * @example something.cpp
//  * An example for an arbitrary file step.1
//  */

// int main(int argc, char** argv) {
//    // connect to robot
//     franka::Robot robot(argv[1]);
//      // load the kinematics and dynamics model
//     franka::Model model = robot.loadModel();
//     franka::RobotState initial_state = robot.readOnce();


//      // start real-time control loop
//     std::cout << "WARNING: Collision thresholds are set to high values. "
//               << "Make sure you have the user stop at hand!" << std::endl
//               << "After starting try to push the robot and see how it reacts." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
    
//      std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
//         impedance_control_callback = [&](const franka::RobotState& robot_state,
//                                          franka::Duration /*duration*/) -> franka::Torques {
//     return {};
//     };

//     robot.control(impedance_control_callback);




// //     size_t count = 0;
// // robot.read([&count](const franka::RobotState& robot_state) {
// //   // Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
// //   // but should not be done in a control loop.
// //   std::cout << robot_state << std::endl;
// //   return count++ < 100;
// // });

//   return 0;
// }

