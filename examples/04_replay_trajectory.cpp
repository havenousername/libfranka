#include <iostream>
#include <cmath>
#include <functional>
#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <vector>
#include <string>
#include <tuple>
#include <array>
#include <chrono>
#include <fstream>
#include <sstream>
#include <queue>

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

std::string readFileToString(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filename);
  }
  
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string trim(const std::string& str) {
  size_t first = str.find_first_not_of(' ');
  if (first == std::string::npos) return "";
  size_t last = str.find_last_not_of(' ');
  return str.substr(first, (last - first + 1));
}

std::vector<double> parseArray(const std::string& json, size_t& pos) {
  std::vector<double> result;
  std::string number;
  bool inNumber = false;

  for (; pos < json.size(); ++pos) {
    char c = json[pos];

    if (c == '[') {
      continue;
    } else if (c == ']') {
      if (!number.empty()) {
        result.push_back(std::stod(number));
      }
      ++pos; // Move past the closing bracket
      break;
    } else if (c == ',') {
      if (!number.empty()) {
        result.push_back(std::stod(number));
        number.clear();
      }
      inNumber = false;
    } else if (std::isdigit(c) || c == '.' || c == '-' || c == 'e' || c == 'E') {
      inNumber = true;
      number += c;
    } else if (std::isspace(c)) {
      if (inNumber) {
        result.push_back(std::stod(number));
        number.clear();
        inNumber = false;
      }
    } else {
      throw std::runtime_error("Unexpected character in array");
    }
  }

  return result;
}

std::vector<PosTuple> parseJson(const std::string& json) {
  std::vector<PosTuple> result;
  size_t pos = 0;

  while (pos < json.size()) {
    if (json[pos] == '{') {
      std::array<double, 16> O_T_EE;
      std::array<double, 7> q;
      bool found_O_T_EE = false, found_q = false;

      while (pos < json.size() && json[pos] != '}') {
        size_t start = json.find('"', pos);
        size_t end = json.find('"', start + 1);
        std::string key = json.substr(start + 1, end - start - 1);
        pos = end + 1;

        if (key == "O_T_EE") {
          while (json[pos] != '[') ++pos;
          std::vector<double> arr = parseArray(json, pos);
          if (arr.size() != 16) {
            throw std::runtime_error("Expected 16 elements in O_T_EE array");
          }
          std::copy(arr.begin(), arr.end(), O_T_EE.begin());
          found_O_T_EE = true;
        } else if (key == "q") {
          while (json[pos] != '[') ++pos;
          std::vector<double> arr = parseArray(json, pos);
          if (arr.size() != 7) {
            throw std::runtime_error("Expected 7 elements in q array");
          }
          std::copy(arr.begin(), arr.end(), q.begin());
          found_q = true;
        }

        while (json[pos] != '}' && json[pos] != '"') ++pos;
      }

      if (found_O_T_EE && found_q) {
        result.emplace_back(O_T_EE, q);
      }
    }
    ++pos;
  }

  return result;
}


constexpr std:string input_file = "output.json";
constexpr std::string out_file = "output-6.json";


int main(int argc, char** argv) {
    try {
        std::string json = readFileToString(input_file);
        try {
            std::vector<PosTuple> trajectoryVec = parseJson(json);
            std::queue<PosTuple> trajectoryQ;

            // Compliance parameters, set up impedance controller
            const double translational_stiffness{150.0};
            const double rotational_stiffness{10.0};
            Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
            stiffness.setZero();
            stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
            stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
            damping.setZero();
            damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                                Eigen::MatrixXd::Identity(3, 3);
            damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                                    Eigen::MatrixXd::Identity(3, 3);

            // transform vector to the queue (to pop from top to bottom movements)
            for (auto v : trajectoryVec) {
                trajectoryQ.push(v);
            }

            // initialize robot
            franka::Robot robot(argv[1]);
            setDefaultBehavior(robot);

            // Set high collision thresholds to enable hand guiding
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
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            std::vector<PosTuple> trajectory; 
            bool hasEntered = false;

            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(
                initial_state.O_T_EE.data()
            ));


            // define callback for the torque control loop
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback = [&](
                                                 const franka::RobotState& robot_state,
                                                 franka::Duration) -> franka::Torques {
                PosTuple posTuple = std::make_tuple(robot_state.O_T_EE, robot_state.q);
                trajectory.push_back(posTuple);
                Eigen::Map<Eigen::Matrix4d> desired_input(std::get<0>(trajectoryQ.front()).data());
                Eigen::Vector3d position_d(desired_input.block<3, 1>(0, 3));
                Eigen::Quaterniond orientation_d(desired_input.block<3, 3>(0, 0));

                if (!trajectoryQ.empty()) {
                    trajectoryQ.pop();
                } else if (!hasEntered) {
                    std::string json = toJson(trajectory);
                    writeToFile(out_file, json);
                    hasEntered = true;
                }

                if (hasEntered) {
                     return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                }
                
                // get state variables
                std::array<double, 7> coriolis_array = model.coriolis(robot_state);
                std::array<double, 42> jacobian_array =
                    model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

                // convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

                Eigen::Affine3d transform;
                transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
                Eigen::Vector3d position(transform.translation());
                Eigen::Quaterniond orientation(transform.rotation());

                // compute error to desired equilibrium pose
                // position error
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position - position_d;

                // orientation error
                // "difference" quaternion
                if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                    orientation.coeffs() << -orientation.coeffs();
                }
                // "difference" quaternion
                Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
                error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                // Transform to base frame
                error.tail(3) << transform.rotation() * error.tail(3);

                // compute control
                Eigen::VectorXd tau_task(7), tau_d(7);

                // Spring damper system with damping ratio=1
                tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
                tau_d << tau_task + coriolis;

                std::array<double, 7> tau_d_array{};
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
                return tau_d_array;
            };
            

            // start real-time control loop
            std::cout << "WARNING: Collision thresholds are set to high values. "
                    << "Make sure you have the user stop at hand!" << std::endl
                    << "After starting try to push the robot and see how it reacts." << std::endl
                    << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(impedance_control_callback);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    } 
  

  return 0;
}
