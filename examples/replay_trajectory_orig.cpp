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
#include <sstream>

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


int main(int argc, char** argv) {
//   std::string json = R"([{"O_T_EE":[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.2, 0.3, 1.0],"q":[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]},{"O_T_EE":[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.4, 0.5, 0.6, 1.0],"q":[0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]}])";

    try {
        std::string filename = "output.json"; 
        std::string json = readFileToString(filename);
        

        try {
            std::vector<PosTuple> trajectoryVec = parseJson(json);

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
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

            robot.control([&trajectoryVec](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
                if (trajectoryVec.size() == 0) {
                    std::cout << std::endl << "Free movement" << std::endl;
                    return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                }
                
                auto movement = std::get<1>(trajectoryVec.back()); 
                std::cout << "\nq: ";
                for (const auto& val : movement) {
                    std::cout << val << " ";
                }
                std::cout << std::endl;

                std::cout << "Size of the vector " << trajectoryVec.size() << std::endl;
                trajectoryVec.pop_back();                
                return movement;
            });

            // for (const auto& tuple : vec) {
            //     const auto& O_T_EE = std::get<0>(tuple);
            //     const auto& q = std::get<1>(tuple);

            //     std::cout << "O_T_EE: ";
            //     for (const auto& val : O_T_EE) {
            //         std::cout << val << " ";
            //     }
            //     std::cout << "\nq: ";
            //     for (const auto& val : q) {
            //         std::cout << val << " ";
            //     }
            //     std::cout << std::endl;
            // }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    } 
  

  return 0;
}
