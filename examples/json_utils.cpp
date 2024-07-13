//
// Created by Andrei Cristea on 13.07.24.
//

#include "json_utils.h"

std::string toJson(const std::vector<PosTuple>& vec) {
  std::string json = "[";
  for (size_t i = 0; i < vec.size(); ++i) {
    const auto& arr1 = std::get<0>(vec[i]);
    const auto& arr2 = std::get<1>(vec[i]);

    json += "{";
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