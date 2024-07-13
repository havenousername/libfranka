//
// Created by Andrei Cristea on 13.07.24.
//

#ifndef JSON_UTILS_H
#define JSON_UTILS_H

#include <string>
#include <vector>
#include <tuple>
#include <array>
#include <fstream>
#include <sstream>
#include <franka/exception.h>
#include <iostream>

using PosTuple = std::tuple<std::array<double, 16>, std::array<double, 7>>;

std::string toJson(const std::vector<PosTuple>& vec);
std::vector<double> parseArray(const std::string& json, size_t& pos);
std::vector<PosTuple> parseJson(const std::string& json);

void writeToFile(const std::string& filename, const std::string& data);
std::string readFileToString(const std::string& filename);
std::string trim(const std::string& str);

#endif // JSON_UTILS_H
