#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

class RegressionResult {
public:
    bool success;
    std::string message;

    static RegressionResult Pass() { return {true, ""}; }
    static RegressionResult Fail(const std::string& msg) { return {false, msg}; }
};

inline std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        result.push_back(item);
    }
    return result;
}

#endif
