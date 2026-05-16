#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

inline bool is_near(int actual, int expected, int tolerance) {
    int diff = actual - expected;
    if (diff < 0) diff = -diff;
    return diff <= tolerance;
}

inline bool is_near(float actual, float expected, float tolerance) {
    float diff = actual - expected;
    if (diff < 0) diff = -diff;
    return diff <= tolerance;
}

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
