#pragma once

#include <vector>
#include <functional>
#include <string>
#include <cstdint>

#include "snapper.hpp"

struct Sample{
    int run;
    double t_s;
    double voltage_v;
    double velocity_in_s;
    double position_in;
};

class Recorder{
private:
    std::vector<Sample> samples;

    //Methods
    std::function<void(double)> setVoltage;
    std::function<double()> getVelocity;
    std::function<FieldPoint()> getCurrentPosition;

    int runCounter=0;
    uint32_t runStartMS=0;

    void recordSample(int run, double voltage);

    //Runs one test
    void runTest(const std::string& name, double direction);

    //All individual tests
    void baseline(int run, double direction);
    void rampTest(int run, double direction);
    void stepTest(int run, double voltage, double direction);

    //Continuously records
    void holdVoltage(int run, double voltage, double timeDuration);

public:
    Recorder(
        std::function<void(double)> setVoltage,
        std::function<double()> getVelocity,
        std::function<FieldPoint()> getCurrentPosition
    );

    //Records one sample
    void record(int run, double voltage);

    //Runs all tests
    void runAllTests(int repetitions, bool forwardAndBackward);

    //Prints all recorded data
    void printCSV();

    //Clears previous data
    void clearData();
};
