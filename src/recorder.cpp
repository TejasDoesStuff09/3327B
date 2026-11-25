#include "recorder.hpp"
#include "pros/rtos.hpp"
#include <cstdio>
#include <cmath>
#include <cstdint>

//sampling interval=10 milliseconds
constexpr int SAMPLE_INTERVAL_MS=10;

//constructor
Recorder::Recorder(
    std::function<void(double)> setVoltage,
    std::function<double()> getVelocity,
    std::function<FieldPoint()> getCurrentPosition
) {
    this->setVoltage=setVoltage;
    this->getVelocity=getVelocity;
    this->getCurrentPosition=getCurrentPosition;
}

//record one sample
void Recorder::record(int run, double voltage){
    FieldPoint position=getCurrentPosition();
    double time=(pros::millis()-runStartMS)/1000.0;

    Sample sample;
    sample.run=run;
    sample.t_s=time;
    sample.voltage_v=voltage;
    sample.velocity_in_s=getVelocity();
  
    sample.position_in=sqrt(position.x*position.x+position.y*position.y);

    samples.push_back(sample);
}

//records one sample and waits 10 milliseconds
void Recorder::recordSample(int run, double voltage){
    record(run, voltage);
    pros::delay(SAMPLE_INTERVAL_MS);
}

//clears all previous data
void Recorder::clearData(){
    samples.clear();
    runCounter=0;
}

//holding voltage while recording
void Recorder::holdVoltage(int run, double voltage, double timeDuration){
    setVoltage(voltage);
    uint32_t startTime=pros::millis();

    while(pros::millis()-startTime<timeDuration*1000){
        recordSample(run, voltage);
    }
  
    setVoltage(0);
}

//baseline sets voltage to 0 V
void Recorder::baseline(int run, double direction){
    (void)direction;
    setVoltage(0);
  
    uint32_t startTime=pros::millis();

    while(pros::millis()-startTime<1000){
        recordSample(run,0);
    }
}

//ramp test: going 0 V to 6 V at approximately 1 V/s
void Recorder::rampTest(int run, double direction){
    setVoltage(0);

    for(double voltage=0; voltage<=6.0; voltage+=0.01){
        double neededVoltage=voltage*direction;

        setVoltage(neededVoltage);
        recordSample(run, neededVoltage);
    }

    setVoltage(0);
}

//step test: going directly to a certain voltage and holding it for 1.5 seconds
void Recorder::stepTest(int run, double voltage, double direction){
    holdVoltage(run, voltage*direction, 1.5);
}

//running one entire test (one direction)
void Recorder::runTest(const std::string& name, double direction){
    runCounter++;
    int currentRun=runCounter;

    //resetting time
    runStartMS=pros::millis();
    printf("Starting run %d: %s\n", currentRun, name.c_str());

    //starting with baseline
    baseline(currentRun, direction);

    //ramp test
    rampTest(currentRun, direction);

    //two step tests
    stepTest(currentRun, 4.0, direction);
    stepTest(currentRun, 8.0, direction);

    //0 voltage for one second as the last part of the run
    baseline(currentRun, direction);
    setVoltage(0);
}

//running all tests
void Recorder::runAllTests(int repetitions, bool forwAndBack){
    clearData();
    for(int round=0; round<repetitions; round++){
        printf("Repetition %d of %d\n", round+1, repetitions);

        //forward
        runTest("Forward", 1.0);

        //reverse runs if forwAndBack is true
        if(forwAndBack){
            runTest("Reverse", -1.0);
        }
    }

    setVoltage(0);
    printf("All tests complete!\n");

    printCSV();
}

//printing all collected data
void Recorder::printCSV(){
    printf("run,t_s,voltage_v,velocity_in_s,position_in\n");

    for(const Sample& sample:samples){
        printf("%d,%.3f,%.3f,%.3f,%.3f\n",
            sample.run,
            sample.t_s,
            sample.voltage_v,
            sample.velocity_in_s,
            sample.position_in);
    }
}
